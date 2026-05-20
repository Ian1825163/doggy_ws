#include <Arduino.h>
#include <math.h>
#include <IntervalTimer.h>
#include <stdlib.h>
#include <string.h>


struct Vec3 {
  double x, y, z;
};
class Leg;  


// -----------------------------
//  Config
// -----------------------------
static constexpr uint32_t BAUD_SERVO = 1000000;
static constexpr uint32_t BAUD_USB   = 115200;

// ROS bridge mode:
//   Jetson -> Teensy: $ANGLES,<seq>,d1,d2,...,d12\n
//   Teensy -> Jetson: $ACK,<seq>,1\n and $HB,<seq>\n
static constexpr bool ENABLE_ROS_BRIDGE = true;
static constexpr uint32_t HEARTBEAT_MS = 100;          // 10 Hz
static constexpr uint32_t COMMAND_TIMEOUT_MS = 500;    // fall back to stand
static constexpr uint32_t FAILSAFE_STAND_MS = 100;     // resend stand at 10 Hz


static constexpr uint16_t CENTER_TICK = 2048;   // STS 12-bit center
static constexpr uint16_t GOAL_SPEED  = 1000;   // 0x03E8
static constexpr uint16_t GOAL_TIME   = 200;    // don't use 0 at first (gentler)


static constexpr bool ENABLE_TIMER = true;     // false: test stable position; true for real time control
static constexpr bool ENABLE_TRAJ  = true;    
static constexpr uint32_t CTRL_US  = 20000;     // 20ms => 50Hz


// Angle safety clamp (deg)
static constexpr double LIMIT_DEG = 70.0;


const double T = 2.0;
const double omega = 2.0 * M_PI / T;
const double lift = 10.0;
const double disp = 30.0;


//gait control parameter
static double phase = 0.0;
const double phaseRate = 1.0 / T;   // 1/T cycle per second




// -----------------------------
//  Utils
// -----------------------------
inline double clampd(double v, double lo, double hi){
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline double deg2rad(double d){ return d * M_PI / 180.0; }
inline double rad2deg(double r){ return r * 180.0 / M_PI; }


inline int16_t wrap4096(int32_t v){
  v %= 4096;
  if(v < 0) v += 4096;
  return (int16_t)v;
}


// rad -> tick offset (can be negative)
inline int32_t rad2tick_offset(double rad){
  return (int32_t)lround(rad * (4096.0 / (2.0*M_PI)));
}


// center + rad (safe wrap 0..4095)
inline uint16_t centerPlusRad(double rad){
  int32_t goal = (int32_t)CENTER_TICK + rad2tick_offset(rad);
  return (uint16_t)wrap4096(goal);
}


inline double wrapPi(double a){
  while(a <= -M_PI) a += 2.0*M_PI;
  while(a >   M_PI) a -= 2.0*M_PI;
  return a;
}
inline double wrap01(double p){
  p = fmod(p, 1.0);
  if(p < 0) p+=1;
  return p;
}
inline double angDiff(double a, double b){
  return wrapPi(a - b);
}




// -----------------------------
//  Debug shared (main loop prints)
// -----------------------------
volatile double debug_x = 0, debug_y = 0, debug_z = 0;
volatile double debug_a1 = 0, debug_a2 = 0, debug_a3 = 0;


unsigned long lastPrintTime = 0;


// -----------------------------
//  Leg class (你的 IK 保留，微調安全)
// -----------------------------
class Leg {
public:
  bool inited_c;
  double last_c_local;


    // --- NEW: FK 4-bar state (theta4 root memory) ---
  bool inited_theta4;
  int  last_theta4_choice;   // 1 or 2


  // --- NEW: IK branch selection state (c1/c2 hysteresis) ---
  bool inited_pick;
  int  last_pick;            // 1=c1, 2=c2




  const double L1;
  const int L2;
  const int L3;
  const int L4;
  const double h;
  const int r;


  // NEW: FK params
  const int R3;
  const int R4;


  double angle1;
  double angle2;
  double angle3;
  double pos[3];


  double z_on;
  double z_off;


  bool use_c2;


  Leg();
  void setTargetPosition(double x, double y, double z);
  void setTargetAngle(double a1, double a2, double a3);
  void computeIK();
    // --- NEW: FK + selection ---
  Vec3 fkEndEffector(double alp, double bet, double gamm, bool do_update);
  void computeIK_FKSelect();      


};


void lift_trajectory(Leg& leg, double t);
void sinusoidal_trajectory(Leg& leg, double p, double duty);
void standing_pose(Leg& leg);
void squat_pose(Leg& leg);


void trot(Leg& RB, Leg& RF, Leg& LB, Leg& LF, double phase);
void walk(Leg& RB, Leg& RF, Leg& LB, Leg& LF, double phase);
void lean(Leg& RB, Leg& RF, Leg& LB, Leg& LF);


void sendMotorPacketDeg(double d1, double d2, double d3,
                        double d4, double d5, double d6,
                        double d7, double d8, double d9,
                        double d10,double d11,double d12);
void sendMotorPacketDegArray(const double deg[12]);
void sendStandPose();
void handleUsbSerial();
void handleCommandLine(const char* line);
bool parseAnglesLine(const char* line, uint32_t& seq, double deg[12]);
uint32_t parseSeqBestEffort(const char* line);
void sendAck(uint32_t seq, bool ok);
void sendHeartbeat();




Leg::Leg()
  : inited_c(false),
    last_c_local(0.0),    
    inited_theta4(false),
    last_theta4_choice(1),
    inited_pick(false),
    last_pick(1),
    L1(26.086),
    L2(80),
    L3(80),
    L4(20),
    h(28.5),
    r(22),
    R3(20),
    R4(80),


    angle1(M_PI/2),
    angle2(M_PI/2),
    angle3(M_PI/2),
    z_on(-123.0),
    z_off(-123.0),
    use_c2(false)
   
{
  pos[0] = 26.086;
  pos[1] = 0.0;
  pos[2] = -155.0;
}


void Leg::setTargetPosition(double x, double y, double z){
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
}


void Leg::computeIK(){
  double x = pos[0], y = pos[1], z = pos[2];


  double l2 = x*x + y*y + z*z;
  double l  = sqrt(l2);
  if(l < 1e-6) l = 1e-6;


  double d2 = l2 - L1*L1;
  if(d2 < 0) d2 = 0;
  double d  = sqrt(d2);
  if(d < 1e-6) d = 1e-6;


  // knee geometry
  double cosK = (L1*L1 + (double)L2*L2 + (double)L3*L3 - l2) / (2.0*L2*L3);
  cosK = clampd(cosK, -1.0, 1.0);
  double theta_k = acos(cosK);


  // theta_b
  double s1 = clampd(y/d, -1.0, 1.0);
  double cos2 = ((double)L2*L2 + d*d - (double)L3*L3) / (2.0*L2*d);
  cos2 = clampd(cos2, -1.0, 1.0);
  double theta_b = M_PI/2.0 - (asin(s1) + acos(cos2));


  // theta_a
  double ca = clampd(L1 / l, -1.0, 1.0);
  double theta_a = atan2(x, fabs(z)) + acos(ca) - M_PI/2.0;


  // theta_c (your linkage)
  double A = 2.0*r*(L4*cos(theta_k) + L2);
  double B = 6.0*r;
  double C = (double)r*r + pow((double)L2 + L4*cos(theta_k), 2)
           + 9.0 + pow(L4*sin(theta_k) - h, 2) - (double)L3*L3;


  double disc = (A*B)*(A*B) - (A*A - C*C)*(B*B - C*C);
  double sq = sqrt(fmax(disc, 0.0));


  double Y1 = -A*B + sq;
  double Y2 = -A*B - sq;
  double X  = A*A - C*C;
  double c1 = wrapPi(atan2(Y1, X));
  double c2 = wrapPi(atan2(Y2, X));


  // adjust the new anhle value to be neraest to ref (+-2pi）, avoid jumping sol
  auto unwrapToNear = [&](double a, double ref){
      return ref + angDiff(a, ref);   // angDiff : (-pi,pi]
  };
  double z_now = z; //get z value


  if(!this->inited_c){
      // determine first theta_c
      if(z_now >= this->z_on) this->last_c_local = c1;
      else {
        this->use_c2 = true;
        this->last_c_local = c2;
      }
      this->inited_c = true;
  }
  else {
      double c1u = unwrapToNear(c1, this->last_c_local);
      double c2u = unwrapToNear(c2, this->last_c_local);


      if(this->use_c2){
          if(z_now >= this->z_off) {
            this->use_c2 = false;
            this->last_c_local = c1u;
          }
          else this->last_c_local = c2u;
          }
      else {
        if(z_now >= this->z_on){
          double err1 = fabs(c1u - this->last_c_local);
          double err2 = fabs(c2u - this->last_c_local);
          err1 = min(err1, 2*M_PI - err1);
          err2 = min(err2, 2*M_PI - err2);
          if (err1 <= err2)
              this->last_c_local = c1u;
          else
              this->last_c_local = c2u;


        }
        else {
          this->last_c_local = c2u;
          this->use_c2 = true;


        }
      }
  }


  double chosen = this->last_c_local;


  // --- 5. Store Final Angles ---
  this->angle1 = wrapPi(theta_a);
  this->angle2 = wrapPi(theta_b);
  this->angle3 = chosen;   // wrap to (-pi,pi)


}
// apply R = Ry(alp) * Rx(bet)
static inline Vec3 R_apply(double alp, double bet, const Vec3& v){
  double cb = cos(bet), sb = sin(bet);
  // Rx(bet)
  Vec3 v1 { v.x, cb*v.y + sb*v.z, -sb*v.y + cb*v.z };


  double ca = cos(alp), sa = sin(alp);
  // Ry(alp)
  Vec3 v2 { ca*v1.x - sa*v1.z, v1.y, sa*v1.x + ca*v1.z };
  return v2;
}


Vec3 Leg::fkEndEffector(double alp, double bet, double gamm, bool do_update){
  // --- theta4 candidates (same as MATLAB) ---
  double a_ = r * sin(gamm);
  double R1_ = sqrt(a_*a_ + h*h);
  double R2_ = sqrt((double)R4*R4 - pow(r*cos(gamm) - 3.0, 2)); // R2 = sqrt(R4^2-(r*cos(g)-3)^2)
  double theta1 = atan2(h, a_);


  double A = 2.0*R1_*R3*sin(theta1);
  double B = 2.0*R1_*R3*cos(theta1) - 2.0*R3*R4;
  double C = R1_*R1_ - R2_*R2_ + (double)R3*R3 + (double)R4*R4 - 2.0*R1_*R4*cos(theta1);


  double disc = C*C * (A*A + B*B - C*C);
  disc = fmax(disc, 0.0);
  double sq = sqrt(disc);


  double th4_1 = atan2(-A*B + sq, A*A - C*C);
  double th4_2 = atan2(-A*B - sq, A*A - C*C);


  // --- error metric (distance should be L3=80) ---
  // B_local_check = [r*cos(g); r*sin(g); h]
  double Bx = r*cos(gamm);
  double By = r*sin(gamm);
  double Bz = h;


  // C1_local = [3; R4 + R3*cos(th4); R3*sin(th4)]
  auto distErr = [&](double th4){
    double C1x = 3.0;
    double C1y = (double)R4 + (double)R3 * cos(th4);
    double C1z = (double)R3 * sin(th4);
    double dx = Bx - C1x, dy = By - C1y, dz = Bz - C1z;
    double d = sqrt(dx*dx + dy*dy + dz*dz);
    return fabs(d - (double)L3);
  };


  double e1 = distErr(th4_1);
  double e2 = distErr(th4_2);


  // --- hysteresis keep continuity (same spirit as MATLAB) ---
  if(!inited_theta4){
    last_theta4_choice = (e1 <= e2) ? 1 : 2;
    inited_theta4 = true;
  }else{
    const double threshold = 0.4;
    if(last_theta4_choice == 1){
      if(e2 < e1 - threshold) last_theta4_choice = 2;
    }else{
      if(e1 < e2 - threshold) last_theta4_choice = 1;
    }
  }


  // if evaluating only, DO NOT update memory
  int choice = last_theta4_choice;
  if(!do_update){
    // compute what choice would be, without changing stored
    choice = (e1 <= e2) ? 1 : 2;
    const double threshold = 0.4;
    if(last_theta4_choice == 1){
      if(e2 < e1 - threshold) choice = 2;
      else choice = 1;
    }else{
      if(e1 < e2 - threshold) choice = 1;
      else choice = 2;
    }
  }


  double th4 = (choice == 1) ? th4_1 : th4_2;


  // if do_update=true, commit
  if(do_update){
    last_theta4_choice = choice;
  }


  // --- compute E_local and transform to global ---
  // D_local = [0; R4; 0]
  // E_local = D_local - [0; R4*cos(th4); R4*sin(th4)] = [0; R4*(1-cos); -R4*sin]
  Vec3 E_local { 0.0, (double)R4 * (1.0 - cos(th4)), -(double)R4 * sin(th4) };


  // sub-assembly origin offset: p0 = R_apply(alp,bet, [L1,0,0])
  Vec3 p0 = R_apply(alp, bet, {L1, 0.0, 0.0});


  // E_global = p0 + R_apply(alp,bet, E_local)
  Vec3 Eg = R_apply(alp, bet, E_local);
  Eg.x += p0.x; Eg.y += p0.y; Eg.z += p0.z;


  return Eg;
}
void Leg::computeIK_FKSelect(){
  double x = pos[0], y = pos[1], z = pos[2];


  // ----- IK a,b,k (same as your computeIK) -----
  double l2 = x*x + y*y + z*z;
  double l  = sqrt(l2);
  if(l < 1e-6) l = 1e-6;


  double d2 = l2 - L1*L1;
  if(d2 < 0) d2 = 0;
  double d  = sqrt(d2);
  if(d < 1e-6) d = 1e-6;


  double cosK = (L1*L1 + (double)L2*L2 + (double)L3*L3 - l2) / (2.0*L2*L3);
  cosK = clampd(cosK, -1.0, 1.0);
  double theta_k = acos(cosK);


  double s1 = clampd(y/d, -1.0, 1.0);
  double cos2 = ((double)L2*L2 + d*d - (double)L3*L3) / (2.0*L2*d);
  cos2 = clampd(cos2, -1.0, 1.0);
  double theta_b = M_PI/2.0 - (asin(s1) + acos(cos2));


  double ca = clampd(L1 / l, -1.0, 1.0);
  double theta_a = atan2(x, fabs(z)) + acos(ca) - M_PI/2.0;


  // ----- IK theta_c roots -----
  double A = 2.0*r*(L4*cos(theta_k) + L2);
  double B = 6.0*r;
  double C = (double)r*r + pow((double)L2 + L4*cos(theta_k), 2)
           + 9.0 + pow(L4*sin(theta_k) - h, 2) - (double)L3*L3;


  double disc = (A*B)*(A*B) - (A*A - C*C)*(B*B - C*C);
  double sq = sqrt(fmax(disc, 0.0));


  double Y1 = -A*B + sq;
  double Y2 = -A*B - sq;
  double X  = A*A - C*C;


  double c1 = wrapPi(atan2(Y1, X));
  double c2 = wrapPi(atan2(Y2, X));


  // unwrap near last_c_local for continuity
  auto unwrapToNear = [&](double a, double ref){
    return ref + angDiff(a, ref);
  };


  double c1u = c1, c2u = c2;
  if(inited_c){
    c1u = unwrapToNear(c1, last_c_local);
    c2u = unwrapToNear(c2, last_c_local);
  }


  // ----- evaluate both by FK end-effector error -----
  Vec3 E1 = fkEndEffector(theta_a, theta_b, c1u, false);
  Vec3 E2 = fkEndEffector(theta_a, theta_b, c2u, false);


  auto norm3 = [](double dx,double dy,double dz){
    return sqrt(dx*dx + dy*dy + dz*dz);
  };


  double pos_err1 = norm3(E1.x-x, E1.y-y, E1.z-z);
  double pos_err2 = norm3(E2.x-x, E2.y-y, E2.z-z);


  double cont1 = inited_c ? fabs(c1u - last_c_local) : 0.0;
  double cont2 = inited_c ? fabs(c2u - last_c_local) : 0.0;


  // weights: match MATLAB
  const double w_pos  = 1.0;
  const double w_cont = 0.05;
  const double hys    = 0.5;


  double J1 = w_pos*pos_err1 + w_cont*cont1;
  double J2 = w_pos*pos_err2 + w_cont*cont2;


  int pick;
  if(!inited_pick){
    pick = (J1 <= J2) ? 1 : 2;
    inited_pick = true;
    inited_c = true;
  }else{
    if(last_pick == 1){
      pick = (J2 < J1 - hys) ? 2 : 1;
    }else{
      pick = (J1 < J2 - hys) ? 1 : 2;
    }
  }


  double chosen = (pick==1) ? c1u : c2u;


  // commit theta4 memory with chosen
  (void)fkEndEffector(theta_a, theta_b, chosen, true);


  last_c_local = chosen;
  last_pick = pick;


  angle1 = wrapPi(theta_a);
  angle2 = wrapPi(theta_b);
  angle3 = chosen;


  // debug
  debug_a1 = angle1; debug_a2 = angle2; debug_a3 = angle3;
}




// -----------------------------
//  Globals
// -----------------------------
IntervalTimer controlTimer;
volatile bool doUpdate = false;
double tt = 0.0;
Leg RF;
Leg LF;
Leg RB;
Leg LB;

static constexpr size_t USB_LINE_MAX = 256;
char usbLine[USB_LINE_MAX];
size_t usbLineLen = 0;
bool usbLineOverflow = false;

uint32_t heartbeatSeq = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastCommandMs = 0;
unsigned long lastFailsafeStandMs = 0;

const double STAND_DEG[12] = {
   0.0, -69.636, -33.474,
  -0.0,  69.636,  33.474,
   0.0,  69.636,  33.474,
  -0.0, -69.636, -33.474
};




// -----------------------------
//  Motor packet (SYNC WRITE 12 motors)
//  - a1..a12 in RAD (internal)
// -----------------------------
void sendMotorPacketRad(double a1, double a2, double a3,
                        double a4, double a5, double a6,
                        double a7, double a8, double a9,
                        double a10,double a11,double a12)
{
  // safety clamp
  auto clampRad = [](double r){
    return clampd(r, deg2rad(-LIMIT_DEG), deg2rad(LIMIT_DEG));
  };


  const uint8_t header[] = {0xFF, 0xFF, 0xFE, 0x58, 0x83, 0x2A, 0x06};
  uint8_t packet[150];
  int idx = 0;


  memcpy(packet, header, sizeof(header));
  idx += sizeof(header);


  const uint8_t ids[12] = {1,2,3,4,5,6,7,8,9,10,11,12};


  double rad[12] = {
    clampRad(a1), clampRad(a2), clampRad(a3),
    clampRad(a4), clampRad(a5), clampRad(a6),
    clampRad(a7), clampRad(a8), clampRad(a9),
    clampRad(a10),clampRad(a11),clampRad(a12)
  };


  uint16_t goal[12];
  for(int i=0;i<12;i++){
    goal[i] = centerPlusRad(rad[i]);
  }


  for(int i=0;i<12;i++){
    packet[idx++] = ids[i];


    // position 0x2A
    packet[idx++] = (goal[i] >> 0) & 0xFF;
    packet[idx++] = (goal[i] >> 8) & 0xFF;


    packet[idx++] = 0x00;                // 資料長度低位
    packet[idx++] = 0x01;                // 資料長度高位
    packet[idx++] = 0xE8;    // 位置值低位
    packet[idx++] = 0x03; // 位置值高位
  }


  // checksum: sum from 0xFE
  uint8_t sum = 0;
  for (int i = 2; i < idx; i++) {
    sum += packet[i];
  }
  uint8_t checksum = ~sum;


  packet[idx++] = checksum;




  Serial4.write(packet, idx);
}


// Convenience: send in DEG (more human-friendly)
void sendMotorPacketDeg(double d1, double d2, double d3,
                        double d4, double d5, double d6,
                        double d7, double d8, double d9,
                        double d10,double d11,double d12)
{
  sendMotorPacketRad(deg2rad(d1), deg2rad(d2), deg2rad(d3),
                     deg2rad(d4), deg2rad(d5), deg2rad(d6),
                     deg2rad(d7), deg2rad(d8), deg2rad(d9),
                     deg2rad(d10),deg2rad(d11),deg2rad(d12));
}

void sendMotorPacketDegArray(const double deg[12])
{
  sendMotorPacketDeg(
    deg[0],  deg[1],  deg[2],
    deg[3],  deg[4],  deg[5],
    deg[6],  deg[7],  deg[8],
    deg[9],  deg[10], deg[11]
  );
}

void sendStandPose()
{
  sendMotorPacketDegArray(STAND_DEG);
}

void sendAck(uint32_t seq, bool ok)
{
  Serial.print("$ACK,");
  Serial.print(seq);
  Serial.print(",");
  Serial.println(ok ? 1 : 0);
}

void sendHeartbeat()
{
  Serial.print("$HB,");
  Serial.println(++heartbeatSeq);
}

uint32_t parseSeqBestEffort(const char* line)
{
  if(strncmp(line, "$ANGLES,", 8) != 0) return 0;
  char* end = nullptr;
  unsigned long seq = strtoul(line + 8, &end, 10);
  if(end == line + 8) return 0;
  return (uint32_t)seq;
}

bool parseAnglesLine(const char* line, uint32_t& seq, double deg[12])
{
  if(strncmp(line, "$ANGLES,", 8) != 0) return false;

  const char* p = line + 8;
  char* end = nullptr;

  unsigned long parsedSeq = strtoul(p, &end, 10);
  if(end == p || *end != ',') return false;
  seq = (uint32_t)parsedSeq;
  p = end + 1;

  for(int i = 0; i < 12; ++i){
    deg[i] = strtod(p, &end);
    if(end == p) return false;
    if(!isfinite(deg[i])) return false;
    p = end;

    if(i < 11){
      if(*p != ',') return false;
      ++p;
    }else{
      while(*p == ' ' || *p == '\t' || *p == '\r') ++p;
      if(*p != '\0') return false;
    }
  }

  return true;
}

void handleCommandLine(const char* line)
{
  if(strncmp(line, "$ANGLES,", 8) != 0){
    return;
  }

  uint32_t seq = 0;
  double deg[12];
  if(parseAnglesLine(line, seq, deg)){
    sendMotorPacketDegArray(deg);
    lastCommandMs = millis();
    sendAck(seq, true);
  }else{
    sendAck(parseSeqBestEffort(line), false);
  }
}

void handleUsbSerial()
{
  while(Serial.available() > 0){
    char c = (char)Serial.read();

    if(c == '\r'){
      continue;
    }

    if(c == '\n'){
      if(!usbLineOverflow){
        usbLine[usbLineLen] = '\0';
        handleCommandLine(usbLine);
      }
      usbLineLen = 0;
      usbLineOverflow = false;
      continue;
    }

    if(usbLineOverflow){
      continue;
    }

    if(usbLineLen < USB_LINE_MAX - 1){
      usbLine[usbLineLen++] = c;
    }else{
      usbLineLen = 0;
      usbLineOverflow = true;
    }
  }
}


// -----------------------------
//  Trajectory (safe small ellipse)
// -----------------------------
void lift_trajectory(Leg& leg, double t){
  const double x0  = 26.086;
  const double y0 = 0.0;
  const double z0 = -150.0;
  //const double Ay = 20.0;
  //const double Az = 10.0;


  double x = x0;
  double y = y0;


  double s = 0.5 * (1.0 - cos(2.0 * omega * t)); // 0..1..0
  double z = z0 + lift * s;


  leg.setTargetPosition(x, y, z);


  return;
}
void sinusoidal_trajectory(Leg& leg, double p, double duty){
  p = wrap01(p);


  const double x0 = 26.086;
  const double y0 = 0.0;
  const double z0 = -150.0;


  if(p < duty){
    // Stance phase: push backward
    double u = p / duty;
    double y = y0 + disp * (0.5 - u);
    leg.setTargetPosition(x0, y, z0);  // ← 加這行
  }else{
    // Swing phase: lift and move forward
    double u = (p - duty) / (1.0 - duty);
    double y = y0 + disp * (u - 0.5);
    double z = z0 + lift * sin(M_PI * u);
    leg.setTargetPosition(x0, y, z);
  }
}
void standing_pose(Leg& leg){
  const double x  = 26.086;
  const double y = 0.0;
  const double z = -150.0;
  leg.setTargetPosition(x, y, z);
}
void squat_pose(Leg& leg){
  const double x  = 26.086;
  const double y = 0.0;
  const double z = -80.0;
  leg.setTargetPosition(x, y, z);
}


//change the phase of one feet every 1/2T
void trot(Leg& RB, Leg& RF, Leg& LB, Leg& LF, double phase){
  const double duty = 0.6; // or 0.5
  sinusoidal_trajectory(RB, phase, duty);
  sinusoidal_trajectory(LF, phase, duty);
  sinusoidal_trajectory(RF, wrap01(phase + 0.5), duty);
  sinusoidal_trajectory(LB, wrap01(phase + 0.5), duty);
 
  RB.computeIK_FKSelect();
  LB.computeIK_FKSelect();
  RF.computeIK_FKSelect();
  LF.computeIK_FKSelect();
}
//change the phase of one feet every 1/4T
void walk(Leg& RB, Leg& RF, Leg& LB, Leg& LF, double phase){
  const double duty = 0.75;
  sinusoidal_trajectory(RB, phase, duty);
  sinusoidal_trajectory(RF, wrap01(phase + 0.25), duty);
  sinusoidal_trajectory(LB, wrap01(phase + 0.5), duty);
  sinusoidal_trajectory(LF, wrap01(phase + 0.75), duty);
 
  RB.computeIK_FKSelect();
  LB.computeIK_FKSelect();
  RF.computeIK_FKSelect();
  LF.computeIK_FKSelect();
}


void lean(Leg& RB, Leg& RF, Leg& LB, Leg& LF){
  squat_pose(RB);
  squat_pose(RF);


  standing_pose(LB);
  standing_pose(LF);
 
  RB.computeIK_FKSelect();
  RF.computeIK_FKSelect();
  LB.computeIK_FKSelect();
  LF.computeIK_FKSelect();


}


// -----------------------------
//  Timer ISR: only set a flag
// -----------------------------
void updateControlISR(){
  doUpdate = true;
}


// -----------------------------
//  setup / loop
// -----------------------------
void setup(){
  Serial4.begin(BAUD_SERVO);
  Serial.begin(BAUD_USB);
  delay(200);


  // 先送一次固定姿勢 (deg)
  sendStandPose();
  delay(1000);

  lastCommandMs = millis();
  lastFailsafeStandMs = millis();
  lastHeartbeatMs = millis();

  if(!ENABLE_ROS_BRIDGE && ENABLE_TIMER){
    controlTimer.begin(updateControlISR, CTRL_US);
  }
}


void loop(){
  if(ENABLE_ROS_BRIDGE){
    handleUsbSerial();

    unsigned long now = millis();
    if(now - lastHeartbeatMs >= HEARTBEAT_MS){
      lastHeartbeatMs = now;
      sendHeartbeat();
    }

    if(now - lastCommandMs >= COMMAND_TIMEOUT_MS &&
       now - lastFailsafeStandMs >= FAILSAFE_STAND_MS){
      lastFailsafeStandMs = now;
      sendStandPose();
    }

    return;
  }

  //control gait here
  static bool enable_trot = true;
  static bool enable_walk = false;
  static bool enable_lean = false;
  double dt = CTRL_US / 1000000.0;
  if(ENABLE_TIMER && doUpdate){
    doUpdate = false;
   
    tt += dt; //20000/1000000.0 = 0.02
    //if(tt > 2.0*M_PI) tt -= 2.0*M_PI;
    phase = wrap01(phase + phaseRate * dt) ;  // dt = 20ms


    if(ENABLE_TRAJ){
      if(enable_lean){
        lean(RB, RF, LB, LF);
      }
      else if(enable_walk){
        walk(RB, RF, LB, LF, phase);
      }
      else if(enable_trot){
        trot(RB, RF, LB, LF, phase);
      }
      else {
        //STAND
        standing_pose(RB);
        standing_pose(RF);


        standing_pose(LB);
        standing_pose(LF);
       
        LB.computeIK_FKSelect();
        RF.computeIK_FKSelect();
        RB.computeIK_FKSelect();
        LF.computeIK_FKSelect();
      }






    sendMotorPacketDeg(
      rad2deg(LB.angle1), (-1)*rad2deg(LB.angle2), rad2deg(LB.angle3),
      (-1)*rad2deg(RB.angle1), rad2deg(RB.angle2) ,  (-1)*rad2deg(RB.angle3),
      rad2deg(RF.angle1), rad2deg(RF.angle2), (-1)*rad2deg(RF.angle3),
      (-1)*rad2deg(LF.angle1), (-1)*rad2deg(LF.angle2), rad2deg(LF.angle3)
    );
    }
  }


  // debug print every 200ms
  /*unsigned long now = millis();
  if(now - lastPrintTime >= 200){
    lastPrintTime = now;


    Serial.print("x: "); Serial.print((double)debug_x, 3);
    Serial.print(", y: "); Serial.print((double)debug_y, 3);
    Serial.print(", z: "); Serial.println((double)debug_z, 3);


    Serial.print("a1(rad): "); Serial.print((double)debug_a1, 6);
    Serial.print(", a2(rad): "); Serial.print((double)debug_a2, 6);
    Serial.print(", a3(rad): "); Serial.println((double)debug_a3, 6);
  }*/
}
