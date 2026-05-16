from setuptools import find_packages, setup

package_name = 'quadruped_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asrlab-yian',
    maintainer_email='asrlab-yian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_node       = quadruped_control.keyboard_node:main',
            'gait_generator_node = quadruped_control.gait_generator_node:main',
            'motor_control_node  = quadruped_control.motor_control_node:main',
            'joint_state_bridge_node = quadruped_control.joint_state_bridge_node:main',
            'policy_node = quadruped_control.policy_node:main',
            'safety_node = quadruped_control.safety_node:main',
            'sts_feedback_test_node = quadruped_control.sts_feedback_test_node:main',

        ],
    },
)
