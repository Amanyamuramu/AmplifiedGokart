from setuptools import setup

package_name = 'pid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amane',
    maintainer_email='corotanman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = pid_control.pid_controller:main',
            'velocity_calculator = pid_control.velocity_calculator:main',
            'pid_controller_vel = pid_control.pid_controller_pose_to_vel:main'
        ],
    },
)
