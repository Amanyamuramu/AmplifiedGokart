from setuptools import setup

package_name = 'gokart'

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
            'serial_publisher_node = gokart.serial_publisher_node:main',
            'encoder_publisher = gokart.encoder_publisher:main',
            'odom_qos_converter = gokart.odom_qos_converter:main',
        ],
    },
)
