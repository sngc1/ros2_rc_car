from setuptools import setup

package_name = 'ros2_rc_car'

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
    maintainer='satoru-n',
    maintainer_email='strngch@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = ros2_rc_car.ackmn_to_rc_MG996R_PCA9685:main',
            'publisher = ros2_rc_car.ackmn_publisher:main',
        ],
    },
)
