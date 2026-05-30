from setuptools import find_packages, setup

package_name = 'propulsion_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='williamwarrenmeeks@gmail.com',
    description='All the propulsion control of the Gorgon.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = propulsion_ros.teleop:main',
            'mission_control = propulsion_ros.mission_control:main',
            'odrive = propulsion_ros.odrive:main'
        ],
    },
)
