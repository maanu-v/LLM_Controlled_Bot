from setuptools import setup
from glob import glob

package_name = 'bot_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test':['pytest'],
    },
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='mark@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
  #  tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard=bot_movement.keyboard:main",
            "motor_control=bot_movement.motor_control:main",
            "odom_motor_pub=bot_movement.odom_motor_pub:main",
            "odom_pub=bot_movement.odom_pub:main"
        ],
    },
)
