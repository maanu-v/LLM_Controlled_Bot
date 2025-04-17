
from setuptools import setup
import os
from glob import glob

package_name = 'web'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test':['pytest'],
    },
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Web interface for Mark robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'web_bridge = web.web_bridge:main',
            'web_server = web.web_server:main',
            'robot_controller = web.robot_controller:main',
        ],
    },
)