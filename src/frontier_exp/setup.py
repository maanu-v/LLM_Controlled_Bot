from setuptools import setup
from glob import glob

package_name = 'frontier_exp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "explorer_node=frontier_exp.explorer_node:main"
        ],
    },
)
