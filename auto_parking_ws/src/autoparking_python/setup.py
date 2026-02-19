from setuptools import find_packages, setup
from glob import glob

package_name = 'autoparking_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch',glob('launch/*.launch.py')),
        ('share/' + package_name+'/parking_rviz',glob('parking_rviz/*.rviz')),
        ('share/' + package_name+'/map',glob('map/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='www',
    maintainer_email='www@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster  =   autoparking_python.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster =   autoparking_python.dynamic_tf_broadcaster:main',
            'base_tf_map =   autoparking_python.base_tf_map:main',
            'PositionPubNode =   autoparking_python.PositionPubNode:main',
            'car1_node =   autoparking_python.car1_node:main',
            'car1_start =   autoparking_python.car1_start:main',
            'parking_server =   autoparking_python.parking_server:main',
            'car2_node =   autoparking_python.car2_node:main',
            'car2_start =   autoparking_python.car2_start:main',
            'car1_test =   autoparking_python.car1_test:main',
            'uwb = autoparking_python.uwb:main',
            'car3_node =   autoparking_python.car3_node:main',
            'car3_start =   autoparking_python.car3_start:main',
            'odom =   autoparking_python.odom:main',
        ],
    },
)
