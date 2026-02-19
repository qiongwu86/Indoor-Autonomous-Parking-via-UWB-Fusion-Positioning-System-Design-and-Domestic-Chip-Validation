from setuptools import find_packages, setup
from glob import glob

package_name = 'server_python'

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
            'static_tf_broadcaster  =   server_python.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster =   server_python.dynamic_tf_broadcaster:main',
            'base_tf_map =   server_python.base_tf_map:main',
            'PositionPubNode =   server_python.PositionPubNode:main',
            'parking_server =   server_python.parking_server:main',
            'ei_paper_server =   server_python.ei_paper_server:main',
            'frse_server =   server_python.frse_server:main',
            'infocom_server =   server_python.infocom_server:main',

        ],
    },
)
