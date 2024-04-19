from setuptools import find_packages, setup
import glob
import os

package_name = 'opencv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param',
            glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryu',
    maintainer_email='yhryu96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_node = opencv_pkg.opencv_node:main',
            'img_pub = opencv_pkg.img_pub:main',
            'rgb_sub = opencv_pkg.rgb_sub:main',
            'capture_serv = opencv_pkg.capture_serv:main',
            'video_server = opencv_pkg.video_server:main',
        ],
    },
)
