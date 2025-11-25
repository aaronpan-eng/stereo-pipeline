from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stereo_rectification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neuroam',
    maintainer_email='neuroam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rectify = stereo_rectification.rectify:main',
            'camera_info_publisher = stereo_rectification.camera_info_publisher:main',
        ],
    },
)
