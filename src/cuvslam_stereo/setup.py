from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'cuvslam_stereo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaron',
    maintainer_email='pan.aa@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cuvslam_stereo_node = cuvslam_stereo.cuvslam_stereo_node:main',
            'cuvslam_stereo_kitti_node = cuvslam_stereo.cuvslam_stereo_kitti_node:main',
        ],
    },
)
