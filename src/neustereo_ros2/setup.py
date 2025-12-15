from glob import glob
from setuptools import find_packages, setup

package_name = 'neustereo_ros2'

# Path to NeuStereo submodule (relative to this setup.py)
neustereo_submodule = '../../submodules/NeuStereo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + ['NeuStereo'],
    package_dir={
        'NeuStereo': f'{neustereo_submodule}/NeuStereo',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
            'neu_stereo_node = neustereo_ros2.neustereo_ros2:main',
        ],
    },
)
