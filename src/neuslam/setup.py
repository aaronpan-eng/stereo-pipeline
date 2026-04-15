import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'neuslam'


# TODO: Figure out if this is the best thing to do here
# Maybe intead of copying weight files to install to have weight files outside of the package
def _collect_model_files():
    """Walk the models/ tree and return (install_dir, [files]) pairs for data_files."""
    entries = []
    for root, _dirs, files in os.walk('models'):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        entries.append((install_dir, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'slam_config'), glob('slam_config/*yaml')),
    ] + _collect_model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frl0',
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
            'neuslam = neuslam.neuslam:main',
        ],
    },
)
