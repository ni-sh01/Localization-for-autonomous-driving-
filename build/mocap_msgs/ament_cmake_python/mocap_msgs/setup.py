from setuptools import find_packages
from setuptools import setup

setup(
    name='mocap_msgs',
    version='0.0.3',
    packages=find_packages(
        include=('mocap_msgs', 'mocap_msgs.*')),
)
