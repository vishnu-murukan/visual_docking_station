from setuptools import find_packages
from setuptools import setup

setup(
    name='visual_docking',
    version='2.0.0',
    packages=find_packages(
        include=('visual_docking', 'visual_docking.*')),
)
