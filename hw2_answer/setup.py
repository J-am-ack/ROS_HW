from glob import glob
import os

from setuptools import setup


package_name = 'hw2_answer'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.log_odds_mapping'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), ['REPORT.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Homework 2 occupancy-grid mapping solutions.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simple_mapping = hw2_answer.simple_mapping:main',
            'log_odds_mapping = hw2_answer.log_odds_mapping.log_odds_mapping:main',
        ],
    },
)
