from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coverage_evaluator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('lib', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='user@todo.todo',
    description='Coverage ratio evaluator for a user-clicked polygon area using robot pose (no map required).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_evaluator_node = coverage_evaluator.coverage_evaluator_node:main',
            'run_camera_coverage = coverage_evaluator.scripts.run_camera_coverage:main',
        ],
    },
)
