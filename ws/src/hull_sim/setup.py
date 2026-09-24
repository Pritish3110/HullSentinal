from setuptools import setup
import os
from glob import glob

package_name = 'hull_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pritish Bhatasana',
    maintainer_email='pritish@todo.todo',
    description='Simulation engine, defect dynamics, and sensor simulation for HullSentinal',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_engine_node = hull_sim.sim_engine_node:main',
        ],
    },
)
