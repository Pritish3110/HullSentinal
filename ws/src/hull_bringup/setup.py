from setuptools import setup
import os
from glob import glob

package_name = 'hull_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pritish Bhatasana',
    maintainer_email='pritish@todo.todo',
    description='Launch orchestration, RViz2 configurations, and visualization marker publishers for HullSentinal',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'marker_publisher = hull_bringup.marker_publisher:main',
            'demo_snapshot = hull_bringup.demo_snapshot:main',
        ],
    },
)
