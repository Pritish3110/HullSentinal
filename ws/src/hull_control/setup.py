from setuptools import setup

package_name = 'hull_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pritish Bhatasana',
    maintainer_email='pritish@todo.todo',
    description='Kinematic controllers, trajectory tracking, tool state machines, and safety barriers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'crawler_controller = hull_control.crawler_controller:main',
            'mapper_controller = hull_control.mapper_controller:main',
            'tool_controller = hull_control.tool_controller:main',
            'safety_barrier = hull_control.safety_barrier:main',
            'geofence_node = hull_control.geofence_node:main',
        ],
    },
)
