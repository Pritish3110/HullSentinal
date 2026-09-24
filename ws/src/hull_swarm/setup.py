from setuptools import setup

package_name = 'hull_swarm'

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
    description='Cognition layer: Belief map, utility scorer, task priority queue, and closed-loop verification monitor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'belief_map_node = hull_swarm.belief_map_node:main',
            'utility_scorer = hull_swarm.utility_scorer:main',
            'verification_monitor = hull_swarm.verification_monitor:main',
        ],
    },
)
