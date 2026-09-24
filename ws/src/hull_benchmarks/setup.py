from setuptools import setup

package_name = 'hull_benchmarks'

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
    description='Automated multi-scenario benchmarking, metric aggregation, and baseline comparison suite',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'benchmark_runner = hull_benchmarks.benchmark_runner:main',
            'naive_baseline_node = hull_benchmarks.naive_baseline_node:main',
        ],
    },
)
