from setuptools import find_packages, setup

setup(
    name='mep_monitor', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mep_monitor']),
        ('share/mep_monitor', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='MEP project maintainer', maintainer_email='maintainer@example.invalid',
    description='Mock-only satellite mission telemetry', license='Proprietary',
    entry_points={'console_scripts': [
        'mock_telemetry = mep_monitor.mock_telemetry:main',
        'monitoring_node = mep_monitor.monitoring_node:main',
        'csv_logger = mep_monitor.csv_logger:main',
        'isaac_metrics_bridge = mep_monitor.isaac_metrics_bridge:main',
        'docking_metrics_bridge = mep_monitor.docking_metrics_bridge:main',
    ]},
)
