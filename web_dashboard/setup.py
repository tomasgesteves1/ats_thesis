from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomas',
    maintainer_email='tomasesteves03@gmail.com',
    description='Web control panel and telemetry dashboard for Tethered UAV-USV Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = web_dashboard.dashboard_node:main'
        ],
    },
)
