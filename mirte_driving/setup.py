from glob import glob

from setuptools import find_packages, setup

package_name = 'mirte_driving_3'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/trees', glob('trees/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group3 spatial-ai',
    maintainer_email='guilherme6henriques@gmail.com',
    description='Navigation/mapping + A-B marker shuttle (Team Member C).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # .py-suffixed entry points to match the team convention.
    entry_points={
        'console_scripts': [
            'shuttle_manager.py = mirte_driving_3.shuttle_manager:main',
            'zone_detector.py = mirte_driving_3.zone_detector:main',
            'scan_filter.py = mirte_driving_3.scan_filter:main',
        ],
    },
)
