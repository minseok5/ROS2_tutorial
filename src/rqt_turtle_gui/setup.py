from setuptools import find_packages, setup

package_name = 'rqt_turtle_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minseok',
    maintainer_email='oms8329@seoultech.ac.kr',
    description='RQT plugin for setting goal position in turtlesim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyqt_mouse = rqt_turtle_gui.pyqt_mouse:main',
        ],
    },
)
