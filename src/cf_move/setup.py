from setuptools import find_packages, setup

package_name = 'cf_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minseok',
    maintainer_email='minseok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'python_node = cf_move.python_node:main',
            'cf_move = cf_move.move:main',
            'cf_param_client = cf_move.goal_parameter_client:main',
            'cf_param_node = cf_move.goal_parameter_node:main',
            'cf_trimove = cf_move.tri_move:main'

        ],
    },
)
