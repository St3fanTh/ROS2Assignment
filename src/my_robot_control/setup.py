from setuptools import find_packages, setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=['my_robot_control'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='login',
    maintainer_email='login@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'home_commander = my_robot_control.home_commander:main',
            'cartesian_commander = my_robot_control.cartesian_commander:main',
            'trajectory_visualizer = my_robot_control.trajectory_visualizer:main',
        ],
    },
)
