from setuptools import find_packages, setup

package_name = 'turtle_sim'

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
    maintainer='kleo',
    maintainer_email='kleo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "turtle_controller=turtle_sim.turtle_controller:main",
            "turtle_spawner=turtle_sim.client_node_spawner:main"
        ],
    },
)
