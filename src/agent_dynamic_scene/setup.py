from setuptools import find_packages, setup

package_name = 'agent_dynamic_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dynamic_scene.launch.py']),
        ('share/' + package_name+ '/config', ['config/scenario_blind_corner.yaml',
            'config/scenario_corridor_cross.yaml',]),
        ('share/' + package_name + '/models/dynamic_cylinder', [
            'models/dynamic_cylinder/model.config',
            'models/dynamic_cylinder/model.sdf',
        ]),
      
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dewey',
    maintainer_email='dewey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_actor_manager = agent_dynamic_scene.dynamic_actor_manager:main',
        ],
    },
)
