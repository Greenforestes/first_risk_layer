import glob

from setuptools import find_packages, setup

package_name = 'agent_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/config', ['config/patrol_config.yaml', 'config/trial_blind_corner.yaml',
    'config/trial_corridor_cross.yaml',]),
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
            'patrol_node = agent_patrol.patrol_node:main',
            'fixed_trial_node = agent_patrol.fixed_trial_node:main',
        ],
    },
)
