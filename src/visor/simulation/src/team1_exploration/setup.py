from setuptools import find_packages, setup

package_name = 'team1_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/explore.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student29',
    maintainer_email='jayantbakolia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_explorer = team1_exploration.frontier_explorer:main',
            "frontier_low_cost_explorer = team1_exploration.frontier_low_cost_explorer:main",
            'frontier_explorer_random = team1_exploration.frontier_explorer_random:main',
        ],
    },
)
