from setuptools import find_packages, setup

package_name = 'shadow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/shadow/launch', [
        'launch/shadow.launch.py',
        'launch/shadow_global.launch.py',
        'launch/mavros.launch.py',
        'launch/simulator.launch.py',

        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='janithcyapa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'shadow_leader = shadow.shadow_leader:main',
              'shadow_follower = shadow.shadow_follower:main',
              'shadow_leader_global = shadow.shadow_leader_global:main',
              'shadow_follower_global = shadow.shadow_follower_global:main',
        ],
    },
)
