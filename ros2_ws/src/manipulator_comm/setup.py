from setuptools import find_packages, setup

package_name = 'manipulator_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/manipulator_comm/launch', ['launch/manipulator_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hulajdusza',
    maintainer_email='163999857+HulajDuszaa@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'i2c_comm_node = manipulator_comm.i2c_comm_node:main',
        ],
    },
)
