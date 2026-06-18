import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'soft_wrist_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'),
    	    glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sahithi',
    maintainer_email='sahithi@todo.todo',
    description='Soft wrist simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
