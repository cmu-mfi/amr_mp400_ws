from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'front_end'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaper',
    maintainer_email='kacpergasior19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'front_end = front_end.waypoint_gui:main',
            'GUI_class = front_end.GUI_class',
            'Camera_class = front_end.Camera_class',
            'Button_class = front_end.Button_class'
        ],
    },
)
