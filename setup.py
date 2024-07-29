from setuptools import setup
import os
from glob import glob

package_name = 'ros2_pcd_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add the following line
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # If you have config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuga Yano',
    maintainer_email='yano.yuuga158@mail.kyutech.jp',
    description='Description of your package',
    license='License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
        ],
    },
)