from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waver_cv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gst_mediapipe_face = waver_cv_bringup.gst_mediapipe_face:main',
            'gst_mediapipe_hands = waver_cv_bringup.gst_mediapipe_hands:main',
            'gst_optical_flow = waver_cv_bringup.gst_optical_flow:main',
            'gst_sparse_optical_flow = waver_cv_bringup.gst_sparse_optical_flow:main',
        ],
    },
)
