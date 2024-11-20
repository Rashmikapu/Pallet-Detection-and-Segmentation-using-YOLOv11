from setuptools import find_packages, setup
from glob import glob

package_name = 'pallet_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # Add data files - yolo, launch folders
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name, glob('yolo/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='rashmik@umd.edu',
    description='Pallet Detection for Peer Robotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = pallet_detection.img_publisher:main',
            'bag = pallet_detection.bag:main',
            'camera_subscriber = pallet_detection.img_subscriber:main'
        ],
    },
)
