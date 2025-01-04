from setuptools import find_packages
from setuptools import setup

package_name = 'dual_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your-email@example.com',
    maintainer='Your Maintainer',
    maintainer_email='maintainer@example.com',
    keywords=['ROS', 'robotics'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Your package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_frame_modifier = dual_arm.pointcloud_frame_modifier_node:main',
        ],
    },
)