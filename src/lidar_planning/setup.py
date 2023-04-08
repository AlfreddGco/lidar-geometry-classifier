from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'lidar_planning'

setup(
    name=PACKAGE_NAME,
    version='0.9.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alfredo Garcia',
    author_email='alfredd.gco@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Lidar planning algorithms.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = lidar_planning.lidar_listener:main',
        ],
    },
)

