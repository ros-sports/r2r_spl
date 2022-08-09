from setuptools import setup

package_name = 'splsm_7_conversion'

setup(
    name=package_name,
    version='3.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijnek',
    maintainer_email='kenjibrameld@gmail.com',
    description=('Converts Standard Platform League Standard Message V7 between ROS msg'
                 'and UDP raw bytes'),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
