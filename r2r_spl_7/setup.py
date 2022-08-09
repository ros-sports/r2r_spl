from setuptools import setup

package_name = 'r2r_spl_7'

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
    description='Robot-To-Robot communication in RoboCup SPL using SPLSM V7',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2r_spl = r2r_spl_7.r2r_spl:main',
        ],
    },
)
