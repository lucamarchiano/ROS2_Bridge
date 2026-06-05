from setuptools import find_packages, setup

package_name = 'pychrono_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Marchiano',
    maintainer_email='luca.marchiano@studenti.polito.it',
    description='ROS2 Python Client for PyChrono SIL/HIL simulation',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pychrono_client_example = pychrono_client.pychrono_client_example:main',
        ],
    },
)
