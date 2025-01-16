import os
from glob import glob
from setuptools import setup

package_name = 'zenoh_security_configuration'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'templates'),
         glob('templates/*.json5')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejandro@openrobotics.org',
    description='This package generates zenoh secutiry configurations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zenoh_security_configuration = \
              zenoh_security_configuration.zenoh_security_configuration:main'
        ],
    },
)
