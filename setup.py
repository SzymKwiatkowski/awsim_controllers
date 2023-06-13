import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'awsim_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='szymon.z.kwiatkowski@gmail.com',
    description='ilqr: maciej.krupka@gmail.com',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_f1_tenth_controller = awsim_controllers.pure_pursuit_f1_tenth_controller:main',
            'validate_topic_sent.py = awsim_controllers.validate_topic_sent:main',
            'stanley_controller = awsim_controllers.stanley_controller:main',
            'ilqr_controller = awsim_controllers.ilqr_f1_tenth_controller:main',
        ],
    },
)
