from setuptools import setup
import os
from glob import glob

package_name = 'apf_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Se tiver launch files ou outros arquivos de dados
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Júlia',
    maintainer_email='juliacarneirobritto@gmail.com',
    description='APF Controller ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_node = apf_controller.apf_node:main',
        ],
    },
)
