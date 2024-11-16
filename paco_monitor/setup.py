from setuptools import find_packages, setup

package_name = 'paco_monitor'

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
    maintainer='blackzafiro',
    maintainer_email='v.arriola@ciencias.unam.mx',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_monitor = paco_monitor.simple_monitor:main'
        ],
    },
)
