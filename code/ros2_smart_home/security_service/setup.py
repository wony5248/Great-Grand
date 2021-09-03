from setuptools import setup

package_name = 'security_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='seungje@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intruder_detector_client = security_service.intruder_detector_client:main',
            'patrol_client = security_service.patrol_client:main',
            'odom = security_service.odom:main',
            'path_pub = security_service.path_pub:main',
            'make_path = security_service.make_path:main',
            'path_tracking = security_service.path_tracking:main'
        ],
    },
)
