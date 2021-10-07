from setuptools import setup

package_name = 'ssafy_bridge'

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
    maintainer_email='mgko@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_to_udp = ssafy_bridge.sub_to_udp:main',
            'udp_to_pub = ssafy_bridge.udp_to_pub:main',
            'udp_to_cam = ssafy_bridge.udp_to_cam:main',
            'cam_viewer = ssafy_bridge.cam_viewer:main',
            'udp_to_laser = ssafy_bridge.udp_to_laser:main'
            
            

        ],
    },
)
