from setuptools import setup

package_name = 'sub2'

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
            'a_star = sub2.a_star:main',
            'a_star_local_path = sub2.a_star_local_path:main',
            'ex_calib = sub2.ex_calib:main',
            'human_detector = sub2.human_detector:main',
            'load_map = sub2.load_map:main',
            'odom = sub2.odom:main',
            'path_tracking = sub2.path_tracking:main',
            'seg_binarizer = sub2.seg_binarizer:main'
        ],
    },
)
