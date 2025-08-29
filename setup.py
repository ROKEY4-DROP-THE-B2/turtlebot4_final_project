from setuptools import find_packages, setup

package_name = 'rokey_pjt'

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
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo=rokey_pjt.4_tb4_yolov8_obj_det_thread:main',
            'depth_checker=rokey_pjt.depth_checker_mouse_mean:main',
            'tf=rokey_pjt.tf_point_transform:main',
            'packbot=rokey_pjt.packbot:main',
        ],
    },
)
