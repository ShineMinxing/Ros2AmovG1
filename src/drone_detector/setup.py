from setuptools import setup
import os

package_name = 'drone_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 把 resource 下的 .pth 安装到 share/drone_detector/resource
        (os.path.join('share', package_name, 'resource'), 
         [os.path.join('resource', 'drone_model_best_0.pth')]),

        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxx',
    maintainer_email='xxx@example.com',
    description='...',
    license='...',
    entry_points={
        'console_scripts': [
            'drone_detector_node = drone_detector.drone_detector_node:main',
        ],
    },
)
