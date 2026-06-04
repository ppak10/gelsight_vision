from setuptools import find_packages, setup

package_name = 'gelsight_vision'

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
    maintainer='ppak',
    maintainer_email='ppak10@gmail.com',
    description='Manages Gelsight and vision from D555 Realsense camera',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_goal_example = gelsight_vision.pose_goal_example:main',
            'move_camera_focus = gelsight_vision.move_camera_focus:main',
            'touch_with_gelsight = gelsight_vision.touch_with_gelsight:main'
        ],
    },
)
