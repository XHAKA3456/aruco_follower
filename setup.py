from setuptools import setup

package_name = 'aruco_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Aruco marker following robot using ROS2 and OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_follower = aruco_follower.aruco_follower:main',
            'aruco_detector = aruco_follower.aruco_detector:main',
            'image_publisher = aruco_follower.image_publisher:main',
        ],
    },
)
