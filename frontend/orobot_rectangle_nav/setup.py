from setuptools import setup

package_name = 'orobot_rectangle_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This should match the Python package directory
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/orobot_rectangle_nav']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ziye Li',
    maintainer_email='liziye725@gmail.com',
    description='ROS 2 package for rectangle navigation and FastAPI integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orobot_nav_node = orobot_rectangle_nav.orobot_nav_node:main',
        ],
    },
)
