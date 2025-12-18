from setuptools import setup

package_name = 'murphy_vision_rs'

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
    maintainer='soap',
    maintainer_email='soap@todo.todo',
    description='RealSense depth to obstacle mask',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_mask = murphy_vision_rs.depth_to_mask:main',
            'pc_to_mask = murphy_vision_rs.pc_to_mask:main',
        ],
    },
)

