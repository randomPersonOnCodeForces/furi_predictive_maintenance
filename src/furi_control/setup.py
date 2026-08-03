from setuptools import find_packages, setup

package_name = 'furi_control'

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
    maintainer='richa',
    maintainer_email='richa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mock_robot = furi_control.mock_robot:main",
            "fault_injector = furi_control.fault_injector:main",
            "trajectory_generator = furi_control.trajectory_generator:main",
        ],
    },
)
