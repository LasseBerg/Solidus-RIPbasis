from setuptools import setup

package_name = 'basis'

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
    maintainer='robot',
    maintainer_email='berg.lars99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor = basis.MotorPrintBridge:main',
            'camera_gimbal = basis.CameraGimbalPrintBridge:main',
            'temp = basis.TempPrintBridge:main',
            'sensor = basis.SensorPrintBridge:main',
            'fuses = basis.FusePrintBridge:main',
        ],
    },
)
