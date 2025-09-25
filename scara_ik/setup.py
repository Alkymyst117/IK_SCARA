from setuptools import setup

package_name = 'scara_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yerson Palacio',
    maintainer_email='yersonpalacio128@gamil.com.com',
    description='SCARA IK demo (console publisher + ik node)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scara_console_publisher = scara_ik.console_publisher:main',
            'scara_ik_node = scara_ik.ik_node:main',
        ],
    },
)

