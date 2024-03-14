from setuptools import find_packages, setup

package_name = 'simple_frontiers_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lajoiepy',
    maintainer_email='lajoie.py@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = simple_frontiers_exploration.simple_frontiers_explorer:main'
        ],
    },
)
