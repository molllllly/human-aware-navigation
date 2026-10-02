from setuptools import setup

package_name = 'air_navigation_examples'

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
    maintainer='cyrille',
    maintainer_email='cyrille.berger@liu.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualize_human = air_navigation_examples.visualize_human:main',
            'ana = air_navigation_examples.ana:main',
            'anasimple = air_navigation_examples.anasimple:main',
        ],
    },
)
