from setuptools import setup

package_name = 'integrated_navigation_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zww',
    maintainer_email='zhuwang2515@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ins_d_data_parse = integrated_navigation_system.ins_d_data_parse:main',
            'ins_d_data_collect = integrated_navigation_system.ins_data_collection:main',
            'map_preprocess = integrated_navigation_system.map_preprocess:main'
        ],
    },
)
