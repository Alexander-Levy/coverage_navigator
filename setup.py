from setuptools import find_packages, setup

package_name = 'coverage_navigator'

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
    maintainer='Alexander Levy',
    maintainer_email='ingalexanderlevy@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'test_coverage = coverage_navigator.test_coverage:main',
                'demo_coverage = coverage_navigator.demo_coverage:main',
                'field_coverage = coverage_navigator.field:main',
        ],
    },
)
