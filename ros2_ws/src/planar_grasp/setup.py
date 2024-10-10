from setuptools import find_packages, setup

package_name = 'planar_grasp'

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
    maintainer='azzam',
    maintainer_email='azzam.shaikh8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "planar_grasp_alphashape = planar_grasp.planar_grasp_alphashape:main",
            "planar_grasp_pcl = planar_grasp.planar_grasp_pcl:main"
        ],
    },
)
