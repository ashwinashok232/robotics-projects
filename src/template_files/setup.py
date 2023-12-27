from setuptools import find_packages, setup

package_name = 'template_files' #MODIFY ME (eg: my_robot_controller)

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
    maintainer='ashwin',
    maintainer_email='ashwinashok232@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
#MODIFY entry_points <executable_name> = <package_name>.<python_file_name_without_.py>:main
#EG: 
#   entry_points={
#       'console_scripts': [
#           "draw_circle = my_robot_controller.draw_circle:main"
#       ]