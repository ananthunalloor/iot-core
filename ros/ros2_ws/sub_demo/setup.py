from setuptools import find_packages, setup

package_name = 'sub_demo'

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
    maintainer='cozmik',
    maintainer_email='root@todo.todo',
    description='Subscriber demo for ros2_mock_publish_topic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'subscriber_demo = sub_demo.subscriber_demo:main',
        ],
    },
)
