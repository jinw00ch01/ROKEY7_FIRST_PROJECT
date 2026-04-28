from setuptools import find_packages, setup

package_name = 'cobot1'

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
    maintainer='choijinwoo',
    maintainer_email='choijinwoo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic = cobot1.move_basic:main',
            'move_periodic = cobot1.move_periodic:main',
            'force_test = cobot1.force_test:main',
            'surveon = cobot1.surveon:main',
            'mini_jog = dsr_rokey2.mini_jog:main',

            'dough_grip_test = cobot1.dough_grip_test:main',
            'press_test = cobot1.press_test:main',
            'plate_setting_test = cobot1.plate_setting_test:main',
            'spatula_test = cobot1.spatula_test:main',
            'dough_to_plate = cobot1.dough_to_plate:main',
            'source_test = cobot1.source_test:main',
            'powder_test = cobot1.powder_test:main',
            'main = cobot1.main:main',
            'dougn_to_plate = cobot1.dougn_to_plate:main',
            'task_manager = cobot1.managers.task_manager_node:main',
            'run_task = cobot1.managers.run_single_task:main',
        ],
    },
)
