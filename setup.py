from setuptools import setup
from glob import glob

package_name = 'fim_track_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py')),
        ('share/' + package_name+'/param', glob('param/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.model')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/'+package_name+'/models/mobile_sensor/meshes/', glob('models/mobile_sensor/meshes/*')),
              ('share/'+package_name+'/models/mobile_sensor/', glob('models/mobile_sensor/*.*')),
   ('share/'+package_name+'/models/source_turtlebot/meshes/', glob('models/source_turtlebot/meshes/*')),
              ('share/'+package_name+'/models/source_turtlebot/', glob('models/source_turtlebot/*.*')),
              
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianpeng',
    maintainer_email='tzhang@g.harvard.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = fim_track_2.talker:main',
        'listener = fim_track_2.listener:main',
        'manual_teleop_key = fim_track_2.manual_teleop_key:main',
        'spawn_entity = fim_track_2.spawn_entity:main',
        'distributed_seeking = fim_track_2.distributed_seeking:main',
        'virtual_sensor = fim_track_2.virtual_sensor:main',
        'virtual_coef = fim_track_2.virtual_coef:main',
        'calibrate = fim_track_2.calibrate_meas_coef:main',
        'arm = fim_track_2.MISSION_CONTROL.ARM:main',
        'move = fim_track_2.MISSION_CONTROL.MOVE:main',
        'log = fim_track_2.remote_data_logger:main',
        'estimate = fim_track_2.distributed_estimation:main'
        ],
    },
)
