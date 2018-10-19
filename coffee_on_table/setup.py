## ! DO NOT MANUALLY INVOKE THIS, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['test_pub', 'camera_image_manipulator','tutorial_package', 'find_mug_on_table', 'ur5_control', 'tf_transform', 'image_rotate', 'robot_control', 'camera_py2ros_wrapper'],
	package_dir={'': 'src'},
)

setup(**setup_args)
