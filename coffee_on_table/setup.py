## ! DO NOT MANUALLY INVOKE THIS, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['camera_image_manipulator','tutorial_package', 'find_mug_on_table', 'ur5_control'],
	package_dir={'': 'src'},
)

setup(**setup_args)
