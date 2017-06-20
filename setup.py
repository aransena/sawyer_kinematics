from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sawyer_kinematics'],
    package_dir={'':'src'},
    requires=['std_msgs', 'rospy', 'geometry_msgs']
)

setup(**setup_args)