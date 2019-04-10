from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_points_map_loader'],
    package_dir={'': 'src'}
)

setup(**d)

