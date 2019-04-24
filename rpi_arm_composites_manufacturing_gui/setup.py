from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rpi_arm_composites_manufacturing_gui'],
    package_dir={'': 'src'},
    #install_requires=['pyqtgraph']
)

setup(**d)
