#!/usr/bin/env python

import sys

from rpi_arm_composites_manufacturing_gui.experiment_gui_module import ExperimentGUI
from rqt_gui.main import Main

plugin = 'rpi_arm_composites_manufacturing_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
