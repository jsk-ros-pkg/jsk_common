#!/usr/bin/env python

from jsk_tools.bag_plotter import *
import sys
if __name__ == "__main__":
    plotter = BagPlotter()
    plotter.parse()
    plotter.run()
    
