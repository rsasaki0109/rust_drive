"""Use the fitted-center union in the unchanged three-round scheduler."""
import importlib.util

import fitted_center_refinement as fixed
import reselected_slope_refinement as previous
from projective_depth_common import ROOT

spec = importlib.util.spec_from_file_location('reselected_center_private_driver',ROOT/'scripts/reselected_slope_refinement.py')
private = importlib.util.module_from_spec(spec);spec.loader.exec_module(private)
private.fixed = fixed

ROUNDS,INNER_CAP = private.ROUNDS,private.INNER_CAP
transition,stop,refine = private.transition,private.stop,private.refine
