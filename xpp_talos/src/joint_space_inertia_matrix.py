from __future__ import print_function

import pinocchio
import os
import numpy

# Get URDF filename
script_dir = os.path.dirname(os.path.realpath(__file__))
pinocchio_model_dir = os.path.join(script_dir, "../urdf")
urdf_filename = pinocchio_model_dir + "/talos_full_legs_v2.urdf"

# Load URDF model
model = pinocchio.buildModelFromUrdf(urdf_filename, pinocchio.JointModelFreeFlyer())
data = model.createData()
print('model name: ' + model.name)

# Normal configuration (every joint is in origin)
q = pinocchio.neutral(model)
q[7:] = numpy.ndarray((12,), buffer=numpy.array([0.0, 0.028730477193730477, -0.7060605463076763, 1.4774145045100333, \
                                                 -0.771353958202357, -0.028730477193733783, 0.0, -0.028730477193724013, -0.7060605463076761, \
                                                 1.4774145045100333, -0.7713539582023572, 0.028730477193724013]))
v = pinocchio.utils.zero(model.nv)

# Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite 
# Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.
pinocchio.ccrba(model, data, q, v)

# Shows the Inertia Matrix
print('Inertia')
print(data.Ig)

# Computes and shows the center of mass
print('Centre of mass:')
print(pinocchio.centerOfMass(model, data, q))
