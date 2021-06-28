#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import pinocchio
import os
import numpy as np

# Get URDF filename
script_dir = os.path.dirname(os.path.realpath("__file__"))
pinocchio_model_dir = os.path.join(script_dir, "../urdf")
urdf_filename = pinocchio_model_dir + "/talos_full_legs_v2.urdf"

# Load URDF model
model = pinocchio.buildModelFromUrdf(urdf_filename, pinocchio.JointModelFreeFlyer())
data = model.createData()
print('model name: ' + model.name)

# Se ponen las posiciones, velocidades, aceleraciones y fuerzas (en el
# marco de referencia de cada articulación) 
q = pinocchio.utils.zero(model.nq)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)
fext = pinocchio.StdVec_Force()
for _ in range(model.njoints):
    fext.append(pinocchio.Force.Zero())

# Valores de prueba
v[12] = 1
a[12] = 10

# Calcula las derivadas de la dinámica
# pinocchio.computeABADerivatives(model, data, q, v, a, fext)

# Muestra las aceleraciones
print('ddq: ' + str(data.ddq))
# Muestra las derivadas de la aceleración articular con respecto a la q
print('ddq_dq: ' + str(data.ddq_dq))
# Muestra las derivadas de la aceleración articular con respecto a la velocidad articular
print('ddq_dv: ' + str(data.ddq_dv))
# Muestra las derivadas del torque con respecto a q
print('dtau_dq: ' + str(data.dtau_dq))
# Muestra las derivadas del torque con respecto a la velocidad articular
print('dtau_dv: ' + str(data.dtau_dv))

# Muestra el jacobiano del centro de masas
# pinocchio.computeForwardKinematicsDerivatives(model, data, q, v, a)
# pinocchio.jacobianCenterOfMass(model, data, q)
# print('center of mass jacobian:\n' + str(data.Jcom.T))
# print('center of mass jacobian dimensions: ' + str(data.Jcom.shape))

# Calcula las derivadas parciales de las velocidades del centro de masas con respecto a q
# print('center of mass vel der:\n' + str(pinocchio.getCenterOfMassVelocityDerivatives(model, data).T))

# v[10] = 4
# v[8] = -3
# pinocchio.computeForwardKinematicsDerivatives(model, data, q, v, a)
# pinocchio.centerOfMass(model, data, q, v)

# Calcula las derivadas parciales de las velocidades del centro de masas con respecto a q
# print('center of mass vel der:\n' + str(pinocchio.getCenterOfMassVelocityDerivatives(model, data).T))

print(q.shape)
q_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
          4.26e-19, 0.0684276, -0.706319, 1.47293, -0.7666, -0.068427, \
          4.33885e-19, -0.0018791, -0.74193, 1.47356, -0.73163, 0.0018791]
q = np.asarray(q_list, dtype=np.float32)
v = np.asarray(q_list, dtype=np.float32)
print(v)
# pinocchio.forwardKinematics(model, data, q)
pinocchio.computeJointJacobians(model, data, q)
pinocchio.updateFramePlacements(model, data)
# pinocchio.computeForwardKinematicsDerivatives(model, data, q, v, a)
frame_id = model.getFrameId('left_sole_link')
J_left = pinocchio.getFrameJacobian(model, data, frame_id,
                                    pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
print('J_left:\n' + str(J_left.T))

base_id = model.getFrameId("base_link")
print('base_link id: ' + str(base_id))

