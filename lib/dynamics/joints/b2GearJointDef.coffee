Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2

b2JointDef = Box2D.Dynamics.Joints.b2JointDef

class Box2D.Dynamics.Joints.b2GearJointDef extends b2JointDef

  type              : b2Joint.e_gearJoint
  joint1            : null
  joint2            : null
  ratio             : 1.0

