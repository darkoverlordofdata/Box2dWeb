Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2

b2JointDef = Box2D.Dynamics.Joints.b2JointDef

class Box2D.Dynamics.Joints.b2MouseJointDef extends b2JointDef

  constructor: ->
    super
    @target = new b2Vec2()
    @type = b2Joint.e_mouseJoint
    @maxForce = 0.0
    @frequencyHz = 5.0
    @dampingRatio = 0.7
    return

