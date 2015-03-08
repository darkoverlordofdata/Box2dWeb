Box2D = require('../../index')

b2Joint           = Box2D.Dynamics.Joints.b2Joint
b2Vec2            = Box2D.Common.Math.b2Vec2
b2JointDef        = Box2D.Dynamics.Joints.b2JointDef


class Box2D.Dynamics.Joints.b2WeldJointDef extends b2JointDef

  type                : b2Joint.e_weldJoint
  referenceAngle      : 0.0
  localAnchorA        : null
  localAnchorB        : null



  constructor: ->
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  Initialize: (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return
