Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2
b2JointDef = Box2D.Dynamics.Joints.b2JointDef


class Box2D.Dynamics.Joints.b2DistanceJointDef extends b2JointDef

  type              : 0
  localAnchorA      : null
  localAnchorB      : null
  userData          : null
  bodyA             : null
  bodyB             : null
  length            : 0
  frequencyHz       : 0.0
  dampingRatio      : 0.0

  constructor: ->
    super
    @localAnchorA = new b2Vec2()
    @type = b2Joint.e_distanceJoint
    @length = 1.0
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

  Initialize: (bA, bB, anchorA, anchorB) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchorA)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchorB)
    dX = anchorB.x - anchorA.x
    dY = anchorB.y - anchorA.y
    @length = Math.sqrt(dX * dX + dY * dY)
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

