Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2


class Box2D.Dynamics.Joints.b2DistanceJointDef

  type              : 0
  localAnchorA      : null
  localAnchorB      : null
  userData          : null
  bodyA             : null
  bodyB             : null
  length            : 0
  frequencyHz       : 0.0
  dampingRatio      : 0.0

  constructor: (bA, bB, anchorA, anchorB) ->
    @type = b2Joint.e_distanceJoint
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @userData = null
    @bodyA = bA  if bA isnt undefined
    @bodyB = bB  if bB isnt undefined
    @localAnchorA.SetV anchorA  if anchorA isnt undefined
    @localAnchorB.SetV anchorB  if anchorB isnt undefined
    if anchorA isnt undefined and anchorB isnt undefined
      dX = anchorB.x - anchorA.x
      dY = anchorB.y - anchorA.y
      @length = Math.sqrt(dX * dX + dY * dY)
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

