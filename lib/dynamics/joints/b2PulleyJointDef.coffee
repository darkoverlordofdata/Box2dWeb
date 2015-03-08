Box2D = require('../../index')

b2Joint           = Box2D.Dynamics.Joints.b2Joint
b2Vec2            = Box2D.Common.Math.b2Vec2
b2JointDef        = Box2D.Dynamics.Joints.b2JointDef


class Box2D.Dynamics.Joints.b2PulleyJointDef extends b2JointDef
  
  
  type                : b2Joint.e_pulleyJoint
  groundAnchorA       : null
  groundAnchorB       : null
  localAnchorA        : null
  localAnchorB        : null
  lengthA             : 0.0
  maxLengthA          : 0.0
  lengthB             : 0.0
  maxLengthB          : 0.0
  ratio               : 1.0
  collideConnected    : true

  constructor: ->
    super
    @groundAnchorA = new b2Vec2()
    @groundAnchorB = new b2Vec2()
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @groundAnchorA.Set (-1.0), 1.0
    @groundAnchorB.Set 1.0, 1.0
    @localAnchorA.Set (-1.0), 0.0
    @localAnchorB.Set 1.0, 0.0
    return

  Initialize: (bA, bB, gaA, gaB, anchorA, anchorB, r) ->
    r = 0  if r is undefined
    @bodyA = bA
    @bodyB = bB
    @groundAnchorA.SetV gaA
    @groundAnchorB.SetV gaB
    @localAnchorA = @bodyA.GetLocalPoint(anchorA)
    @localAnchorB = @bodyB.GetLocalPoint(anchorB)
    d1X = anchorA.x - gaA.x
    d1Y = anchorA.y - gaA.y
    @lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y)
    d2X = anchorB.x - gaB.x
    d2Y = anchorB.y - gaB.y
    @lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y)
    @ratio = r
    C = @lengthA + @ratio * @lengthB
    @maxLengthA = C - @ratio * b2PulleyJoint.b2_minPulleyLength
    @maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / @ratio
    return
