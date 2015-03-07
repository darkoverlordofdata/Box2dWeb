Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Joints.b2RevoluteJointDef

  type                : 0
  localAnchorA        : null
  localAnchorB        : null
  userData            : null
  bodyA               : null
  bodyB               : null
  referenceAngle      : 0.0
  lowerAngle          : 0.0
  upperAngle          : 0.0
  maxMotorTorque      : 0.0
  motorSpeed          : 0.0
  enableLimit         : false
  enableMotor         : false

  constructor: (bA, bB, anchorA, anchorB) ->
    @type = b2Joint.e_revoluteJoint
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @userData = null
    @bodyA = bA  if bA isnt undefined
    @bodyB = bB  if bB isnt undefined
    @localAnchorA.SetV anchorA  if anchorA isnt undefined
    @localAnchorB.SetV anchorB  if anchorB isnt undefined
    @referenceAngle = 0.0
    @lowerAngle = 0.0
    @upperAngle = 0.0
    @maxMotorTorque = 0.0
    @motorSpeed = 0.0
    @enableLimit = false
    @enableMotor = false
    return

  Initialize: (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return
