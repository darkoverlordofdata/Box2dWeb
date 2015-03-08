Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint
b2Vec2 = Box2D.Common.Math.b2Vec2

b2JointDef = Box2D.Dynamics.Joints.b2JointDef

class Box2D.Dynamics.Joints.b2RevoluteJointDef extends b2JointDef

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

  constructor: ->
    super
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @type = b2Joint.e_revoluteJoint
    @localAnchorA.Set 0.0, 0.0
    @localAnchorB.Set 0.0, 0.0
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
