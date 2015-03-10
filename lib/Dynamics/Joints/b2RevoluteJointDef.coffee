class Box2D.Dynamics.Joints.b2RevoluteJointDef extends b2JointDef

  type                : b2Joint.e_revoluteJoint
  localAnchorA        : null
  localAnchorB        : null
  referenceAngle      : 0.0
  lowerAngle          : 0.0
  upperAngle          : 0.0
  maxMotorTorque      : 0.0
  motorSpeed          : 0.0
  enableLimit         : false
  enableMotor         : false

  constructor: ->
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAnchorA.Set 0.0, 0.0
    @localAnchorB.Set 0.0, 0.0
    return

  Initialize: (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return
