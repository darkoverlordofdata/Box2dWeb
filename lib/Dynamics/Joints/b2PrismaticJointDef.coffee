class Box2D.Dynamics.Joints.b2PrismaticJointDef extends b2JointDef


  type                : b2Joint.e_prismaticJoint
  localAnchorA        : null
  localAnchorB        : null
  localAxisA          : null
  referenceAngle      : 0.0
  enableLimit         : false
  lowerTranslation    : 0.0
  upperTranslation    : 0.0
  enableMotor         : false
  maxMotorForce       : 0.0
  motorSpeed          : 0.0

  constructor: ->
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAxisA = new b2Vec2()
    @type = b2Joint.e_prismaticJoint
    @localAxisA.Set 1.0, 0.0
    return

  Initialize: (bA, bB, anchor, axis) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @localAxisA = @bodyA.GetLocalVector(axis)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return