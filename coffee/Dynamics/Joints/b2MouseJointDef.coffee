class Box2D.Dynamics.Joints.b2MouseJointDef extends b2JointDef

  type            : b2Joint.e_mouseJoint
  target          : null
  maxForce        : 0.0
  frequencyHz     : 5.0
  dampingRatio    : 0.7

  constructor: ->
    @target = new b2Vec2()
    return

