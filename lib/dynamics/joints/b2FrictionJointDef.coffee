Box2D = require('../../index')

b2Joint       = Box2D.Dynamics.Joints.b2Joint
b2Vec2        = Box2D.Common.Math.b2Vec2
b2JointDef    = Box2D.Dynamics.Joints.b2JointDef

class Box2D.Dynamics.Joints.b2FrictionJointDef extends b2JointDef

  type              : b2Joint.e_frictionJoint
  localAnchorA      : null
  localAnchorB      : null
  maxForce          : 0.0
  maxTorque         : 0.0

constructor: ->
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  Initialize: (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    return


