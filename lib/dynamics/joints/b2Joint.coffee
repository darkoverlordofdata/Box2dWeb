Box2D = require('../../index')

class Box2D.Dynamics.Joints.b2Joint

  @e_distanceJoint = 0
  @e_revoluteJoint = 1


  userData      : null
  bodyA         : null
  bodyB         : null
  length        : 0
  next          : null

  constructor: (def) ->
    @m_edgeA = new b2JointEdge()
    @m_edgeB = new b2JointEdge()
    @m_localCenterA = new b2Vec2()
    @m_localCenterB = new b2Vec2()
    b2Settings.b2Assert def.bodyA isnt def.bodyB
    @m_type = def.type
    @m_prev = null
    @m_next = null
    @m_bodyA = def.bodyA
    @m_bodyB = def.bodyB
    @m_collideConnected = def.collideConnected
    @m_islandFlag = false
    @m_userData = def.userData
    return

  GetType: ->
    @m_type

  GetAnchorA: ->
    null

  GetAnchorB: ->
    null

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    null

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  GetBodyA: ->
    @m_bodyA

  GetBodyB: ->
    @m_bodyB

  GetNext: ->
    @m_next

  GetUserData: ->
    @m_userData

  SetUserData: (data) ->
    @m_userData = data
    return

  IsActive: ->
    @m_bodyA.IsActive() and @m_bodyB.IsActive()

  @Create: (def, allocator) ->
    joint = null
    switch def.type
      when b2Joint.e_distanceJoint
        joint = new b2DistanceJoint(((if def instanceof b2DistanceJointDef then def else null)))
      when b2Joint.e_mouseJoint
        joint = new b2MouseJoint(((if def instanceof b2MouseJointDef then def else null)))
      when b2Joint.e_prismaticJoint
        joint = new b2PrismaticJoint(((if def instanceof b2PrismaticJointDef then def else null)))
      when b2Joint.e_revoluteJoint
        joint = new b2RevoluteJoint(((if def instanceof b2RevoluteJointDef then def else null)))
      when b2Joint.e_pulleyJoint
        joint = new b2PulleyJoint(((if def instanceof b2PulleyJointDef then def else null)))
      when b2Joint.e_gearJoint
        joint = new b2GearJoint(((if def instanceof b2GearJointDef then def else null)))
      when b2Joint.e_lineJoint
        joint = new b2LineJoint(((if def instanceof b2LineJointDef then def else null)))
      when b2Joint.e_weldJoint
        joint = new b2WeldJoint(((if def instanceof b2WeldJointDef then def else null)))
      when b2Joint.e_frictionJoint
        joint = new b2FrictionJoint(((if def instanceof b2FrictionJointDef then def else null)))
      else
    joint

  @Destroy: (joint, allocator) ->

  InitVelocityConstraints: (step) ->

  SolveVelocityConstraints: (step) ->

  FinalizeVelocityConstraints: ->

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    false

  @e_unknownJoint = 0
  @e_revoluteJoint = 1
  @e_prismaticJoint = 2
  @e_distanceJoint = 3
  @e_pulleyJoint = 4
  @e_mouseJoint = 5
  @e_gearJoint = 6
  @e_lineJoint = 7
  @e_weldJoint = 8
  @e_frictionJoint = 9
  @e_inactiveLimit = 0
  @e_atLowerLimit = 1
  @e_atUpperLimit = 2
  @e_equalLimits = 3
