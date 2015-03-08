Box2D = require('../../index')

b2Vec2              = Box2D.Common.Math.b2Vec2
b2JointEdge         = Box2D.Dynamics.Joints.b2JointEdge
b2DistanceJoint     = Box2D.Dynamics.Joints.b2DistanceJoint
b2MouseJoint        = Box2D.Dynamics.Joints.b2MouseJoint
b2PrismaticJoint    = Box2D.Dynamics.Joints.b2PrismaticJoint
b2RevoluteJoint     = Box2D.Dynamics.Joints.b2RevoluteJoint
b2PulleyJoint       = Box2D.Dynamics.Joints.b2PulleyJoint
b2GearJoint         = Box2D.Dynamics.Joints.b2GearJoint
b2LineJoint         = Box2D.Dynamics.Joints.b2LineJoint
b2WeldJoint         = Box2D.Dynamics.Joints.b2WeldJoint
b2FrictionJoint     = Box2D.Dynamics.Joints.b2FrictionJoint
b2DistanceJointDef  = Box2D.Dynamics.Joints.b2DistanceJointDef
b2MouseJointDef     = Box2D.Dynamics.Joints.b2MouseJointDef
b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef
b2RevoluteJointDef  = Box2D.Dynamics.Joints.b2RevoluteJointDef
b2PulleyJointDef    = Box2D.Dynamics.Joints.b2PulleyJointDef
b2GearJointDef      = Box2D.Dynamics.Joints.b2GearJointDef
b2LineJointDef      = Box2D.Dynamics.Joints.b2LineJointDef
b2WeldJointDef      = Box2D.Dynamics.Joints.b2WeldJointDef
b2FrictionJointDef  = Box2D.Dynamics.Joints.b2FrictionJointDef

class Box2D.Dynamics.Joints.b2Joint

  @e_unknownJoint     = 0
  @e_revoluteJoint    = 1
  @e_prismaticJoint   = 2
  @e_distanceJoint    = 3
  @e_pulleyJoint      = 4
  @e_mouseJoint       = 5
  @e_gearJoint        = 6
  @e_lineJoint        = 7
  @e_weldJoint        = 8
  @e_frictionJoint    = 9
  @e_inactiveLimit    = 0
  @e_atLowerLimit     = 1
  @e_atUpperLimit     = 2
  @e_equalLimits      = 3


  m_type                  : b2Joint.e_unknownJoint
  m_edgeA                 : null
  m_edgeB                 : null
  m_localAnchor1          : null
  m_localAnchor2          : null
  m_localCenterA          : null
  m_localCenterB          : null
  m_prev                  : null
  m_next                  : null
  m_bodyA                 : null
  m_bodyB                 : null
  m_collideConnected      : null
  m_islandFlag            : null
  m_userData              : null

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
    inv_dt = 0  if inv_dt is undefined
    null

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
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
    baumgarte = 0  if baumgarte is undefined
    false

