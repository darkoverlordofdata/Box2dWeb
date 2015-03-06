Box2D = require('../index')

b2Mat22     = Box2D.Common.Math.b2Mat22
b2Math      = Box2D.Common.Math.b2Math
b2Transform = Box2D.Common.Math.b2Transform
b2Vec2      = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.b2Body

  @b2_staticBody          = 0
  @b2_kinematicBody       = 1
  @b2_dynamicBody         = 1
  @s_xf1                  = new b2Transform()
  @e_islandFlag           = 0x0001
  @e_awakeFlag            = 0x0002
  @e_allowSleepFlag       = 0x0004
  @e_bulletFlag           = 0x0008
  @e_fixedRotationFlag    = 0x0010
  @e_activeFlag           = 0x0020

  m_flags: 0
  m_type: 0
  m_world: null
  m_xf: null
  m_sweep: null
  m_jointList: null
  m_controllerList: null
  m_contactList: null
  m_controllerCount: 0
  m_prev: null
  m_next: null
  m_linearVelocity: null
  m_angularVelocity: null
  m_linearDamping: null
  m_angularDamping: null
  m_force: null
  m_torque: 0.0
  m_sleepTime: 0.0
  m_mass: 0
  m_invMass: 0
  m_I: 0.0
  m_invI: 0.0
  m_inertiaScale = 0.0
  m_userData = null
  m_fixtureList = null
  m_fixtureCount = 0


  constructor: (bd, world) ->
    @m_xf = new b2Transform();
    @m_sweep = new b2Sweep();
    @m_linearVelocity = new b2Vec2();
    @m_force = new b2Vec2();

    @m_flags = 0
    if (bd.bullet)
      @m_flags |= b2Body.e_bulletFlag

    if (bd.fixedRotation)
      @m_flags |= b2Body.e_fixedRotationFlag

    if (bd.allowSleep)
      @m_flags |= b2Body.e_allowSleepFlag

    if (bd.awake)
      @m_flags |= b2Body.e_awakeFlag

    if (bd.active)
      @m_flags |= b2Body.e_activeFlag

    @m_world = world
    @m_xf.position.SetV(bd.position)
    @m_xf.R.Set(bd.angle)
    @m_sweep.localCenter.SetZero()
    @m_sweep.t0 = 1.0
    @m_sweep.a0 = @m_sweep.a = bd.angle
    tMat = @m_xf.R
    tVec = @m_sweep.localCenter
    @m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    @m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    @m_sweep.c.x += @m_xf.position.x
    @m_sweep.c.y += @m_xf.position.y
    @m_sweep.c0.SetV(@m_sweep.c)
    @m_jointList = null
    @m_controllerList = null
    @m_contactList = null
    @m_controllerCount = 0
    @m_prev = null
    @m_next = null
    @m_linearVelocity.SetV(bd.linearVelocity)
    @m_angularVelocity = bd.angularVelocity
    @m_linearDamping = bd.linearDamping
    @m_angularDamping = bd.angularDamping
    @m_force.Set(0.0, 0.0)
    @m_torque = 0.0
    @m_sleepTime = 0.0
    @m_type = bd.type
    if (@m_type == b2Body.b2_dynamicBody)
      @m_mass = 1.0
      @m_invMass = 1.0

    else
      @m_mass = 0.0
      @m_invMass = 0.0

    @m_I = 0.0
    @m_invI = 0.0
    @m_inertiaScale = bd.inertiaScale
    @m_userData = bd.userData
    @m_fixtureList = null
    @m_fixtureCount = 0

  CreateFixture: (def) ->

  GetFixtureList: ->

  DestroyFixture: (fixture) ->

  SetPositionAndAngle: (position, angle) ->

  GetPosition: ->

  SetPosition: (position) ->

  GetLinearVelocity: ->

  GetWorldCenter: ->

  GetLocalCenter: ->

  GetLocalPoint: (worldPoint) ->

  ApplyImpulse: (impulse, point, wake) ->

  GetMass: ->

  IsAwake: ->

  GetAngularVelocity: ->

  SetFixedRotation: (fixed) ->

  SetAwake: (state) ->

  SetLinearVelocity: (vel) ->

  ApplyForceToCenter: (force, wake) ->

  ApplyForce: (force, point, wake) ->

  ApplyTorque: (torque, wake) ->

  SetLinearDamping: (damp) ->

  SetAngularVelocity: (angvel) ->

  SetType: (type) ->

  SetActive: (state) ->

  IsActive: ->

  GetAngle: ->

  SetAngle: (angle) ->

  GetContactList: ->

  SetUserData: (data) ->

  GetUserData: ->
    @m_userData

  GetWorld: ->
    @m_world

