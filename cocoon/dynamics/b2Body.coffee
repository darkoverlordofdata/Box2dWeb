Box2D = require('../index')

b2Mat22     = Box2D.Common.Math.b2Mat22
b2Math      = Box2D.Common.Math.b2Math
b2Transform = Box2D.Common.Math.b2Transform
b2Vec2      = Box2D.Common.Math.b2Vec2
b2Fixture   = Box2D.Dynamics.b2Fixture

class Box2D.Dynamics.b2Body

  @b2_staticBody          = 0
  @b2_kinematicBody       = 1
  @b2_dynamicBody         = 1 # ??? b2Body.b2_dynamicBody = 2;

  m_world       : null
  m_bodyID      : null
  m_userData    : null
  m_xf          : null
  m_fixtures    : null
  m_active      : false
  m_prev        : null
  m_next        : null
  m_type        : null

  constructor: (bd, world) ->
    # Backup userdata and set it to null so Cocoon Doesn't read it
    userData = bd.userData
    bd.userData = null

    @m_world    = world
    @m_xf       = new b2Transform(bd.position, b2Mat22.FromAngle(bd.angle))
    @m_fixtures = []
    @m_active   = bd.active
    @m_type     = bd.type

    if (bd.type is b2Body.b2_staticBody)
      bd.density = 0


    @m_bodyID = window.ext.IDTK_SRV_BOX2D.makeCall('createBody', world.m_worldID, bd)
    @m_userData = userData

    # Restore userdata
    bd.userData = userData

  CreateFixture: (def) ->
    userData = def.userData
    def.userData = null

    fixtureID = window.ext.IDTK_SRV_BOX2D.makeCall('createFixture', @m_world.m_worldID, @m_bodyID, def)
    def.userData = userData

    fixture = new b2Fixture(this, userData, fixtureID, def)
    @m_world.m_fixturesList[fixtureID] = fixture
    @m_fixtures.push(fixture)
    return fixture

  GetFixtureList: ->
    if (@m_fixtures.length is 0)
      return null
    return @m_fixtures[0]

  DestroyFixture: (fixture) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('deleteFixture', @m_world.m_worldID, fixture.m_fixtureID)
    delete @m_world.m_fixturesList[fixture.m_fixtureID]
    return

  SetPositionAndAngle: (position, angle) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setBodyTransform', @m_world.m_worldID, @m_bodyID, position.x, position.y, angle)
    @m_xf.R.Set(angle)
    @m_xf.position.SetV(position)
    return

  GetPosition: ->
    return @m_xf.position

  SetPosition: (position) ->
    @SetPositionAndAngle(position, @GetAngle())
    return

  GetLinearVelocity: ->
    v = window.ext.IDTK_SRV_BOX2D.makeCall('getLinearVelocity', @m_world.m_worldID, @m_bodyID)
    return new b2Vec2(v[0],v[1])

  SetLinearVelocity: (vel) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setLinearVelocity', @m_world.m_worldID, @m_bodyID, vel.x, vel.y)
    return

  SetLinearDamping: (damp) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setLinearDamping', @m_world.m_worldID, @m_bodyID, damp)
    return

  GetWorldCenter: ->
    p = window.ext.IDTK_SRV_BOX2D.makeCall('getWorldCenter', @m_world.m_worldID, @m_bodyID)
    return new b2Vec2(p[0],p[1])

  GetLocalCenter: ->
    p = window.ext.IDTK_SRV_BOX2D.makeCall('getLocalCenter', @m_world.m_worldID, @m_bodyID)
    return new b2Vec2(p[0],p[1])

  GetLocalPoint: (worldPoint) ->
    return b2Math.MulXT(@m_xf, worldPoint)

  GetUserData: ->
    return @m_userData

  SetUserData: (data) ->
    @m_userData = data
    return

  GetMass: ->
    return window.ext.IDTK_SRV_BOX2D.makeCall('getMass', @m_world.m_worldID, @m_bodyID)

  IsAwake: ->
    return window.ext.IDTK_SRV_BOX2D.makeCall('isAwake', @m_world.m_worldID, @m_bodyID)

  SetAwake: (state) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setAwake', @m_world.m_worldID, @m_bodyID, state)
    return

  GetAngularVelocity: ->
    return window.ext.IDTK_SRV_BOX2D.makeCall('getAngularVelocity', @m_world.m_worldID, @m_bodyID)

  SetAngularVelocity: (angvel) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setAngularVelocity', @m_world.m_worldID, @m_bodyID, angvel)
    return

  SetFixedRotation: (fixed) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setFixedRotation', @m_world.m_worldID, @m_bodyID, fixed)
    return

  IsActive: ->
    return @m_active

  SetActive: (state) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('setActive', @m_world.m_worldID, @m_bodyID, state)
    @m_active = state
    return

  GetAngle: ->
    return @m_xf.R.GetAngle()

  SetAngle: (angle) ->
    if (angle is undefined)
      angle = 0

    @SetPositionAndAngle(@GetPosition(), angle)
    return

  SetType: (type) ->
    @m_type = type
    window.ext.IDTK_SRV_BOX2D.makeCall('setType', @m_world.m_worldID, @m_bodyID, type)
    return

  GetType: () ->
    return @m_type

  GetContactList: ->
    contacts = window.ext.IDTK_SRV_BOX2D.makeCall('getObjectContacts', @m_world.m_worldID, @m_bodyID)
    result = []
    for contact in contacts
      result.push(@m_world.m_bodyList[contact])

    return result

  GetWorld: ->
    return @m_world

  GetNext: ->
    return @m_next

  ApplyImpulse: (impulse, point, wake) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('applyImpulse', @m_world.m_worldID, @m_bodyID, impulse.x, impulse.y, point.x, point.y, wake)
    return

  ApplyForceToCenter: (force, wake) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('applyForceToCenter', @m_world.m_worldID, @m_bodyID, force.x, force.y, wake)
    return

  ApplyForce: (force, point, wake) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('applyForce', @m_world.m_worldID, @m_bodyID, force.x, force.y, point.x, point.y, wake)
    return

  ApplyTorque: (torque, wake) ->
    window.ext.IDTK_SRV_BOX2D.makeCall('applyTorque', @m_world.m_worldID, @m_bodyID, torque, wake)
    return


