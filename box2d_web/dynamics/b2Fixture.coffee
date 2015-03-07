Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.b2Fixture

  m_body            : null
  m_userData        : null
  m_fixtureID       : null
  m_shape           : null
  m_isSensor        : false
  m_density         : null
  m_friction        : null
  m_restitution     : null

  constructor: (body, userData, fixtureID, def) ->
    @m_body = body
    @m_userData = userData
    @m_fixtureID = fixtureID
    @m_shape = {}
    @m_shape.m_centroid = new b2Vec2()
    @m_isSensor = false
    @m_density = def.density
    @m_friction = def.friction
    @m_restitution = def.restitution
    @m_isSensor = def.isSensor
    return

  GetBody: ->
    @m_body

  GetShape: ->
    console.log "fixture.GetShape not yet supported in CocoonJS Box2D binding"
    return null

  GetUserData: ->
    return @m_userData

  SetSensor: (isSensor) ->
    @m_isSensor = isSensor
    window.ext.IDTK_SRV_BOX2D.makeCall "setSensor", @m_body.m_world.m_worldID, @m_fixtureID, @m_isSensor
    return

  IsSensor: ->
    return @m_isSensor

  SetDensity: (density) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "setDensity", @m_body.m_world.m_worldID, @m_fixtureID, density
    @m_density = density
    return

  SetFriction: (friction) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "setFriction", @m_body.m_world.m_worldID, @m_fixtureID, friction
    @m_friction = friction
    return

  SetRestitution: (restitution) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "setRestitution", @m_body.m_world.m_worldID, @m_fixtureID, restitution
    @m_restitution = restitution
    return

  GetDensity: ->
    return @m_density

  GetFriction: ->
    return @m_friction

  GetRestitution: ->
    return @m_restitution

