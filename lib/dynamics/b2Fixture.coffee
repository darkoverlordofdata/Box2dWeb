Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2
b2FilterData = Box2D.Dynamics.b2FilterData

class Box2D.Dynamics.b2Fixture

  m_filter          : null
  m_shape           : null
  m_isSensor        : false
  m_body            : null
  m_next            : null
  m_userData        : null
  m_density         : null
  m_friction        : null
  m_restitution     : null
  m_aabb            : null
  m_proxy           : null

  constructor: ->
    @m_filter = new b2FilterData()
    @m_shape.GetType()

  GetShape: ->
    return @m_shape

  SetSensor: (sensor) ->
    return  if @m_isSensor is sensor
    @m_isSensor = sensor
    return  unless @m_body?
    edge = @m_body.GetContactList()
    while edge
      contact = edge.contact
      fixtureA = contact.GetFixtureA()
      fixtureB = contact.GetFixtureB()
      contact.SetSensor fixtureA.IsSensor() or fixtureB.IsSensor()  if fixtureA is this or fixtureB is this
      edge = edge.next
    return

  IsSensor: ->
    return @m_isSensor

  SetFilterData: (filter) ->
    @m_filter = filter.Copy()
    return  if @m_body
    edge = @m_body.GetContactList()
    while edge
      contact = edge.contact
      fixtureA = contact.GetFixtureA()
      fixtureB = contact.GetFixtureB()
      contact.FlagForFiltering()  if fixtureA is this or fixtureB is this
      edge = edge.next
    return

  GetFilterData: ->
    return @m_filter.Copy()

  GetBody: ->
    return @m_body

  GetNext: ->
    return @m_next

  GetUserData: ->
    return @m_userData

  SetUserData: (data) ->
    return @m_userData = data
    return

  TestPoint: (p) ->
    return @m_shape.TestPoint @m_body.GetTransform(), p

  RayCast: (output, input) ->
    return @m_shape.RayCast output, input, @m_body.GetTransform()

  GetMassData: (massData) ->
    massData = null  if massData is `undefined`
    massData = new b2MassData()  unless massData?
    @m_shape.ComputeMass massData, @m_density
    return massData

  SetDensity: (density) ->
    density = 0  if density is `undefined`
    @m_density = density
    return

  GetDensity: ->
    @m_density

  GetFriction: ->
    return @m_friction

  SetFriction: (friction) ->
    friction = 0  if friction is `undefined`
    @m_friction = friction
    return

  GetRestitution: ->
    @m_restitution

  SetRestitution: (restitution) ->
    restitution = 0  if restitution is `undefined`
    @m_restitution = restitution
    return

  GetAABB: ->
    return @m_aabb

  b2Fixture::b2Fixture = ->
    @m_aabb = new b2AABB()
    @m_userData = null
    @m_body = null
    @m_next = null
    @m_shape = null
    @m_density = 0.0
    @m_friction = 0.0
    @m_restitution = 0.0
    return

  Create: (body, xf, def) ->
    @m_userData = def.userData
    @m_friction = def.friction
    @m_restitution = def.restitution
    @m_body = body
    @m_next = null
    @m_filter = def.filter.Copy()
    @m_isSensor = def.isSensor
    @m_shape = def.shape.Copy()
    @m_density = def.density
    return

  Destroy: ->
    @m_shape = null
    return

  CreateProxy: (broadPhase, xf) ->
    @m_shape.ComputeAABB @m_aabb, xf
    @m_proxy = broadPhase.CreateProxy(@m_aabb, this)
    return

  DestroyProxy: (broadPhase) ->
    return  unless @m_proxy?
    broadPhase.DestroyProxy @m_proxy
    @m_proxy = null
    return

  Synchronize: (broadPhase, transform1, transform2) ->
    return  unless @m_proxy
    aabb1 = new b2AABB()
    aabb2 = new b2AABB()
    @m_shape.ComputeAABB aabb1, transform1
    @m_shape.ComputeAABB aabb2, transform2
    @m_aabb.Combine aabb1, aabb2
    displacement = b2Math.SubtractVV(transform2.position, transform1.position)
    broadPhase.MoveProxy @m_proxy, @m_aabb, displacement
    return
