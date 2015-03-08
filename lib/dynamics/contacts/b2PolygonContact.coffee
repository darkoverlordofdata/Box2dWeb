Box2D = require('../index')

b2Controller  = Box2D.Dynamics.Contacts.b2Contact
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Contacts.b2PolygonContact extends b2Contact

  constructor: ->
    super
    return

  @Create = (allocator) ->
    new b2PolygonContact()

  @Destroy = (contact, allocator) ->

  Reset: (fixtureA, fixtureB) ->
    super fixtureA, fixtureB
    return

  Evaluate: ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    b2Collision.CollidePolygons @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2PolygonShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return
