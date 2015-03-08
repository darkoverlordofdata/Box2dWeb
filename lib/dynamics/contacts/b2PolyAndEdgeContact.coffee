Box2D = require('../index')

b2Controller  = Box2D.Dynamics.Contacts.b2Contact
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Contacts.b2PolyAndEdgeContact extends b2Contact


  constructor: (manifold, polygon, xf1, edge, xf2) ->
    super
    return

  @Create = (allocator) ->
    new b2PolyAndEdgeContact()

  @Destroy = (contact, allocator) ->

  Reset: (fixtureA, fixtureB) ->
    super fixtureA, fixtureB
    b2Settings.b2Assert fixtureA.GetType() is b2Shape.e_polygonShape
    b2Settings.b2Assert fixtureB.GetType() is b2Shape.e_edgeShape
    return

  Evaluate: ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    @b2CollidePolyAndEdge @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2EdgeShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

