Box2D = require('../index')

b2Contact  = Box2D.Dynamics.Contacts.b2Contact

class Box2D.Dynamics.Contacts.b2PolyAndCircleContact extends b2Contact

  @Create = (allocator) ->
    return new b2PolyAndCircleContact()

  @Destroy = (contact, allocator) ->

  Reset: (fixtureA, fixtureB) ->
    super(fixtureA, fixtureB)
    b2Settings.b2Assert fixtureA.GetType() is b2Shape.e_polygonShape
    b2Settings.b2Assert fixtureB.GetType() is b2Shape.e_circleShape
    return

  Evaluate = ->
    bA = @m_fixtureA.m_body
    bB = @m_fixtureB.m_body
    b2Collision.CollidePolygonAndCircle @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

