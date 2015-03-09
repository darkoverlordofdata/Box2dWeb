Box2D = require('../../index')

b2Contact         = Box2D.Dynamics.Contacts.b2Contact
b2Collision       = Box2D.Collision.b2Collision
b2PolygonShape    = Box2D.Collision.Shapes.b2PolygonShape

class Box2D.Dynamics.Contacts.b2PolygonContact extends b2Contact


  @Create = (allocator) ->
    return new b2PolygonContact()

  @Destroy = (contact, allocator) ->

  Reset: (fixtureA, fixtureB) ->
    super(fixtureA, fixtureB)
    return

  Evaluate: ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    b2Collision.CollidePolygons @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2PolygonShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return
