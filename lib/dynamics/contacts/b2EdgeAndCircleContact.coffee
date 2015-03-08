Box2D = require('../../index')

b2Contact  = Box2D.Dynamics.Contacts.b2Contact

class Box2D.Dynamics.Contacts.b2EdgeAndCircleContact extends b2Contact

  m_fixtureA: null
  m_fixtureB: null
  m_manifold: null

  @Create: (allocator) ->
    return new b2EdgeAndCircleContact()
  
  @Destroy: (contact, allocator) ->
  
  Reset: (fixtureA, fixtureB) ->
    super(fixtureA, fixtureB)
    return
  
  Evaluate: ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    @b2CollideEdgeAndCircle @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2EdgeShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return
  
  b2CollideEdgeAndCircle = (manifold, edge, xf1, circle, xf2) ->
