Box2D = require('../index')

b2Controller  = Box2D.Dynamics.Contacts.b2Contact
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22

class Box2D.Dynamics.Contacts.b2EdgeAndCircleContact extends b2Contact

  m_fixtureA: null
  m_fixtureB: null
  m_manifold: null

  constructor: ->
    super
    return
  
  @Create: (allocator) ->
    new b2EdgeAndCircleContact()
  
  @Destroy: (contact, allocator) ->
  
  Reset: (fixtureA, fixtureB) ->
    super fixtureA, fixtureB
    return
  
  Evaluate: ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    @b2CollideEdgeAndCircle @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2EdgeShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return
  
  b2CollideEdgeAndCircle = (manifold, edge, xf1, circle, xf2) ->
