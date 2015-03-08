Box2D = require('../index')


class Box2D.Collision.b2ManifoldPoint

  constructor: ->
    @m_localPoint = new b2Vec2()
    @m_id = new b2ContactID()
    @Reset()
    return

  Reset: ->
    @m_localPoint.SetZero()
    @m_normalImpulse = 0.0
    @m_tangentImpulse = 0.0
    @m_id.key = 0
    return

  Set: (m) ->
    @m_localPoint.SetV m.m_localPoint
    @m_normalImpulse = m.m_normalImpulse
    @m_tangentImpulse = m.m_tangentImpulse
    @m_id.Set m.m_id
    return
 