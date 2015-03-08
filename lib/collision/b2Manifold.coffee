Box2D = require('../index')


class Box2D.Collision.b2Manifold

  constructor: ->
    @m_pointCount = 0
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2ManifoldPoint()
      i++
    @m_localPlaneNormal = new b2Vec2()
    @m_localPoint = new b2Vec2()
    return

  Reset: ->
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Reset()
      i++
    @m_localPlaneNormal.SetZero()
    @m_localPoint.SetZero()
    @m_type = 0
    @m_pointCount = 0
    return

  Set: (m) ->
    @m_pointCount = m.m_pointCount
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Set m.m_points[i]
      i++
    @m_localPlaneNormal.SetV m.m_localPlaneNormal
    @m_localPoint.SetV m.m_localPoint
    @m_type = m.m_type
    return

  Copy: ->
    copy = new b2Manifold()
    copy.Set this
    copy

  @e_circles = 0x0001
  @e_faceA = 0x0002
  @e_faceB = 0x0004
