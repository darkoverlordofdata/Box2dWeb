Box2D = require('../index')

#Array = Box2D.Array
b2Settings        = Box2D.Common.b2Settings
b2Shape           = Box2D.Collision.Shapes.b2Shape
b2CircleShape     = Box2D.Collision.Shapes.b2CircleShape
b2PolygonShape    = Box2D.Collision.Shapes.b2PolygonShape

class Box2D.Collision.b2DistanceProxy

  m_vertices      : null
  m_count         : 0
  m_radius        : 0



  Set: (shape) ->
    switch shape.GetType()
      when b2Shape.e_circleShape
        circle = ((if shape instanceof b2CircleShape then shape else null))
        @m_vertices = new Array(1, true)
        @m_vertices[0] = circle.m_p
        @m_count = 1
        @m_radius = circle.m_radius
      when b2Shape.e_polygonShape
        polygon = ((if shape instanceof b2PolygonShape then shape else null))
        @m_vertices = polygon.m_vertices
        @m_count = polygon.m_vertexCount
        @m_radius = polygon.m_radius
      else
        b2Settings.b2Assert false
    return

  GetSupport: (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    bestIndex

  GetSupportVertex: (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    @m_vertices[bestIndex]

  GetVertexCount: ->
    @m_count

  GetVertex: (index) ->
    index = 0  if index is undefined
    b2Settings.b2Assert 0 <= index and index < @m_count
    @m_vertices[index]
 