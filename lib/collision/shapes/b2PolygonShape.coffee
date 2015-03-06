Box2D = require('../../index')

class Box2D.Collision.Shapes.b2PolygonShape

  SetAsBox: (width, height) ->
    @type = "box"
    @width = width
    @height = height
    return

  SetAsEdge: (v1, v2) ->
    @type = "edge"
    @p1x = v1.x
    @p1y = v1.y
    @p2x = v2.x
    @p2y = v2.y
    return

  SetAsArray: (vec, length) ->
    @type = "polygon"
    @vertices = []
    i = 0

    while i < length
      @vertices.push vec[i].x
      @vertices.push vec[i].y
      i++
    return


