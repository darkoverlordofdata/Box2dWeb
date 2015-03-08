Box2D = require('../../index')

b2Shape         = Box2D.Collision.Shapes.b2Shape
b2Vec2          = Box2D.Common.Math.b2Vec2

class Box2D.Collision.Shapes.b2MassData extends b2Shape

  mass    : 0.0
  center  : null
  I       : 0.0

  constructor: ->
    @center = new b2Vec2(0, 0)
    return

