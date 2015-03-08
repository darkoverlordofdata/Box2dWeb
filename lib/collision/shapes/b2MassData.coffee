Box2D = require('../../index')

b2Shape = Box2D.Collision.Shapes.b2Shape

class Box2D.Collision.Shapes.b2MassData extends b2Shape

  constructor: ->
    @mass = 0.0
    @center = new b2Vec2(0, 0)
    @I = 0.0
    return

