Box2D = require('../index')

b2Vec2          = Box2D.Common.Math.b2Vec2

class Box2D.Collision.b2DistanceInput

  pointA      : null
  pointB      : null

  constructor: ->
    @pointA = new b2Vec2()
    @pointB = new b2Vec2()
    return
