Box2D = require('../index')

b2Vec2          = Box2D.Common.Math.b2Vec2


class Box2D.Collision.b2RayCastOutput

  normal: null

  
  constructor: ->
    @normal = new b2Vec2()
    return
