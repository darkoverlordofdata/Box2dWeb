Box2D = require('../index')

b2Vec2          = Box2D.Common.Math.b2Vec2
b2ContactID     = Box2D.Collision.b2ContactID

class Box2D.Collision.b2ContactPoint

  position    : null
  velocity    : null
  normal      : null
  id          : null

  constructor: ->
    @position = new b2Vec2()
    @velocity = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return
