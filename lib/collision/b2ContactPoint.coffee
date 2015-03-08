Box2D = require('../index')


class Box2D.Collision.b2ContactPoint

  constructor: ->
    @position = new b2Vec2()
    @velocity = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return
