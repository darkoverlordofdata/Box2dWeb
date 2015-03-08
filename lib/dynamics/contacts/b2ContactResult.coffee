Box2D = require('../index')

b2ContactID   = Box2D.Collision.b2ContactID
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Contacts.b2ContactResult

  position    : null
  normal      : null
  id          : null

  constructor: ->
    @position = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return
