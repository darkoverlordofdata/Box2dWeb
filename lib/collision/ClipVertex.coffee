Box2D = require('../index')

b2Vec2          = Box2D.Common.Math.b2Vec2
b2ContactID     = Box2D.Collision.b2ContactID

class Box2D.Collision.ClipVertex

  v     : null
  id    : null

  constructor: ->
    @v = new b2Vec2()
    @id = new b2ContactID()
    return

  Set: (other) ->
    @v.SetV other.v
    @id.Set other.id
    return
