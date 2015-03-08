Box2D = require('../index')


class Box2D.Collision.ClipVertex

  constructor: ->
    @v = new b2Vec2()
    @id = new b2ContactID()
    return

  Set: (other) ->
    @v.SetV other.v
    @id.Set other.id
    return
