Box2D = require('../index')

Vector = Box2D.Vector

class Box2D.Collision.b2SimplexCache

  indexA: null
  indexB: null

  constructor: ->
    @indexA = new Vector(3)
    @indexB = new Vector(3)
    return
