Box2D = require('../index')


class Box2D.Collision.b2BoundValues

  lowerValues: null
  upperValues: null

  constructor: ->
    @lowerValues = new Box2D.Vector()
    @lowerValues[0] = 0.0
    @lowerValues[1] = 0.0
    @upperValues = new Box2D.Vector()
    @upperValues[0] = 0.0
    @upperValues[1] = 0.0
    return

