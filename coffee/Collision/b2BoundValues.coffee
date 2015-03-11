class Box2D.Collision.b2BoundValues

  lowerValues: null
  upperValues: null

  constructor: ->
    @lowerValues = new Vector()
    @lowerValues[0] = 0.0
    @lowerValues[1] = 0.0
    @upperValues = new Vector()
    @upperValues[0] = 0.0
    @upperValues[1] = 0.0
    return

