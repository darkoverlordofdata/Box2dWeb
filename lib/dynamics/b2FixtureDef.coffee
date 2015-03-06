Box2D = require('../index')

class Box2D.Dynamics.b2FixtureDef

  constructor: ->
    @shape = null
    @userData = null
    @friction = 0.2
    @restitution = 0.0
    @density = 0.0
    @isSensor = false
    @filter =
      categoryBits: 1
      maskBits: 0xFFFF
      groupIndex: 0

    return


