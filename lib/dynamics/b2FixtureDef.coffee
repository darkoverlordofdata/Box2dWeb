Box2D = require('../index')

b2FilterData = Box2D.Dynamics.b2FilterData

class Box2D.Dynamics.b2FixtureDef

  shape           : null
  userData        : null
  friction        : 0.2
  restitution     : 0.0
  density         : 0.0
  isSensor        : false
  filter          : null

  b2FixtureDef::b2FixtureDef = ->
    @filter = new b2FilterData()
    @shape = null
    @userData = null
    @friction = 0.2
    @restitution = 0.0
    @density = 0.0
    @filter.categoryBits = 0x0001
    @filter.maskBits = 0xFFFF
    @filter.groupIndex = 0
    @isSensor = false
    return

