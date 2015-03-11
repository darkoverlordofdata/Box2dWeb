class Box2D.Dynamics.b2FixtureDef

  shape           : null
  userData        : null
  friction        : 0.2
  restitution     : 0.0
  density         : 0.0
  isSensor        : false
  filter          : null

  constructor: ->
    @filter = new b2FilterData()
    @filter.categoryBits = 0x0001
    @filter.maskBits = 0xFFFF
    @filter.groupIndex = 0
    return

