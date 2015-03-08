Box2D = require('../index')

class Box2D.Dynamics.b2FilterData

  categoryBits  : 0
  maskBits      : 0
  groupIndex    : 0

  constructor: ->
    @categoryBits = 0x0001
    @maskBits = 0xFFFF
    @groupIndex = 0
    return

  Copy: ->
    copy = new b2FilterData()
    copy.categoryBits = @categoryBits
    copy.maskBits = @maskBits
    copy.groupIndex = @groupIndex
    return copy

