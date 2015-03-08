Box2D = require('../index')

class Box2D.Dynamics.b2FilterData

  categoryBits  : 0x0001
  maskBits      : 0xFFFF
  groupIndex    : 0

  Copy: ->
    copy = new b2FilterData()
    copy.categoryBits = @categoryBits
    copy.maskBits = @maskBits
    copy.groupIndex = @groupIndex
    return copy

