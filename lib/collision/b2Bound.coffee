Box2D = require('../index')


class Box2D.Collision.b2Bound

  IsLower: ->
    (@value & 1) is 0

  IsUpper: ->
    (@value & 1) is 1

  Swap: (b) ->
    tempValue = @value
    tempProxy = @proxy
    tempStabbingCount = @stabbingCount
    @value = b.value
    @proxy = b.proxy
    @stabbingCount = b.stabbingCount
    b.value = tempValue
    b.proxy = tempProxy
    b.stabbingCount = tempStabbingCount
    return
