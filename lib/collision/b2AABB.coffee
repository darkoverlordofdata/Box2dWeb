Box2D = require('../index')

b2Vec2          = Box2D.Common.Math.b2Vec2

class Box2D.Collision.b2AABB

  lowerBound: null
  upperBound: null


  constructor: ->
    @lowerBound = new b2Vec2()
    @upperBound = new b2Vec2()
    return

  IsValid: ->
    dX = @upperBound.x - @lowerBound.x
    dY = @upperBound.y - @lowerBound.y
    valid = dX >= 0.0 and dY >= 0.0
    valid = valid and @lowerBound.IsValid() and @upperBound.IsValid()
    valid

  GetCenter: ->
    new b2Vec2((@lowerBound.x + @upperBound.x) / 2, (@lowerBound.y + @upperBound.y) / 2)

  GetExtents: ->
    new b2Vec2((@upperBound.x - @lowerBound.x) / 2, (@upperBound.y - @lowerBound.y) / 2)

  Contains: (aabb) ->
    result = true
    result = result and @lowerBound.x <= aabb.lowerBound.x
    result = result and @lowerBound.y <= aabb.lowerBound.y
    result = result and aabb.upperBound.x <= @upperBound.x
    result = result and aabb.upperBound.y <= @upperBound.y
    result

  RayCast: (output, input) ->
    tmin = (-Number.MAX_VALUE)
    tmax = Number.MAX_VALUE
    pX = input.p1.x
    pY = input.p1.y
    dX = input.p2.x - input.p1.x
    dY = input.p2.y - input.p1.y
    absDX = Math.abs(dX)
    absDY = Math.abs(dY)
    normal = output.normal
    inv_d = 0
    t1 = 0
    t2 = 0
    t3 = 0
    s = 0
    if absDX < Number.MIN_VALUE
      return false  if pX < @lowerBound.x or @upperBound.x < pX
    else
      inv_d = 1.0 / dX
      t1 = (@lowerBound.x - pX) * inv_d
      t2 = (@upperBound.x - pX) * inv_d
      s = (-1.0)
      if t1 > t2
        t3 = t1
        t1 = t2
        t2 = t3
        s = 1.0
      if t1 > tmin
        normal.x = s
        normal.y = 0
        tmin = t1
      tmax = Math.min(tmax, t2)
      return false  if tmin > tmax
    if absDY < Number.MIN_VALUE
      return false  if pY < @lowerBound.y or @upperBound.y < pY
    else
      inv_d = 1.0 / dY
      t1 = (@lowerBound.y - pY) * inv_d
      t2 = (@upperBound.y - pY) * inv_d
      s = (-1.0)
      if t1 > t2
        t3 = t1
        t1 = t2
        t2 = t3
        s = 1.0
      if t1 > tmin
        normal.y = s
        normal.x = 0
        tmin = t1
      tmax = Math.min(tmax, t2)
      return false  if tmin > tmax
    output.fraction = tmin
    true

  TestOverlap: (other) ->
    d1X = other.lowerBound.x - @upperBound.x
    d1Y = other.lowerBound.y - @upperBound.y
    d2X = @lowerBound.x - other.upperBound.x
    d2Y = @lowerBound.y - other.upperBound.y
    return false  if d1X > 0.0 or d1Y > 0.0
    return false  if d2X > 0.0 or d2Y > 0.0
    true

  @Combine: (aabb1, aabb2) ->
    aabb = new b2AABB()
    aabb.Combine aabb1, aabb2
    aabb

  Combine: (aabb1, aabb2) ->
    @lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x)
    @lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y)
    @upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x)
    @upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y)
    return
 