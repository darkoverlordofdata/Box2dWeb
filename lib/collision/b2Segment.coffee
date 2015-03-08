Box2D = require('../index')


class Box2D.Collision.b2Segment

  constructor: ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    return

  TestSegment: (lambda, normal, segment, maxLambda) ->
    maxLambda = 0  if maxLambda is `undefined`
    s = segment.p1
    rX = segment.p2.x - s.x
    rY = segment.p2.y - s.y
    dX = @p2.x - @p1.x
    dY = @p2.y - @p1.y
    nX = dY
    nY = (-dX)
    k_slop = 100.0 * Number.MIN_VALUE
    denom = (-(rX * nX + rY * nY))
    if denom > k_slop
      bX = s.x - @p1.x
      bY = s.y - @p1.y
      a = (bX * nX + bY * nY)
      if 0.0 <= a and a <= maxLambda * denom
        mu2 = (-rX * bY) + rY * bX
        if (-k_slop * denom) <= mu2 and mu2 <= denom * (1.0 + k_slop)
          a /= denom
          nLen = Math.sqrt(nX * nX + nY * nY)
          nX /= nLen
          nY /= nLen
          lambda[0] = a
          normal.Set nX, nY
          return true
    false

  Extend: (aabb) ->
    @ExtendForward aabb
    @ExtendBackward aabb
    return

  ExtendForward: (aabb) ->
    dX = @p2.x - @p1.x
    dY = @p2.y - @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p1.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p1.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p1.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p1.y) / dY else Number.POSITIVE_INFINITY)))
    @p2.x = @p1.x + dX * lambda
    @p2.y = @p1.y + dY * lambda
    return

  ExtendBackward: (aabb) ->
    dX = (-@p2.x) + @p1.x
    dY = (-@p2.y) + @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p2.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p2.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p2.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p2.y) / dY else Number.POSITIVE_INFINITY)))
    @p1.x = @p2.x + dX * lambda
    @p1.y = @p2.y + dY * lambda
    return
