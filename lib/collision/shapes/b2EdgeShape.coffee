Box2D = require('../../index')

b2Shape = Box2D.Collision.Shapes.b2Shape

class Box2D.Collision.Shapes.b2EdgeShape extends b2Shape

  constructor: ->
    super
    @s_supportVec = new b2Vec2()
    @m_v1 = new b2Vec2()
    @m_v2 = new b2Vec2()
    @m_coreV1 = new b2Vec2()
    @m_coreV2 = new b2Vec2()
    @m_normal = new b2Vec2()
    @m_direction = new b2Vec2()
    @m_cornerDir1 = new b2Vec2()
    @m_cornerDir2 = new b2Vec2()
    @m_type = b2Shape.e_edgeShape
    @m_prevEdge = null
    @m_nextEdge = null
    @m_v1 = v1
    @m_v2 = v2
    @m_direction.Set @m_v2.x - @m_v1.x, @m_v2.y - @m_v1.y
    @m_length = @m_direction.Normalize()
    @m_normal.Set @m_direction.y, (-@m_direction.x)
    @m_coreV1.Set (-b2Settings.b2_toiSlop * (@m_normal.x - @m_direction.x)) + @m_v1.x, (-b2Settings.b2_toiSlop * (@m_normal.y - @m_direction.y)) + @m_v1.y
    @m_coreV2.Set (-b2Settings.b2_toiSlop * (@m_normal.x + @m_direction.x)) + @m_v2.x, (-b2Settings.b2_toiSlop * (@m_normal.y + @m_direction.y)) + @m_v2.y
    @m_cornerDir1 = @m_normal
    @m_cornerDir2.Set (-@m_normal.x), (-@m_normal.y)
    return

  TestPoint: (transform, p) ->
    false

  RayCast: (output, input, transform) ->
    tMat = undefined
    rX = input.p2.x - input.p1.x
    rY = input.p2.y - input.p1.y
    tMat = transform.R
    v1X = transform.position.x + (tMat.col1.x * @m_v1.x + tMat.col2.x * @m_v1.y)
    v1Y = transform.position.y + (tMat.col1.y * @m_v1.x + tMat.col2.y * @m_v1.y)
    nX = transform.position.y + (tMat.col1.y * @m_v2.x + tMat.col2.y * @m_v2.y) - v1Y
    nY = (-(transform.position.x + (tMat.col1.x * @m_v2.x + tMat.col2.x * @m_v2.y) - v1X))
    k_slop = 100.0 * Number.MIN_VALUE
    denom = (-(rX * nX + rY * nY))
    if denom > k_slop
      bX = input.p1.x - v1X
      bY = input.p1.y - v1Y
      a = (bX * nX + bY * nY)
      if 0.0 <= a and a <= input.maxFraction * denom
        mu2 = (-rX * bY) + rY * bX
        if (-k_slop * denom) <= mu2 and mu2 <= denom * (1.0 + k_slop)
          a /= denom
          output.fraction = a
          nLen = Math.sqrt(nX * nX + nY * nY)
          output.normal.x = nX / nLen
          output.normal.y = nY / nLen
          return true
    false

  ComputeAABB: (aabb, transform) ->
    tMat = transform.R
    v1X = transform.position.x + (tMat.col1.x * @m_v1.x + tMat.col2.x * @m_v1.y)
    v1Y = transform.position.y + (tMat.col1.y * @m_v1.x + tMat.col2.y * @m_v1.y)
    v2X = transform.position.x + (tMat.col1.x * @m_v2.x + tMat.col2.x * @m_v2.y)
    v2Y = transform.position.y + (tMat.col1.y * @m_v2.x + tMat.col2.y * @m_v2.y)
    if v1X < v2X
      aabb.lowerBound.x = v1X
      aabb.upperBound.x = v2X
    else
      aabb.lowerBound.x = v2X
      aabb.upperBound.x = v1X
    if v1Y < v2Y
      aabb.lowerBound.y = v1Y
      aabb.upperBound.y = v2Y
    else
      aabb.lowerBound.y = v2Y
      aabb.upperBound.y = v1Y
    return

  ComputeMass: (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = 0
    massData.center.SetV @m_v1
    massData.I = 0
    return

  ComputeSubmergedArea: (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    v0 = new b2Vec2(normal.x * offset, normal.y * offset)
    v1 = b2Math.MulX(xf, @m_v1)
    v2 = b2Math.MulX(xf, @m_v2)
    d1 = b2Math.Dot(normal, v1) - offset
    d2 = b2Math.Dot(normal, v2) - offset
    if d1 > 0
      if d2 > 0
        return 0
      else
        v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x
        v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y
    else
      if d2 > 0
        v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x
        v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y
      else
    c.x = (v0.x + v1.x + v2.x) / 3
    c.y = (v0.y + v1.y + v2.y) / 3
    0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x))

  GetLength: ->
    @m_length

  GetVertex1: ->
    @m_v1

  GetVertex2: ->
    @m_v2

  GetCoreVertex1: ->
    @m_coreV1

  GetCoreVertex2: ->
    @m_coreV2

  GetNormalVector: ->
    @m_normal

  GetDirectionVector: ->
    @m_direction

  GetCorner1Vector: ->
    @m_cornerDir1

  GetCorner2Vector: ->
    @m_cornerDir2

  Corner1IsConvex: ->
    @m_cornerConvex1

  Corner2IsConvex: ->
    @m_cornerConvex2

  GetFirstVertex: (xf) ->
    tMat = xf.R
    new b2Vec2(xf.position.x + (tMat.col1.x * @m_coreV1.x + tMat.col2.x * @m_coreV1.y), xf.position.y + (tMat.col1.y * @m_coreV1.x + tMat.col2.y * @m_coreV1.y))

  GetNextEdge: ->
    @m_nextEdge

  GetPrevEdge: ->
    @m_prevEdge

  Support: (xf, dX, dY) ->
    dX = 0  if dX is `undefined`
    dY = 0  if dY is `undefined`
    tMat = xf.R
    v1X = xf.position.x + (tMat.col1.x * @m_coreV1.x + tMat.col2.x * @m_coreV1.y)
    v1Y = xf.position.y + (tMat.col1.y * @m_coreV1.x + tMat.col2.y * @m_coreV1.y)
    v2X = xf.position.x + (tMat.col1.x * @m_coreV2.x + tMat.col2.x * @m_coreV2.y)
    v2Y = xf.position.y + (tMat.col1.y * @m_coreV2.x + tMat.col2.y * @m_coreV2.y)
    if (v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)
      @s_supportVec.x = v1X
      @s_supportVec.y = v1Y
    else
      @s_supportVec.x = v2X
      @s_supportVec.y = v2Y
    @s_supportVec

  SetPrevEdge: (edge, core, cornerDir, convex) ->
    @m_prevEdge = edge
    @m_coreV1 = core
    @m_cornerDir1 = cornerDir
    @m_cornerConvex1 = convex
    return

  SetNextEdge: (edge, core, cornerDir, convex) ->
    @m_nextEdge = edge
    @m_coreV2 = core
    @m_cornerDir2 = cornerDir
    @m_cornerConvex2 = convex
    return

 