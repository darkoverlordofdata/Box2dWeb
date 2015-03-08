(->
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2Body = Box2D.Dynamics.b2Body
  b2BodyDef = Box2D.Dynamics.b2BodyDef
  b2ContactFilter = Box2D.Dynamics.b2ContactFilter
  b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse
  b2ContactListener = Box2D.Dynamics.b2ContactListener
  b2ContactManager = Box2D.Dynamics.b2ContactManager
  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
  b2DestructionListener = Box2D.Dynamics.b2DestructionListener
  b2FilterData = Box2D.Dynamics.b2FilterData
  b2Fixture = Box2D.Dynamics.b2Fixture
  b2FixtureDef = Box2D.Dynamics.b2FixtureDef
  b2Island = Box2D.Dynamics.b2Island
  b2TimeStep = Box2D.Dynamics.b2TimeStep
  b2World = Box2D.Dynamics.b2World
  b2AABB = Box2D.Collision.b2AABB
  b2Bound = Box2D.Collision.b2Bound
  b2BoundValues = Box2D.Collision.b2BoundValues
  b2Collision = Box2D.Collision.b2Collision
  b2ContactID = Box2D.Collision.b2ContactID
  b2ContactPoint = Box2D.Collision.b2ContactPoint
  b2Distance = Box2D.Collision.b2Distance
  b2DistanceInput = Box2D.Collision.b2DistanceInput
  b2DistanceOutput = Box2D.Collision.b2DistanceOutput
  b2DistanceProxy = Box2D.Collision.b2DistanceProxy
  b2DynamicTree = Box2D.Collision.b2DynamicTree
  b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase
  b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode
  b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair
  b2Manifold = Box2D.Collision.b2Manifold
  b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint
  b2Point = Box2D.Collision.b2Point
  b2RayCastInput = Box2D.Collision.b2RayCastInput
  b2RayCastOutput = Box2D.Collision.b2RayCastOutput
  b2Segment = Box2D.Collision.b2Segment
  b2SeparationFunction = Box2D.Collision.b2SeparationFunction
  b2Simplex = Box2D.Collision.b2Simplex
  b2SimplexCache = Box2D.Collision.b2SimplexCache
  b2SimplexVertex = Box2D.Collision.b2SimplexVertex
  b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact
  b2TOIInput = Box2D.Collision.b2TOIInput
  b2WorldManifold = Box2D.Collision.b2WorldManifold
  ClipVertex = Box2D.Collision.ClipVertex
  Features = Box2D.Collision.Features
  IBroadPhase = Box2D.Collision.IBroadPhase
  Box2D.inherit b2CircleShape, Box2D.Collision.Shapes.b2Shape
  b2CircleShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2CircleShape.b2CircleShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    @m_p = new b2Vec2()
    return

  b2CircleShape::Copy = ->
    s = new b2CircleShape()
    s.Set this
    s

  b2CircleShape::Set = (other) ->
    @__super.Set.call this, other
    if Box2D.is(other, b2CircleShape)
      other2 = ((if other instanceof b2CircleShape then other else null))
      @m_p.SetV other2.m_p
    return

  b2CircleShape::TestPoint = (transform, p) ->
    tMat = transform.R
    dX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    dY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    dX = p.x - dX
    dY = p.y - dY
    (dX * dX + dY * dY) <= @m_radius * @m_radius

  b2CircleShape::RayCast = (output, input, transform) ->
    tMat = transform.R
    positionX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    positionY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    sX = input.p1.x - positionX
    sY = input.p1.y - positionY
    b = (sX * sX + sY * sY) - @m_radius * @m_radius
    rX = input.p2.x - input.p1.x
    rY = input.p2.y - input.p1.y
    c = (sX * rX + sY * rY)
    rr = (rX * rX + rY * rY)
    sigma = c * c - rr * b
    return false  if sigma < 0.0 or rr < Number.MIN_VALUE
    a = (-(c + Math.sqrt(sigma)))
    if 0.0 <= a and a <= input.maxFraction * rr
      a /= rr
      output.fraction = a
      output.normal.x = sX + a * rX
      output.normal.y = sY + a * rY
      output.normal.Normalize()
      return true
    false

  b2CircleShape::ComputeAABB = (aabb, transform) ->
    tMat = transform.R
    pX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    pY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    aabb.lowerBound.Set pX - @m_radius, pY - @m_radius
    aabb.upperBound.Set pX + @m_radius, pY + @m_radius
    return

  b2CircleShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = density * b2Settings.b2_pi * @m_radius * @m_radius
    massData.center.SetV @m_p
    massData.I = massData.mass * (0.5 * @m_radius * @m_radius + (@m_p.x * @m_p.x + @m_p.y * @m_p.y))
    return

  b2CircleShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    p = b2Math.MulX(xf, @m_p)
    l = (-(b2Math.Dot(normal, p) - offset))
    return 0  if l < (-@m_radius) + Number.MIN_VALUE
    if l > @m_radius
      c.SetV p
      return Math.PI * @m_radius * @m_radius
    r2 = @m_radius * @m_radius
    l2 = l * l
    area = r2 * (Math.asin(l / @m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2)
    com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area)
    c.x = p.x + normal.x * com
    c.y = p.y + normal.y * com
    area

  b2CircleShape::GetLocalPosition = ->
    @m_p

  b2CircleShape::SetLocalPosition = (position) ->
    @m_p.SetV position
    return

  b2CircleShape::GetRadius = ->
    @m_radius

  b2CircleShape::SetRadius = (radius) ->
    radius = 0  if radius is `undefined`
    @m_radius = radius
    return

  b2CircleShape::b2CircleShape = (radius) ->
    radius = 0  if radius is `undefined`
    @__super.b2Shape.call this
    @m_type = b2Shape.e_circleShape
    @m_radius = radius
    return

  b2EdgeChainDef.b2EdgeChainDef = ->

  b2EdgeChainDef::b2EdgeChainDef = ->
    @vertexCount = 0
    @isALoop = true
    @vertices = []
    return

  Box2D.inherit b2EdgeShape, Box2D.Collision.Shapes.b2Shape
  b2EdgeShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2EdgeShape.b2EdgeShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    @s_supportVec = new b2Vec2()
    @m_v1 = new b2Vec2()
    @m_v2 = new b2Vec2()
    @m_coreV1 = new b2Vec2()
    @m_coreV2 = new b2Vec2()
    @m_normal = new b2Vec2()
    @m_direction = new b2Vec2()
    @m_cornerDir1 = new b2Vec2()
    @m_cornerDir2 = new b2Vec2()
    return

  b2EdgeShape::TestPoint = (transform, p) ->
    false

  b2EdgeShape::RayCast = (output, input, transform) ->
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

  b2EdgeShape::ComputeAABB = (aabb, transform) ->
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

  b2EdgeShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = 0
    massData.center.SetV @m_v1
    massData.I = 0
    return

  b2EdgeShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
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

  b2EdgeShape::GetLength = ->
    @m_length

  b2EdgeShape::GetVertex1 = ->
    @m_v1

  b2EdgeShape::GetVertex2 = ->
    @m_v2

  b2EdgeShape::GetCoreVertex1 = ->
    @m_coreV1

  b2EdgeShape::GetCoreVertex2 = ->
    @m_coreV2

  b2EdgeShape::GetNormalVector = ->
    @m_normal

  b2EdgeShape::GetDirectionVector = ->
    @m_direction

  b2EdgeShape::GetCorner1Vector = ->
    @m_cornerDir1

  b2EdgeShape::GetCorner2Vector = ->
    @m_cornerDir2

  b2EdgeShape::Corner1IsConvex = ->
    @m_cornerConvex1

  b2EdgeShape::Corner2IsConvex = ->
    @m_cornerConvex2

  b2EdgeShape::GetFirstVertex = (xf) ->
    tMat = xf.R
    new b2Vec2(xf.position.x + (tMat.col1.x * @m_coreV1.x + tMat.col2.x * @m_coreV1.y), xf.position.y + (tMat.col1.y * @m_coreV1.x + tMat.col2.y * @m_coreV1.y))

  b2EdgeShape::GetNextEdge = ->
    @m_nextEdge

  b2EdgeShape::GetPrevEdge = ->
    @m_prevEdge

  b2EdgeShape::Support = (xf, dX, dY) ->
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

  b2EdgeShape::b2EdgeShape = (v1, v2) ->
    @__super.b2Shape.call this
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

  b2EdgeShape::SetPrevEdge = (edge, core, cornerDir, convex) ->
    @m_prevEdge = edge
    @m_coreV1 = core
    @m_cornerDir1 = cornerDir
    @m_cornerConvex1 = convex
    return

  b2EdgeShape::SetNextEdge = (edge, core, cornerDir, convex) ->
    @m_nextEdge = edge
    @m_coreV2 = core
    @m_cornerDir2 = cornerDir
    @m_cornerConvex2 = convex
    return

  b2MassData.b2MassData = ->
    @mass = 0.0
    @center = new b2Vec2(0, 0)
    @I = 0.0
    return

  Box2D.inherit b2PolygonShape, Box2D.Collision.Shapes.b2Shape
  b2PolygonShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2PolygonShape.b2PolygonShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    return

  b2PolygonShape::Copy = ->
    s = new b2PolygonShape()
    s.Set this
    s

  b2PolygonShape::Set = (other) ->
    @__super.Set.call this, other
    if Box2D.is(other, b2PolygonShape)
      other2 = ((if other instanceof b2PolygonShape then other else null))
      @m_centroid.SetV other2.m_centroid
      @m_vertexCount = other2.m_vertexCount
      @Reserve @m_vertexCount
      i = 0

      while i < @m_vertexCount
        @m_vertices[i].SetV other2.m_vertices[i]
        @m_normals[i].SetV other2.m_normals[i]
        i++
    return

  b2PolygonShape::SetAsArray = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    v = new Vector()
    i = 0
    tVec = undefined
    i = 0
    while i < vertices.length
      tVec = vertices[i]
      v.push tVec
      ++i
    @SetAsVector v, vertexCount
    return

  b2PolygonShape.AsArray = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsArray vertices, vertexCount
    polygonShape

  b2PolygonShape::SetAsVector = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    vertexCount = vertices.length  if vertexCount is 0
    b2Settings.b2Assert 2 <= vertexCount
    @m_vertexCount = vertexCount
    @Reserve vertexCount
    i = 0
    i = 0
    while i < @m_vertexCount
      @m_vertices[i].SetV vertices[i]
      i++
    i = 0
    while i < @m_vertexCount
      i1 = parseInt(i)
      i2 = parseInt((if i + 1 < @m_vertexCount then i + 1 else 0))
      edge = b2Math.SubtractVV(@m_vertices[i2], @m_vertices[i1])
      b2Settings.b2Assert edge.LengthSquared() > Number.MIN_VALUE
      @m_normals[i].SetV b2Math.CrossVF(edge, 1.0)
      @m_normals[i].Normalize()
      ++i
    @m_centroid = b2PolygonShape.ComputeCentroid(@m_vertices, @m_vertexCount)
    return

  b2PolygonShape.AsVector = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsVector vertices, vertexCount
    polygonShape

  b2PolygonShape::SetAsBox = (hx, hy) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    @m_vertexCount = 4
    @Reserve 4
    @m_vertices[0].Set (-hx), (-hy)
    @m_vertices[1].Set hx, (-hy)
    @m_vertices[2].Set hx, hy
    @m_vertices[3].Set (-hx), hy
    @m_normals[0].Set 0.0, (-1.0)
    @m_normals[1].Set 1.0, 0.0
    @m_normals[2].Set 0.0, 1.0
    @m_normals[3].Set (-1.0), 0.0
    @m_centroid.SetZero()
    return

  b2PolygonShape.AsBox = (hx, hy) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsBox hx, hy
    polygonShape

  b2PolygonShape::SetAsOrientedBox = (hx, hy, center, angle) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    center = null  if center is `undefined`
    angle = 0.0  if angle is `undefined`
    @m_vertexCount = 4
    @Reserve 4
    @m_vertices[0].Set (-hx), (-hy)
    @m_vertices[1].Set hx, (-hy)
    @m_vertices[2].Set hx, hy
    @m_vertices[3].Set (-hx), hy
    @m_normals[0].Set 0.0, (-1.0)
    @m_normals[1].Set 1.0, 0.0
    @m_normals[2].Set 0.0, 1.0
    @m_normals[3].Set (-1.0), 0.0
    @m_centroid = center
    xf = new b2Transform()
    xf.position = center
    xf.R.Set angle
    i = 0

    while i < @m_vertexCount
      @m_vertices[i] = b2Math.MulX(xf, @m_vertices[i])
      @m_normals[i] = b2Math.MulMV(xf.R, @m_normals[i])
      ++i
    return

  b2PolygonShape.AsOrientedBox = (hx, hy, center, angle) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    center = null  if center is `undefined`
    angle = 0.0  if angle is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsOrientedBox hx, hy, center, angle
    polygonShape

  b2PolygonShape::SetAsEdge = (v1, v2) ->
    @m_vertexCount = 2
    @Reserve 2
    @m_vertices[0].SetV v1
    @m_vertices[1].SetV v2
    @m_centroid.x = 0.5 * (v1.x + v2.x)
    @m_centroid.y = 0.5 * (v1.y + v2.y)
    @m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0)
    @m_normals[0].Normalize()
    @m_normals[1].x = (-@m_normals[0].x)
    @m_normals[1].y = (-@m_normals[0].y)
    return

  b2PolygonShape.AsEdge = (v1, v2) ->
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsEdge v1, v2
    polygonShape

  b2PolygonShape::TestPoint = (xf, p) ->
    tVec = undefined
    tMat = xf.R
    tX = p.x - xf.position.x
    tY = p.y - xf.position.y
    pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y)
    pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y)
    i = 0

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      tX = pLocalX - tVec.x
      tY = pLocalY - tVec.y
      tVec = @m_normals[i]
      dot = (tVec.x * tX + tVec.y * tY)
      return false  if dot > 0.0
      ++i
    true

  b2PolygonShape::RayCast = (output, input, transform) ->
    lower = 0.0
    upper = input.maxFraction
    tX = 0
    tY = 0
    tMat = undefined
    tVec = undefined
    tX = input.p1.x - transform.position.x
    tY = input.p1.y - transform.position.y
    tMat = transform.R
    p1X = (tX * tMat.col1.x + tY * tMat.col1.y)
    p1Y = (tX * tMat.col2.x + tY * tMat.col2.y)
    tX = input.p2.x - transform.position.x
    tY = input.p2.y - transform.position.y
    tMat = transform.R
    p2X = (tX * tMat.col1.x + tY * tMat.col1.y)
    p2Y = (tX * tMat.col2.x + tY * tMat.col2.y)
    dX = p2X - p1X
    dY = p2Y - p1Y
    index = parseInt((-1))
    i = 0

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      tX = tVec.x - p1X
      tY = tVec.y - p1Y
      tVec = @m_normals[i]
      numerator = (tVec.x * tX + tVec.y * tY)
      denominator = (tVec.x * dX + tVec.y * dY)
      if denominator is 0.0
        return false  if numerator < 0.0
      else
        if denominator < 0.0 and numerator < lower * denominator
          lower = numerator / denominator
          index = i
        else upper = numerator / denominator  if denominator > 0.0 and numerator < upper * denominator
      return false  if upper < lower - Number.MIN_VALUE
      ++i
    if index >= 0
      output.fraction = lower
      tMat = transform.R
      tVec = @m_normals[index]
      output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      return true
    false

  b2PolygonShape::ComputeAABB = (aabb, xf) ->
    tMat = xf.R
    tVec = @m_vertices[0]
    lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    upperX = lowerX
    upperY = lowerY
    i = 1

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      lowerX = (if lowerX < vX then lowerX else vX)
      lowerY = (if lowerY < vY then lowerY else vY)
      upperX = (if upperX > vX then upperX else vX)
      upperY = (if upperY > vY then upperY else vY)
      ++i
    aabb.lowerBound.x = lowerX - @m_radius
    aabb.lowerBound.y = lowerY - @m_radius
    aabb.upperBound.x = upperX + @m_radius
    aabb.upperBound.y = upperY + @m_radius
    return

  b2PolygonShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    if @m_vertexCount is 2
      massData.center.x = 0.5 * (@m_vertices[0].x + @m_vertices[1].x)
      massData.center.y = 0.5 * (@m_vertices[0].y + @m_vertices[1].y)
      massData.mass = 0.0
      massData.I = 0.0
      return
    centerX = 0.0
    centerY = 0.0
    area = 0.0
    I = 0.0
    p1X = 0.0
    p1Y = 0.0
    k_inv3 = 1.0 / 3.0
    i = 0

    while i < @m_vertexCount
      p2 = @m_vertices[i]
      p3 = (if i + 1 < @m_vertexCount then @m_vertices[parseInt(i + 1)] else @m_vertices[0])
      e1X = p2.x - p1X
      e1Y = p2.y - p1Y
      e2X = p3.x - p1X
      e2Y = p3.y - p1Y
      D = e1X * e2Y - e1Y * e2X
      triangleArea = 0.5 * D
      area += triangleArea
      centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x)
      centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y)
      px = p1X
      py = p1Y
      ex1 = e1X
      ey1 = e1Y
      ex2 = e2X
      ey2 = e2Y
      intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px
      inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py
      I += D * (intx2 + inty2)
      ++i
    massData.mass = density * area
    centerX *= 1.0 / area
    centerY *= 1.0 / area
    massData.center.Set centerX, centerY
    massData.I = density * I
    return

  b2PolygonShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    normalL = b2Math.MulTMV(xf.R, normal)
    offsetL = offset - b2Math.Dot(normal, xf.position)
    depths = new Vector_a2j_Number()
    diveCount = 0
    intoIndex = parseInt((-1))
    outoIndex = parseInt((-1))
    lastSubmerged = false
    i = 0
    i = 0
    while i < @m_vertexCount
      depths[i] = b2Math.Dot(normalL, @m_vertices[i]) - offsetL
      isSubmerged = depths[i] < (-Number.MIN_VALUE)
      if i > 0
        if isSubmerged
          unless lastSubmerged
            intoIndex = i - 1
            diveCount++
        else
          if lastSubmerged
            outoIndex = i - 1
            diveCount++
      lastSubmerged = isSubmerged
      ++i
    switch diveCount
      when 0
        if lastSubmerged
          md = new b2MassData()
          @ComputeMass md, 1
          c.SetV b2Math.MulX(xf, md.center)
          return md.mass
        else
          return 0
      when 1
        if intoIndex is (-1)
          intoIndex = @m_vertexCount - 1
        else
          outoIndex = @m_vertexCount - 1
    intoIndex2 = parseInt((intoIndex + 1) % @m_vertexCount)
    outoIndex2 = parseInt((outoIndex + 1) % @m_vertexCount)
    intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex])
    outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex])
    intoVec = new b2Vec2(@m_vertices[intoIndex].x * (1 - intoLamdda) + @m_vertices[intoIndex2].x * intoLamdda, @m_vertices[intoIndex].y * (1 - intoLamdda) + @m_vertices[intoIndex2].y * intoLamdda)
    outoVec = new b2Vec2(@m_vertices[outoIndex].x * (1 - outoLamdda) + @m_vertices[outoIndex2].x * outoLamdda, @m_vertices[outoIndex].y * (1 - outoLamdda) + @m_vertices[outoIndex2].y * outoLamdda)
    area = 0
    center = new b2Vec2()
    p2 = @m_vertices[intoIndex2]
    p3 = undefined
    i = intoIndex2
    until i is outoIndex2
      i = (i + 1) % @m_vertexCount
      if i is outoIndex2
        p3 = outoVec
      else
        p3 = @m_vertices[i]
      triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x))
      area += triangleArea
      center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3
      center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3
      p2 = p3
    center.Multiply 1 / area
    c.SetV b2Math.MulX(xf, center)
    area

  b2PolygonShape::GetVertexCount = ->
    @m_vertexCount

  b2PolygonShape::GetVertices = ->
    @m_vertices

  b2PolygonShape::GetNormals = ->
    @m_normals

  b2PolygonShape::GetSupport = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_vertexCount
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    bestIndex

  b2PolygonShape::GetSupportVertex = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_vertexCount
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    @m_vertices[bestIndex]

  b2PolygonShape::Validate = ->
    false

  b2PolygonShape::b2PolygonShape = ->
    @__super.b2Shape.call this
    @m_type = b2Shape.e_polygonShape
    @m_centroid = new b2Vec2()
    @m_vertices = new Vector()
    @m_normals = new Vector()
    return

  b2PolygonShape::Reserve = (count) ->
    count = 0  if count is `undefined`
    i = parseInt(@m_vertices.length)

    while i < count
      @m_vertices[i] = new b2Vec2()
      @m_normals[i] = new b2Vec2()
      i++
    return

  b2PolygonShape.ComputeCentroid = (vs, count) ->
    count = 0  if count is `undefined`
    c = new b2Vec2()
    area = 0.0
    p1X = 0.0
    p1Y = 0.0
    inv3 = 1.0 / 3.0
    i = 0

    while i < count
      p2 = vs[i]
      p3 = (if i + 1 < count then vs[parseInt(i + 1)] else vs[0])
      e1X = p2.x - p1X
      e1Y = p2.y - p1Y
      e2X = p3.x - p1X
      e2Y = p3.y - p1Y
      D = (e1X * e2Y - e1Y * e2X)
      triangleArea = 0.5 * D
      area += triangleArea
      c.x += triangleArea * inv3 * (p1X + p2.x + p3.x)
      c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y)
      ++i
    c.x *= 1.0 / area
    c.y *= 1.0 / area
    c

  b2PolygonShape.ComputeOBB = (obb, vs, count) ->
    count = 0  if count is `undefined`
    i = 0
    p = new Vector(count + 1)
    i = 0
    while i < count
      p[i] = vs[i]
      ++i
    p[count] = p[0]
    minArea = Number.MAX_VALUE
    i = 1
    while i <= count
      root = p[parseInt(i - 1)]
      uxX = p[i].x - root.x
      uxY = p[i].y - root.y
      length = Math.sqrt(uxX * uxX + uxY * uxY)
      uxX /= length
      uxY /= length
      uyX = (-uxY)
      uyY = uxX
      lowerX = Number.MAX_VALUE
      lowerY = Number.MAX_VALUE
      upperX = (-Number.MAX_VALUE)
      upperY = (-Number.MAX_VALUE)
      j = 0

      while j < count
        dX = p[j].x - root.x
        dY = p[j].y - root.y
        rX = (uxX * dX + uxY * dY)
        rY = (uyX * dX + uyY * dY)
        lowerX = rX  if rX < lowerX
        lowerY = rY  if rY < lowerY
        upperX = rX  if rX > upperX
        upperY = rY  if rY > upperY
        ++j
      area = (upperX - lowerX) * (upperY - lowerY)
      if area < 0.95 * minArea
        minArea = area
        obb.R.col1.x = uxX
        obb.R.col1.y = uxY
        obb.R.col2.x = uyX
        obb.R.col2.y = uyY
        centerX = 0.5 * (lowerX + upperX)
        centerY = 0.5 * (lowerY + upperY)
        tMat = obb.R
        obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY)
        obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY)
        obb.extents.x = 0.5 * (upperX - lowerX)
        obb.extents.y = 0.5 * (upperY - lowerY)
      ++i
    return

  Box2D.postDefs.push ->
    Box2D.Collision.Shapes.b2PolygonShape.s_mat = new b2Mat22()
    return

  b2Shape.b2Shape = ->

  b2Shape::Copy = ->
    null

  b2Shape::Set = (other) ->
    @m_radius = other.m_radius
    return

  b2Shape::GetType = ->
    @m_type

  b2Shape::TestPoint = (xf, p) ->
    false

  b2Shape::RayCast = (output, input, transform) ->
    false

  b2Shape::ComputeAABB = (aabb, xf) ->

  b2Shape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    return

  b2Shape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    0

  b2Shape.TestOverlap = (shape1, transform1, shape2, transform2) ->
    input = new b2DistanceInput()
    input.proxyA = new b2DistanceProxy()
    input.proxyA.Set shape1
    input.proxyB = new b2DistanceProxy()
    input.proxyB.Set shape2
    input.transformA = transform1
    input.transformB = transform2
    input.useRadii = true
    simplexCache = new b2SimplexCache()
    simplexCache.count = 0
    output = new b2DistanceOutput()
    b2Distance.Distance output, simplexCache, input
    output.distance < 10.0 * Number.MIN_VALUE

  b2Shape::b2Shape = ->
    @m_type = b2Shape.e_unknownShape
    @m_radius = b2Settings.b2_linearSlop
    return

  Box2D.postDefs.push ->
    Box2D.Collision.Shapes.b2Shape.e_unknownShape = parseInt((-1))
    Box2D.Collision.Shapes.b2Shape.e_circleShape = 0
    Box2D.Collision.Shapes.b2Shape.e_polygonShape = 1
    Box2D.Collision.Shapes.b2Shape.e_edgeShape = 2
    Box2D.Collision.Shapes.b2Shape.e_shapeTypeCount = 3
    Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1
    Box2D.Collision.Shapes.b2Shape.e_missCollide = 0
    Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = parseInt((-1))
    return

  return
)()
