(->
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
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
  b2AABB.b2AABB = ->
    @lowerBound = new b2Vec2()
    @upperBound = new b2Vec2()
    return

  b2AABB::IsValid = ->
    dX = @upperBound.x - @lowerBound.x
    dY = @upperBound.y - @lowerBound.y
    valid = dX >= 0.0 and dY >= 0.0
    valid = valid and @lowerBound.IsValid() and @upperBound.IsValid()
    valid

  b2AABB::GetCenter = ->
    new b2Vec2((@lowerBound.x + @upperBound.x) / 2, (@lowerBound.y + @upperBound.y) / 2)

  b2AABB::GetExtents = ->
    new b2Vec2((@upperBound.x - @lowerBound.x) / 2, (@upperBound.y - @lowerBound.y) / 2)

  b2AABB::Contains = (aabb) ->
    result = true
    result = result and @lowerBound.x <= aabb.lowerBound.x
    result = result and @lowerBound.y <= aabb.lowerBound.y
    result = result and aabb.upperBound.x <= @upperBound.x
    result = result and aabb.upperBound.y <= @upperBound.y
    result

  b2AABB::RayCast = (output, input) ->
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

  b2AABB::TestOverlap = (other) ->
    d1X = other.lowerBound.x - @upperBound.x
    d1Y = other.lowerBound.y - @upperBound.y
    d2X = @lowerBound.x - other.upperBound.x
    d2Y = @lowerBound.y - other.upperBound.y
    return false  if d1X > 0.0 or d1Y > 0.0
    return false  if d2X > 0.0 or d2Y > 0.0
    true

  b2AABB.Combine = (aabb1, aabb2) ->
    aabb = new b2AABB()
    aabb.Combine aabb1, aabb2
    aabb

  b2AABB::Combine = (aabb1, aabb2) ->
    @lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x)
    @lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y)
    @upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x)
    @upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y)
    return

  b2Bound.b2Bound = ->

  b2Bound::IsLower = ->
    (@value & 1) is 0

  b2Bound::IsUpper = ->
    (@value & 1) is 1

  b2Bound::Swap = (b) ->
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

  b2BoundValues.b2BoundValues = ->

  b2BoundValues::b2BoundValues = ->
    @lowerValues = new Vector_a2j_Number()
    @lowerValues[0] = 0.0
    @lowerValues[1] = 0.0
    @upperValues = new Vector_a2j_Number()
    @upperValues[0] = 0.0
    @upperValues[1] = 0.0
    return

  b2Collision.b2Collision = ->

  b2Collision.ClipSegmentToLine = (vOut, vIn, normal, offset) ->
    offset = 0  if offset is `undefined`
    cv = undefined
    numOut = 0
    cv = vIn[0]
    vIn0 = cv.v
    cv = vIn[1]
    vIn1 = cv.v
    distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset
    distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset
    vOut[numOut++].Set vIn[0]  if distance0 <= 0.0
    vOut[numOut++].Set vIn[1]  if distance1 <= 0.0
    if distance0 * distance1 < 0.0
      interp = distance0 / (distance0 - distance1)
      cv = vOut[numOut]
      tVec = cv.v
      tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x)
      tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y)
      cv = vOut[numOut]
      cv2 = undefined
      if distance0 > 0.0
        cv2 = vIn[0]
        cv.id = cv2.id
      else
        cv2 = vIn[1]
        cv.id = cv2.id
      ++numOut
    numOut

  b2Collision.EdgeSeparation = (poly1, xf1, edge1, poly2, xf2) ->
    edge1 = 0  if edge1 is `undefined`
    count1 = parseInt(poly1.m_vertexCount)
    vertices1 = poly1.m_vertices
    normals1 = poly1.m_normals
    count2 = parseInt(poly2.m_vertexCount)
    vertices2 = poly2.m_vertices
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = normals1[edge1]
    normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY)
    normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY)
    index = 0
    minDot = Number.MAX_VALUE
    i = 0

    while i < count2
      tVec = vertices2[i]
      dot = tVec.x * normal1X + tVec.y * normal1Y
      if dot < minDot
        minDot = dot
        index = i
      ++i
    tVec = vertices1[edge1]
    tMat = xf1.R
    v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tVec = vertices2[index]
    tMat = xf2.R
    v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    v2X -= v1X
    v2Y -= v1Y
    separation = v2X * normal1WorldX + v2Y * normal1WorldY
    separation

  b2Collision.FindMaxSeparation = (edgeIndex, poly1, xf1, poly2, xf2) ->
    count1 = parseInt(poly1.m_vertexCount)
    normals1 = poly1.m_normals
    tVec = undefined
    tMat = undefined
    tMat = xf2.R
    tVec = poly2.m_centroid
    dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf1.R
    tVec = poly1.m_centroid
    dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y)
    dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y)
    edge = 0
    maxDot = (-Number.MAX_VALUE)
    i = 0

    while i < count1
      tVec = normals1[i]
      dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y)
      if dot > maxDot
        maxDot = dot
        edge = i
      ++i
    s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2)
    prevEdge = parseInt((if edge - 1 >= 0 then edge - 1 else count1 - 1))
    sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2)
    nextEdge = parseInt((if edge + 1 < count1 then edge + 1 else 0))
    sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2)
    bestEdge = 0
    bestSeparation = 0
    increment = 0
    if sPrev > s and sPrev > sNext
      increment = (-1)
      bestEdge = prevEdge
      bestSeparation = sPrev
    else if sNext > s
      increment = 1
      bestEdge = nextEdge
      bestSeparation = sNext
    else
      edgeIndex[0] = edge
      return s
    loop
      if increment is (-1)
        edge = (if bestEdge - 1 >= 0 then bestEdge - 1 else count1 - 1)
      else
        edge = (if bestEdge + 1 < count1 then bestEdge + 1 else 0)
      s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2)
      if s > bestSeparation
        bestEdge = edge
        bestSeparation = s
      else
        break
    edgeIndex[0] = bestEdge
    bestSeparation

  b2Collision.FindIncidentEdge = (c, poly1, xf1, edge1, poly2, xf2) ->
    edge1 = 0  if edge1 is `undefined`
    count1 = parseInt(poly1.m_vertexCount)
    normals1 = poly1.m_normals
    count2 = parseInt(poly2.m_vertexCount)
    vertices2 = poly2.m_vertices
    normals2 = poly2.m_normals
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = normals1[edge1]
    normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y)
    normal1Y = (tMat.col2.x * normal1X + tMat.col2.y * normal1Y)
    normal1X = tX
    index = 0
    minDot = Number.MAX_VALUE
    i = 0

    while i < count2
      tVec = normals2[i]
      dot = (normal1X * tVec.x + normal1Y * tVec.y)
      if dot < minDot
        minDot = dot
        index = i
      ++i
    tClip = undefined
    i1 = parseInt(index)
    i2 = parseInt((if i1 + 1 < count2 then i1 + 1 else 0))
    tClip = c[0]
    tVec = vertices2[i1]
    tMat = xf2.R
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tClip.id.features.referenceEdge = edge1
    tClip.id.features.incidentEdge = i1
    tClip.id.features.incidentVertex = 0
    tClip = c[1]
    tVec = vertices2[i2]
    tMat = xf2.R
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tClip.id.features.referenceEdge = edge1
    tClip.id.features.incidentEdge = i2
    tClip.id.features.incidentVertex = 1
    return

  b2Collision.MakeClipPointVector = ->
    r = new Vector(2)
    r[0] = new ClipVertex()
    r[1] = new ClipVertex()
    r

  b2Collision.CollidePolygons = (manifold, polyA, xfA, polyB, xfB) ->
    cv = undefined
    manifold.m_pointCount = 0
    totalRadius = polyA.m_radius + polyB.m_radius
    edgeA = 0
    b2Collision.s_edgeAO[0] = edgeA
    separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB)
    edgeA = b2Collision.s_edgeAO[0]
    return  if separationA > totalRadius
    edgeB = 0
    b2Collision.s_edgeBO[0] = edgeB
    separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA)
    edgeB = b2Collision.s_edgeBO[0]
    return  if separationB > totalRadius
    poly1 = undefined
    poly2 = undefined
    xf1 = undefined
    xf2 = undefined
    edge1 = 0
    flip = 0
    k_relativeTol = 0.98
    k_absoluteTol = 0.001
    tMat = undefined
    if separationB > k_relativeTol * separationA + k_absoluteTol
      poly1 = polyB
      poly2 = polyA
      xf1 = xfB
      xf2 = xfA
      edge1 = edgeB
      manifold.m_type = b2Manifold.e_faceB
      flip = 1
    else
      poly1 = polyA
      poly2 = polyB
      xf1 = xfA
      xf2 = xfB
      edge1 = edgeA
      manifold.m_type = b2Manifold.e_faceA
      flip = 0
    incidentEdge = b2Collision.s_incidentEdge
    b2Collision.FindIncidentEdge incidentEdge, poly1, xf1, edge1, poly2, xf2
    count1 = parseInt(poly1.m_vertexCount)
    vertices1 = poly1.m_vertices
    local_v11 = vertices1[edge1]
    local_v12 = undefined
    if edge1 + 1 < count1
      local_v12 = vertices1[parseInt(edge1 + 1)]
    else
      local_v12 = vertices1[0]
    localTangent = b2Collision.s_localTangent
    localTangent.Set local_v12.x - local_v11.x, local_v12.y - local_v11.y
    localTangent.Normalize()
    localNormal = b2Collision.s_localNormal
    localNormal.x = localTangent.y
    localNormal.y = (-localTangent.x)
    planePoint = b2Collision.s_planePoint
    planePoint.Set 0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y)
    tangent = b2Collision.s_tangent
    tMat = xf1.R
    tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y)
    tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y)
    tangent2 = b2Collision.s_tangent2
    tangent2.x = (-tangent.x)
    tangent2.y = (-tangent.y)
    normal = b2Collision.s_normal
    normal.x = tangent.y
    normal.y = (-tangent.x)
    v11 = b2Collision.s_v11
    v12 = b2Collision.s_v12
    v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y)
    v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y)
    v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y)
    v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y)
    frontOffset = normal.x * v11.x + normal.y * v11.y
    sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius
    sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius
    clipPoints1 = b2Collision.s_clipPoints1
    clipPoints2 = b2Collision.s_clipPoints2
    np = 0
    np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1)
    return  if np < 2
    np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2)
    return  if np < 2
    manifold.m_localPlaneNormal.SetV localNormal
    manifold.m_localPoint.SetV planePoint
    pointCount = 0
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      cv = clipPoints2[i]
      separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset
      if separation <= totalRadius
        cp = manifold.m_points[pointCount]
        tMat = xf2.R
        tX = cv.v.x - xf2.position.x
        tY = cv.v.y - xf2.position.y
        cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y)
        cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y)
        cp.m_id.Set cv.id
        cp.m_id.features.flip = flip
        ++pointCount
      ++i
    manifold.m_pointCount = pointCount
    return

  b2Collision.CollideCircles = (manifold, circle1, xf1, circle2, xf2) ->
    manifold.m_pointCount = 0
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = circle1.m_p
    p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    tVec = circle2.m_p
    p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dX = p2X - p1X
    dY = p2Y - p1Y
    distSqr = dX * dX + dY * dY
    radius = circle1.m_radius + circle2.m_radius
    return  if distSqr > radius * radius
    manifold.m_type = b2Manifold.e_circles
    manifold.m_localPoint.SetV circle1.m_p
    manifold.m_localPlaneNormal.SetZero()
    manifold.m_pointCount = 1
    manifold.m_points[0].m_localPoint.SetV circle2.m_p
    manifold.m_points[0].m_id.key = 0
    return

  b2Collision.CollidePolygonAndCircle = (manifold, polygon, xf1, circle, xf2) ->
    manifold.m_pointCount = 0
    tPoint = undefined
    dX = 0
    dY = 0
    positionX = 0
    positionY = 0
    tVec = undefined
    tMat = undefined
    tMat = xf2.R
    tVec = circle.m_p
    cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dX = cX - xf1.position.x
    dY = cY - xf1.position.y
    tMat = xf1.R
    cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y)
    cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y)
    dist = 0
    normalIndex = 0
    separation = (-Number.MAX_VALUE)
    radius = polygon.m_radius + circle.m_radius
    vertexCount = parseInt(polygon.m_vertexCount)
    vertices = polygon.m_vertices
    normals = polygon.m_normals
    i = 0

    while i < vertexCount
      tVec = vertices[i]
      dX = cLocalX - tVec.x
      dY = cLocalY - tVec.y
      tVec = normals[i]
      s = tVec.x * dX + tVec.y * dY
      return  if s > radius
      if s > separation
        separation = s
        normalIndex = i
      ++i
    vertIndex1 = parseInt(normalIndex)
    vertIndex2 = parseInt((if vertIndex1 + 1 < vertexCount then vertIndex1 + 1 else 0))
    v1 = vertices[vertIndex1]
    v2 = vertices[vertIndex2]
    if separation < Number.MIN_VALUE
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.SetV normals[normalIndex]
      manifold.m_localPoint.x = 0.5 * (v1.x + v2.x)
      manifold.m_localPoint.y = 0.5 * (v1.y + v2.y)
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
      return
    u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y)
    u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y)
    if u1 <= 0.0
      return  if (cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = cLocalX - v1.x
      manifold.m_localPlaneNormal.y = cLocalY - v1.y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.SetV v1
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    else if u2 <= 0
      return  if (cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = cLocalX - v2.x
      manifold.m_localPlaneNormal.y = cLocalY - v2.y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.SetV v2
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    else
      faceCenterX = 0.5 * (v1.x + v2.x)
      faceCenterY = 0.5 * (v1.y + v2.y)
      separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y
      return  if separation > radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = normals[vertIndex1].x
      manifold.m_localPlaneNormal.y = normals[vertIndex1].y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.Set faceCenterX, faceCenterY
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    return

  b2Collision.TestOverlap = (a, b) ->
    t1 = b.lowerBound
    t2 = a.upperBound
    d1X = t1.x - t2.x
    d1Y = t1.y - t2.y
    t1 = a.lowerBound
    t2 = b.upperBound
    d2X = t1.x - t2.x
    d2Y = t1.y - t2.y
    return false  if d1X > 0.0 or d1Y > 0.0
    return false  if d2X > 0.0 or d2Y > 0.0
    true

  Box2D.postDefs.push ->
    Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1)
    Box2D.Collision.b2Collision.s_edgeBO = new Vector_a2j_Number(1)
    Box2D.Collision.b2Collision.s_localTangent = new b2Vec2()
    Box2D.Collision.b2Collision.s_localNormal = new b2Vec2()
    Box2D.Collision.b2Collision.s_planePoint = new b2Vec2()
    Box2D.Collision.b2Collision.s_normal = new b2Vec2()
    Box2D.Collision.b2Collision.s_tangent = new b2Vec2()
    Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2()
    Box2D.Collision.b2Collision.s_v11 = new b2Vec2()
    Box2D.Collision.b2Collision.s_v12 = new b2Vec2()
    Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2()
    Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff
    return

  b2ContactID.b2ContactID = ->
    @features = new Features()
    return

  b2ContactID::b2ContactID = ->
    @features._m_id = this
    return

  b2ContactID::Set = (id) ->
    @key = id._key
    return

  b2ContactID::Copy = ->
    id = new b2ContactID()
    id.key = @key
    id

  Object.defineProperty b2ContactID::, "key",
    enumerable: false
    configurable: true
    get: ->
      @_key

  Object.defineProperty b2ContactID::, "key",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_key = value
      @features._referenceEdge = @_key & 0x000000ff
      @features._incidentEdge = ((@_key & 0x0000ff00) >> 8) & 0x000000ff
      @features._incidentVertex = ((@_key & 0x00ff0000) >> 16) & 0x000000ff
      @features._flip = ((@_key & 0xff000000) >> 24) & 0x000000ff
      return

  b2ContactPoint.b2ContactPoint = ->
    @position = new b2Vec2()
    @velocity = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return

  b2Distance.b2Distance = ->

  b2Distance.Distance = (output, cache, input) ->
    ++b2Distance.b2_gjkCalls
    proxyA = input.proxyA
    proxyB = input.proxyB
    transformA = input.transformA
    transformB = input.transformB
    simplex = b2Distance.s_simplex
    simplex.ReadCache cache, proxyA, transformA, proxyB, transformB
    vertices = simplex.m_vertices
    k_maxIters = 20
    saveA = b2Distance.s_saveA
    saveB = b2Distance.s_saveB
    saveCount = 0
    closestPoint = simplex.GetClosestPoint()
    distanceSqr1 = closestPoint.LengthSquared()
    distanceSqr2 = distanceSqr1
    i = 0
    p = undefined
    iter = 0
    while iter < k_maxIters
      saveCount = simplex.m_count
      i = 0
      while i < saveCount
        saveA[i] = vertices[i].indexA
        saveB[i] = vertices[i].indexB
        i++
      switch simplex.m_count
        when 1, 2
          simplex.Solve2()
        when 3
          simplex.Solve3()
        else
          b2Settings.b2Assert false
      break  if simplex.m_count is 3
      p = simplex.GetClosestPoint()
      distanceSqr2 = p.LengthSquared()
      
      ###*
      todo: this looks like a bug
      ###
      
      #if (distanceSqr2 > distanceSqr1) {}
      distanceSqr1 = distanceSqr2
      d = simplex.GetSearchDirection()
      break  if d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE
      vertex = vertices[simplex.m_count]
      vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()))
      vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA))
      vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d))
      vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB))
      vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA)
      ++iter
      ++b2Distance.b2_gjkIters
      duplicate = false
      i = 0
      while i < saveCount
        if vertex.indexA is saveA[i] and vertex.indexB is saveB[i]
          duplicate = true
          break
        i++
      break  if duplicate
      ++simplex.m_count
    b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter)
    simplex.GetWitnessPoints output.pointA, output.pointB
    output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length()
    output.iterations = iter
    simplex.WriteCache cache
    if input.useRadii
      rA = proxyA.m_radius
      rB = proxyB.m_radius
      if output.distance > rA + rB and output.distance > Number.MIN_VALUE
        output.distance -= rA + rB
        normal = b2Math.SubtractVV(output.pointB, output.pointA)
        normal.Normalize()
        output.pointA.x += rA * normal.x
        output.pointA.y += rA * normal.y
        output.pointB.x -= rB * normal.x
        output.pointB.y -= rB * normal.y
      else
        p = new b2Vec2()
        p.x = .5 * (output.pointA.x + output.pointB.x)
        p.y = .5 * (output.pointA.y + output.pointB.y)
        output.pointA.x = output.pointB.x = p.x
        output.pointA.y = output.pointB.y = p.y
        output.distance = 0.0
    return

  Box2D.postDefs.push ->
    Box2D.Collision.b2Distance.s_simplex = new b2Simplex()
    Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3)
    Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3)
    return

  b2DistanceInput.b2DistanceInput = ->

  b2DistanceOutput.b2DistanceOutput = ->
    @pointA = new b2Vec2()
    @pointB = new b2Vec2()
    return

  b2DistanceProxy.b2DistanceProxy = ->

  b2DistanceProxy::Set = (shape) ->
    switch shape.GetType()
      when b2Shape.e_circleShape
        circle = ((if shape instanceof b2CircleShape then shape else null))
        @m_vertices = new Vector(1, true)
        @m_vertices[0] = circle.m_p
        @m_count = 1
        @m_radius = circle.m_radius
      when b2Shape.e_polygonShape
        polygon = ((if shape instanceof b2PolygonShape then shape else null))
        @m_vertices = polygon.m_vertices
        @m_count = polygon.m_vertexCount
        @m_radius = polygon.m_radius
      else
        b2Settings.b2Assert false
    return

  b2DistanceProxy::GetSupport = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    bestIndex

  b2DistanceProxy::GetSupportVertex = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    @m_vertices[bestIndex]

  b2DistanceProxy::GetVertexCount = ->
    @m_count

  b2DistanceProxy::GetVertex = (index) ->
    index = 0  if index is `undefined`
    b2Settings.b2Assert 0 <= index and index < @m_count
    @m_vertices[index]

  b2DynamicTree.b2DynamicTree = ->

  b2DynamicTree::b2DynamicTree = ->
    @m_root = null
    @m_freeList = null
    @m_path = 0
    @m_insertionCount = 0
    return

  b2DynamicTree::CreateProxy = (aabb, userData) ->
    node = @AllocateNode()
    extendX = b2Settings.b2_aabbExtension
    extendY = b2Settings.b2_aabbExtension
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    node.aabb.upperBound.x = aabb.upperBound.x + extendX
    node.aabb.upperBound.y = aabb.upperBound.y + extendY
    node.userData = userData
    @InsertLeaf node
    node

  b2DynamicTree::DestroyProxy = (proxy) ->
    @RemoveLeaf proxy
    @FreeNode proxy
    return

  b2DynamicTree::MoveProxy = (proxy, aabb, displacement) ->
    b2Settings.b2Assert proxy.IsLeaf()
    return false  if proxy.aabb.Contains(aabb)
    @RemoveLeaf proxy
    extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.x > 0 then displacement.x else (-displacement.x)))
    extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.y > 0 then displacement.y else (-displacement.y)))
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY
    @InsertLeaf proxy
    true

  b2DynamicTree::Rebalance = (iterations) ->
    iterations = 0  if iterations is `undefined`
    return  unless @m_root?
    i = 0

    while i < iterations
      node = @m_root
      bit = 0
      while node.IsLeaf() is false
        node = (if (@m_path >> bit) & 1 then node.child2 else node.child1)
        bit = (bit + 1) & 31
      ++@m_path
      @RemoveLeaf node
      @InsertLeaf node
      i++
    return

  b2DynamicTree::GetFatAABB = (proxy) ->
    proxy.aabb

  b2DynamicTree::GetUserData = (proxy) ->
    proxy.userData

  b2DynamicTree::Query = (callback, aabb) ->
    return  unless @m_root?
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      if node.aabb.TestOverlap(aabb)
        if node.IsLeaf()
          proceed = callback(node)
          return  unless proceed
        else
          stack[count++] = node.child1
          stack[count++] = node.child2
    return

  b2DynamicTree::RayCast = (callback, input) ->
    return  unless @m_root?
    p1 = input.p1
    p2 = input.p2
    r = b2Math.SubtractVV(p1, p2)
    r.Normalize()
    v = b2Math.CrossFV(1.0, r)
    abs_v = b2Math.AbsV(v)
    maxFraction = input.maxFraction
    segmentAABB = new b2AABB()
    tX = 0
    tY = 0
    tX = p1.x + maxFraction * (p2.x - p1.x)
    tY = p1.y + maxFraction * (p2.y - p1.y)
    segmentAABB.lowerBound.x = Math.min(p1.x, tX)
    segmentAABB.lowerBound.y = Math.min(p1.y, tY)
    segmentAABB.upperBound.x = Math.max(p1.x, tX)
    segmentAABB.upperBound.y = Math.max(p1.y, tY)
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      continue  if node.aabb.TestOverlap(segmentAABB) is false
      c = node.aabb.GetCenter()
      h = node.aabb.GetExtents()
      separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y
      continue  if separation > 0.0
      if node.IsLeaf()
        subInput = new b2RayCastInput()
        subInput.p1 = input.p1
        subInput.p2 = input.p2
        subInput.maxFraction = input.maxFraction
        maxFraction = callback(subInput, node)
        return  if maxFraction is 0.0
        if maxFraction > 0.0
          tX = p1.x + maxFraction * (p2.x - p1.x)
          tY = p1.y + maxFraction * (p2.y - p1.y)
          segmentAABB.lowerBound.x = Math.min(p1.x, tX)
          segmentAABB.lowerBound.y = Math.min(p1.y, tY)
          segmentAABB.upperBound.x = Math.max(p1.x, tX)
          segmentAABB.upperBound.y = Math.max(p1.y, tY)
      else
        stack[count++] = node.child1
        stack[count++] = node.child2
    return

  b2DynamicTree::AllocateNode = ->
    if @m_freeList
      node = @m_freeList
      @m_freeList = node.parent
      node.parent = null
      node.child1 = null
      node.child2 = null
      return node
    new b2DynamicTreeNode()

  b2DynamicTree::FreeNode = (node) ->
    node.parent = @m_freeList
    @m_freeList = node
    return

  b2DynamicTree::InsertLeaf = (leaf) ->
    ++@m_insertionCount
    unless @m_root?
      @m_root = leaf
      @m_root.parent = null
      return
    center = leaf.aabb.GetCenter()
    sibling = @m_root
    if sibling.IsLeaf() is false
      loop
        child1 = sibling.child1
        child2 = sibling.child2
        norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y)
        norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y)
        if norm1 < norm2
          sibling = child1
        else
          sibling = child2
        break unless sibling.IsLeaf() is false
    node1 = sibling.parent
    node2 = @AllocateNode()
    node2.parent = node1
    node2.userData = null
    node2.aabb.Combine leaf.aabb, sibling.aabb
    if node1
      if sibling.parent.child1 is sibling
        node1.child1 = node2
      else
        node1.child2 = node2
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      loop
        break  if node1.aabb.Contains(node2.aabb)
        node1.aabb.Combine node1.child1.aabb, node1.child2.aabb
        node2 = node1
        node1 = node1.parent
        break unless node1
    else
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      @m_root = node2
    return

  b2DynamicTree::RemoveLeaf = (leaf) ->
    if leaf is @m_root
      @m_root = null
      return
    node2 = leaf.parent
    node1 = node2.parent
    sibling = undefined
    if node2.child1 is leaf
      sibling = node2.child2
    else
      sibling = node2.child1
    if node1
      if node1.child1 is node2
        node1.child1 = sibling
      else
        node1.child2 = sibling
      sibling.parent = node1
      @FreeNode node2
      while node1
        oldAABB = node1.aabb
        node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb)
        break  if oldAABB.Contains(node1.aabb)
        node1 = node1.parent
    else
      @m_root = sibling
      sibling.parent = null
      @FreeNode node2
    return

  b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase = ->
    @m_tree = new b2DynamicTree()
    @m_moveBuffer = new Vector()
    @m_pairBuffer = new Vector()
    @m_pairCount = 0
    return

  b2DynamicTreeBroadPhase::CreateProxy = (aabb, userData) ->
    proxy = @m_tree.CreateProxy(aabb, userData)
    ++@m_proxyCount
    @BufferMove proxy
    proxy

  b2DynamicTreeBroadPhase::DestroyProxy = (proxy) ->
    @UnBufferMove proxy
    --@m_proxyCount
    @m_tree.DestroyProxy proxy
    return

  b2DynamicTreeBroadPhase::MoveProxy = (proxy, aabb, displacement) ->
    buffer = @m_tree.MoveProxy(proxy, aabb, displacement)
    @BufferMove proxy  if buffer
    return

  b2DynamicTreeBroadPhase::TestOverlap = (proxyA, proxyB) ->
    aabbA = @m_tree.GetFatAABB(proxyA)
    aabbB = @m_tree.GetFatAABB(proxyB)
    aabbA.TestOverlap aabbB

  b2DynamicTreeBroadPhase::GetUserData = (proxy) ->
    @m_tree.GetUserData proxy

  b2DynamicTreeBroadPhase::GetFatAABB = (proxy) ->
    @m_tree.GetFatAABB proxy

  b2DynamicTreeBroadPhase::GetProxyCount = ->
    @m_proxyCount

  b2DynamicTreeBroadPhase::UpdatePairs = (callback) ->
    __this = this
    __this.m_pairCount = 0
    i = 0
    queryProxy = undefined
    i = 0
    while i < __this.m_moveBuffer.length
      QueryCallback = (proxy) ->
        return true  if proxy is queryProxy
        __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair()  if __this.m_pairCount is __this.m_pairBuffer.length
        pair = __this.m_pairBuffer[__this.m_pairCount]
        pair.proxyA = (if proxy < queryProxy then proxy else queryProxy)
        pair.proxyB = (if proxy >= queryProxy then proxy else queryProxy)
        ++__this.m_pairCount
        true
      queryProxy = __this.m_moveBuffer[i]
      fatAABB = __this.m_tree.GetFatAABB(queryProxy)
      __this.m_tree.Query QueryCallback, fatAABB
      ++i
    __this.m_moveBuffer.length = 0
    i = 0

    while i < __this.m_pairCount
      primaryPair = __this.m_pairBuffer[i]
      userDataA = __this.m_tree.GetUserData(primaryPair.proxyA)
      userDataB = __this.m_tree.GetUserData(primaryPair.proxyB)
      callback userDataA, userDataB
      ++i
      while i < __this.m_pairCount
        pair = __this.m_pairBuffer[i]
        break  if pair.proxyA isnt primaryPair.proxyA or pair.proxyB isnt primaryPair.proxyB
        ++i
    return

  b2DynamicTreeBroadPhase::Query = (callback, aabb) ->
    @m_tree.Query callback, aabb
    return

  b2DynamicTreeBroadPhase::RayCast = (callback, input) ->
    @m_tree.RayCast callback, input
    return

  b2DynamicTreeBroadPhase::Validate = ->

  b2DynamicTreeBroadPhase::Rebalance = (iterations) ->
    iterations = 0  if iterations is `undefined`
    @m_tree.Rebalance iterations
    return

  b2DynamicTreeBroadPhase::BufferMove = (proxy) ->
    @m_moveBuffer[@m_moveBuffer.length] = proxy
    return

  b2DynamicTreeBroadPhase::UnBufferMove = (proxy) ->
    i = parseInt(@m_moveBuffer.indexOf(proxy))
    @m_moveBuffer.splice i, 1
    return

  b2DynamicTreeBroadPhase::ComparePairs = (pair1, pair2) ->
    0

  b2DynamicTreeBroadPhase.__implements = {}
  b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true
  b2DynamicTreeNode.b2DynamicTreeNode = ->
    @aabb = new b2AABB()
    return

  b2DynamicTreeNode::IsLeaf = ->
    not @child1?

  b2DynamicTreePair.b2DynamicTreePair = ->

  b2Manifold.b2Manifold = ->
    @m_pointCount = 0
    return

  b2Manifold::b2Manifold = ->
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2ManifoldPoint()
      i++
    @m_localPlaneNormal = new b2Vec2()
    @m_localPoint = new b2Vec2()
    return

  b2Manifold::Reset = ->
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Reset()
      i++
    @m_localPlaneNormal.SetZero()
    @m_localPoint.SetZero()
    @m_type = 0
    @m_pointCount = 0
    return

  b2Manifold::Set = (m) ->
    @m_pointCount = m.m_pointCount
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Set m.m_points[i]
      i++
    @m_localPlaneNormal.SetV m.m_localPlaneNormal
    @m_localPoint.SetV m.m_localPoint
    @m_type = m.m_type
    return

  b2Manifold::Copy = ->
    copy = new b2Manifold()
    copy.Set this
    copy

  Box2D.postDefs.push ->
    Box2D.Collision.b2Manifold.e_circles = 0x0001
    Box2D.Collision.b2Manifold.e_faceA = 0x0002
    Box2D.Collision.b2Manifold.e_faceB = 0x0004
    return

  b2ManifoldPoint.b2ManifoldPoint = ->
    @m_localPoint = new b2Vec2()
    @m_id = new b2ContactID()
    return

  b2ManifoldPoint::b2ManifoldPoint = ->
    @Reset()
    return

  b2ManifoldPoint::Reset = ->
    @m_localPoint.SetZero()
    @m_normalImpulse = 0.0
    @m_tangentImpulse = 0.0
    @m_id.key = 0
    return

  b2ManifoldPoint::Set = (m) ->
    @m_localPoint.SetV m.m_localPoint
    @m_normalImpulse = m.m_normalImpulse
    @m_tangentImpulse = m.m_tangentImpulse
    @m_id.Set m.m_id
    return

  b2Point.b2Point = ->
    @p = new b2Vec2()
    return

  b2Point::Support = (xf, vX, vY) ->
    vX = 0  if vX is `undefined`
    vY = 0  if vY is `undefined`
    @p

  b2Point::GetFirstVertex = (xf) ->
    @p

  b2RayCastInput.b2RayCastInput = ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    return

  b2RayCastInput::b2RayCastInput = (p1, p2, maxFraction) ->
    p1 = null  if p1 is `undefined`
    p2 = null  if p2 is `undefined`
    maxFraction = 1  if maxFraction is `undefined`
    @p1.SetV p1  if p1
    @p2.SetV p2  if p2
    @maxFraction = maxFraction
    return

  b2RayCastOutput.b2RayCastOutput = ->
    @normal = new b2Vec2()
    return

  b2Segment.b2Segment = ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    return

  b2Segment::TestSegment = (lambda, normal, segment, maxLambda) ->
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

  b2Segment::Extend = (aabb) ->
    @ExtendForward aabb
    @ExtendBackward aabb
    return

  b2Segment::ExtendForward = (aabb) ->
    dX = @p2.x - @p1.x
    dY = @p2.y - @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p1.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p1.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p1.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p1.y) / dY else Number.POSITIVE_INFINITY)))
    @p2.x = @p1.x + dX * lambda
    @p2.y = @p1.y + dY * lambda
    return

  b2Segment::ExtendBackward = (aabb) ->
    dX = (-@p2.x) + @p1.x
    dY = (-@p2.y) + @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p2.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p2.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p2.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p2.y) / dY else Number.POSITIVE_INFINITY)))
    @p1.x = @p2.x + dX * lambda
    @p1.y = @p2.y + dY * lambda
    return

  b2SeparationFunction.b2SeparationFunction = ->
    @m_localPoint = new b2Vec2()
    @m_axis = new b2Vec2()
    return

  b2SeparationFunction::Initialize = (cache, proxyA, transformA, proxyB, transformB) ->
    @m_proxyA = proxyA
    @m_proxyB = proxyB
    count = parseInt(cache.count)
    b2Settings.b2Assert 0 < count and count < 3
    localPointA = undefined
    localPointA1 = undefined
    localPointA2 = undefined
    localPointB = undefined
    localPointB1 = undefined
    localPointB2 = undefined
    pointAX = 0
    pointAY = 0
    pointBX = 0
    pointBY = 0
    normalX = 0
    normalY = 0
    tMat = undefined
    tVec = undefined
    s = 0
    sgn = 0
    if count is 1
      @m_type = b2SeparationFunction.e_points
      localPointA = @m_proxyA.GetVertex(cache.indexA[0])
      localPointB = @m_proxyB.GetVertex(cache.indexB[0])
      tVec = localPointA
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointB
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      @m_axis.x = pointBX - pointAX
      @m_axis.y = pointBY - pointAY
      @m_axis.Normalize()
    else if cache.indexB[0] is cache.indexB[1]
      @m_type = b2SeparationFunction.e_faceA
      localPointA1 = @m_proxyA.GetVertex(cache.indexA[0])
      localPointA2 = @m_proxyA.GetVertex(cache.indexA[1])
      localPointB = @m_proxyB.GetVertex(cache.indexB[0])
      @m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x)
      @m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y)
      @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0)
      @m_axis.Normalize()
      tVec = @m_axis
      tMat = transformA.R
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tVec = @m_localPoint
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointB
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY
      @m_axis.NegativeSelf()  if s < 0.0
    else if cache.indexA[0] is cache.indexA[0]
      @m_type = b2SeparationFunction.e_faceB
      localPointB1 = @m_proxyB.GetVertex(cache.indexB[0])
      localPointB2 = @m_proxyB.GetVertex(cache.indexB[1])
      localPointA = @m_proxyA.GetVertex(cache.indexA[0])
      @m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x)
      @m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y)
      @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0)
      @m_axis.Normalize()
      tVec = @m_axis
      tMat = transformB.R
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tVec = @m_localPoint
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointA
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY
      @m_axis.NegativeSelf()  if s < 0.0
    else
      localPointA1 = @m_proxyA.GetVertex(cache.indexA[0])
      localPointA2 = @m_proxyA.GetVertex(cache.indexA[1])
      localPointB1 = @m_proxyB.GetVertex(cache.indexB[0])
      localPointB2 = @m_proxyB.GetVertex(cache.indexB[1])
      pA = b2Math.MulX(transformA, localPointA)
      dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1))
      pB = b2Math.MulX(transformB, localPointB)
      dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1))
      a = dA.x * dA.x + dA.y * dA.y
      e = dB.x * dB.x + dB.y * dB.y
      r = b2Math.SubtractVV(dB, dA)
      c = dA.x * r.x + dA.y * r.y
      f = dB.x * r.x + dB.y * r.y
      b = dA.x * dB.x + dA.y * dB.y
      denom = a * e - b * b
      s = 0.0
      s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0)  unless denom is 0.0
      t = (b * s + f) / e
      if t < 0.0
        t = 0.0
        s = b2Math.Clamp((b - c) / a, 0.0, 1.0)
      localPointA = new b2Vec2()
      localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x)
      localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y)
      localPointB = new b2Vec2()
      localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x)
      localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y)
      if s is 0.0 or s is 1.0
        @m_type = b2SeparationFunction.e_faceB
        @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0)
        @m_axis.Normalize()
        @m_localPoint = localPointB
        tVec = @m_axis
        tMat = transformB.R
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tVec = @m_localPoint
        tMat = transformB.R
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tVec = localPointA
        tMat = transformA.R
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY
        @m_axis.NegativeSelf()  if s < 0.0
      else
        @m_type = b2SeparationFunction.e_faceA
        @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0)
        @m_localPoint = localPointA
        tVec = @m_axis
        tMat = transformA.R
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tVec = @m_localPoint
        tMat = transformA.R
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tVec = localPointB
        tMat = transformB.R
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY
        @m_axis.NegativeSelf()  if s < 0.0
    return

  b2SeparationFunction::Evaluate = (transformA, transformB) ->
    axisA = undefined
    axisB = undefined
    localPointA = undefined
    localPointB = undefined
    pointA = undefined
    pointB = undefined
    seperation = 0
    normal = undefined
    switch @m_type
      when b2SeparationFunction.e_points
        axisA = b2Math.MulTMV(transformA.R, @m_axis)
        axisB = b2Math.MulTMV(transformB.R, @m_axis.GetNegative())
        localPointA = @m_proxyA.GetSupportVertex(axisA)
        localPointB = @m_proxyB.GetSupportVertex(axisB)
        pointA = b2Math.MulX(transformA, localPointA)
        pointB = b2Math.MulX(transformB, localPointB)
        seperation = (pointB.x - pointA.x) * @m_axis.x + (pointB.y - pointA.y) * @m_axis.y
        seperation
      when b2SeparationFunction.e_faceA
        normal = b2Math.MulMV(transformA.R, @m_axis)
        pointA = b2Math.MulX(transformA, @m_localPoint)
        axisB = b2Math.MulTMV(transformB.R, normal.GetNegative())
        localPointB = @m_proxyB.GetSupportVertex(axisB)
        pointB = b2Math.MulX(transformB, localPointB)
        seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y
        seperation
      when b2SeparationFunction.e_faceB
        normal = b2Math.MulMV(transformB.R, @m_axis)
        pointB = b2Math.MulX(transformB, @m_localPoint)
        axisA = b2Math.MulTMV(transformA.R, normal.GetNegative())
        localPointA = @m_proxyA.GetSupportVertex(axisA)
        pointA = b2Math.MulX(transformA, localPointA)
        seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y
        seperation
      else
        b2Settings.b2Assert false
        0.0

  Box2D.postDefs.push ->
    Box2D.Collision.b2SeparationFunction.e_points = 0x01
    Box2D.Collision.b2SeparationFunction.e_faceA = 0x02
    Box2D.Collision.b2SeparationFunction.e_faceB = 0x04
    return

  b2Simplex.b2Simplex = ->
    @m_v1 = new b2SimplexVertex()
    @m_v2 = new b2SimplexVertex()
    @m_v3 = new b2SimplexVertex()
    @m_vertices = new Vector(3)
    return

  b2Simplex::b2Simplex = ->
    @m_vertices[0] = @m_v1
    @m_vertices[1] = @m_v2
    @m_vertices[2] = @m_v3
    return

  b2Simplex::ReadCache = (cache, proxyA, transformA, proxyB, transformB) ->
    b2Settings.b2Assert 0 <= cache.count and cache.count <= 3
    wALocal = undefined
    wBLocal = undefined
    @m_count = cache.count
    vertices = @m_vertices
    i = 0

    while i < @m_count
      v = vertices[i]
      v.indexA = cache.indexA[i]
      v.indexB = cache.indexB[i]
      wALocal = proxyA.GetVertex(v.indexA)
      wBLocal = proxyB.GetVertex(v.indexB)
      v.wA = b2Math.MulX(transformA, wALocal)
      v.wB = b2Math.MulX(transformB, wBLocal)
      v.w = b2Math.SubtractVV(v.wB, v.wA)
      v.a = 0
      i++
    if @m_count > 1
      metric1 = cache.metric
      metric2 = @GetMetric()
      @m_count = 0  if metric2 < .5 * metric1 or 2.0 * metric1 < metric2 or metric2 < Number.MIN_VALUE
    if @m_count is 0
      v = vertices[0]
      v.indexA = 0
      v.indexB = 0
      wALocal = proxyA.GetVertex(0)
      wBLocal = proxyB.GetVertex(0)
      v.wA = b2Math.MulX(transformA, wALocal)
      v.wB = b2Math.MulX(transformB, wBLocal)
      v.w = b2Math.SubtractVV(v.wB, v.wA)
      @m_count = 1
    return

  b2Simplex::WriteCache = (cache) ->
    cache.metric = @GetMetric()
    cache.count = Box2D.parseUInt(@m_count)
    vertices = @m_vertices
    i = 0

    while i < @m_count
      cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA)
      cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB)
      i++
    return

  b2Simplex::GetSearchDirection = ->
    switch @m_count
      when 1
        @m_v1.w.GetNegative()
      when 2
        e12 = b2Math.SubtractVV(@m_v2.w, @m_v1.w)
        sgn = b2Math.CrossVV(e12, @m_v1.w.GetNegative())
        if sgn > 0.0
          b2Math.CrossFV 1.0, e12
        else
          b2Math.CrossVF e12, 1.0
      else
        b2Settings.b2Assert false
        new b2Vec2()

  b2Simplex::GetClosestPoint = ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
        new b2Vec2()
      when 1
        @m_v1.w
      when 2
        new b2Vec2(@m_v1.a * @m_v1.w.x + @m_v2.a * @m_v2.w.x, @m_v1.a * @m_v1.w.y + @m_v2.a * @m_v2.w.y)
      else
        b2Settings.b2Assert false
        new b2Vec2()

  b2Simplex::GetWitnessPoints = (pA, pB) ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
      when 1
        pA.SetV @m_v1.wA
        pB.SetV @m_v1.wB
      when 2
        pA.x = @m_v1.a * @m_v1.wA.x + @m_v2.a * @m_v2.wA.x
        pA.y = @m_v1.a * @m_v1.wA.y + @m_v2.a * @m_v2.wA.y
        pB.x = @m_v1.a * @m_v1.wB.x + @m_v2.a * @m_v2.wB.x
        pB.y = @m_v1.a * @m_v1.wB.y + @m_v2.a * @m_v2.wB.y
      when 3
        pB.x = pA.x = @m_v1.a * @m_v1.wA.x + @m_v2.a * @m_v2.wA.x + @m_v3.a * @m_v3.wA.x
        pB.y = pA.y = @m_v1.a * @m_v1.wA.y + @m_v2.a * @m_v2.wA.y + @m_v3.a * @m_v3.wA.y
      else
        b2Settings.b2Assert false

  b2Simplex::GetMetric = ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
        0.0
      when 1
        0.0
      when 2
        b2Math.SubtractVV(@m_v1.w, @m_v2.w).Length()
      when 3
        b2Math.CrossVV b2Math.SubtractVV(@m_v2.w, @m_v1.w), b2Math.SubtractVV(@m_v3.w, @m_v1.w)
      else
        b2Settings.b2Assert false
        0.0

  b2Simplex::Solve2 = ->
    w1 = @m_v1.w
    w2 = @m_v2.w
    e12 = b2Math.SubtractVV(w2, w1)
    d12_2 = (-(w1.x * e12.x + w1.y * e12.y))
    if d12_2 <= 0.0
      @m_v1.a = 1.0
      @m_count = 1
      return
    d12_1 = (w2.x * e12.x + w2.y * e12.y)
    if d12_1 <= 0.0
      @m_v2.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v2
      return
    inv_d12 = 1.0 / (d12_1 + d12_2)
    @m_v1.a = d12_1 * inv_d12
    @m_v2.a = d12_2 * inv_d12
    @m_count = 2
    return

  b2Simplex::Solve3 = ->
    w1 = @m_v1.w
    w2 = @m_v2.w
    w3 = @m_v3.w
    e12 = b2Math.SubtractVV(w2, w1)
    w1e12 = b2Math.Dot(w1, e12)
    w2e12 = b2Math.Dot(w2, e12)
    d12_1 = w2e12
    d12_2 = (-w1e12)
    e13 = b2Math.SubtractVV(w3, w1)
    w1e13 = b2Math.Dot(w1, e13)
    w3e13 = b2Math.Dot(w3, e13)
    d13_1 = w3e13
    d13_2 = (-w1e13)
    e23 = b2Math.SubtractVV(w3, w2)
    w2e23 = b2Math.Dot(w2, e23)
    w3e23 = b2Math.Dot(w3, e23)
    d23_1 = w3e23
    d23_2 = (-w2e23)
    n123 = b2Math.CrossVV(e12, e13)
    d123_1 = n123 * b2Math.CrossVV(w2, w3)
    d123_2 = n123 * b2Math.CrossVV(w3, w1)
    d123_3 = n123 * b2Math.CrossVV(w1, w2)
    if d12_2 <= 0.0 and d13_2 <= 0.0
      @m_v1.a = 1.0
      @m_count = 1
      return
    if d12_1 > 0.0 and d12_2 > 0.0 and d123_3 <= 0.0
      inv_d12 = 1.0 / (d12_1 + d12_2)
      @m_v1.a = d12_1 * inv_d12
      @m_v2.a = d12_2 * inv_d12
      @m_count = 2
      return
    if d13_1 > 0.0 and d13_2 > 0.0 and d123_2 <= 0.0
      inv_d13 = 1.0 / (d13_1 + d13_2)
      @m_v1.a = d13_1 * inv_d13
      @m_v3.a = d13_2 * inv_d13
      @m_count = 2
      @m_v2.Set @m_v3
      return
    if d12_1 <= 0.0 and d23_2 <= 0.0
      @m_v2.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v2
      return
    if d13_1 <= 0.0 and d23_1 <= 0.0
      @m_v3.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v3
      return
    if d23_1 > 0.0 and d23_2 > 0.0 and d123_1 <= 0.0
      inv_d23 = 1.0 / (d23_1 + d23_2)
      @m_v2.a = d23_1 * inv_d23
      @m_v3.a = d23_2 * inv_d23
      @m_count = 2
      @m_v1.Set @m_v3
      return
    inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3)
    @m_v1.a = d123_1 * inv_d123
    @m_v2.a = d123_2 * inv_d123
    @m_v3.a = d123_3 * inv_d123
    @m_count = 3
    return

  b2SimplexCache.b2SimplexCache = ->
    @indexA = new Vector_a2j_Number(3)
    @indexB = new Vector_a2j_Number(3)
    return

  b2SimplexVertex.b2SimplexVertex = ->

  b2SimplexVertex::Set = (other) ->
    @wA.SetV other.wA
    @wB.SetV other.wB
    @w.SetV other.w
    @a = other.a
    @indexA = other.indexA
    @indexB = other.indexB
    return

  b2TimeOfImpact.b2TimeOfImpact = ->

  b2TimeOfImpact.TimeOfImpact = (input) ->
    ++b2TimeOfImpact.b2_toiCalls
    proxyA = input.proxyA
    proxyB = input.proxyB
    sweepA = input.sweepA
    sweepB = input.sweepB
    b2Settings.b2Assert sweepA.t0 is sweepB.t0
    b2Settings.b2Assert 1.0 - sweepA.t0 > Number.MIN_VALUE
    radius = proxyA.m_radius + proxyB.m_radius
    tolerance = input.tolerance
    alpha = 0.0
    k_maxIterations = 1000
    iter = 0
    target = 0.0
    b2TimeOfImpact.s_cache.count = 0
    b2TimeOfImpact.s_distanceInput.useRadii = false
    loop
      sweepA.GetTransform b2TimeOfImpact.s_xfA, alpha
      sweepB.GetTransform b2TimeOfImpact.s_xfB, alpha
      b2TimeOfImpact.s_distanceInput.proxyA = proxyA
      b2TimeOfImpact.s_distanceInput.proxyB = proxyB
      b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA
      b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB
      b2Distance.Distance b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput
      if b2TimeOfImpact.s_distanceOutput.distance <= 0.0
        alpha = 1.0
        break
      b2TimeOfImpact.s_fcn.Initialize b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB
      separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if separation <= 0.0
        alpha = 1.0
        break
      if iter is 0
        if separation > radius
          target = b2Math.Max(radius - tolerance, 0.75 * radius)
        else
          target = b2Math.Max(separation - tolerance, 0.02 * radius)
      if separation - target < 0.5 * tolerance
        if iter is 0
          alpha = 1.0
          break
        break
      newAlpha = alpha
      x1 = alpha
      x2 = 1.0
      f1 = separation
      sweepA.GetTransform b2TimeOfImpact.s_xfA, x2
      sweepB.GetTransform b2TimeOfImpact.s_xfB, x2
      f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if f2 >= target
        alpha = 1.0
        break
      rootIterCount = 0
      loop
        x = 0
        if rootIterCount & 1
          x = x1 + (target - f1) * (x2 - x1) / (f2 - f1)
        else
          x = 0.5 * (x1 + x2)
        sweepA.GetTransform b2TimeOfImpact.s_xfA, x
        sweepB.GetTransform b2TimeOfImpact.s_xfB, x
        f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
        if b2Math.Abs(f - target) < 0.025 * tolerance
          newAlpha = x
          break
        if f > target
          x1 = x
          f1 = f
        else
          x2 = x
          f2 = f
        ++rootIterCount
        ++b2TimeOfImpact.b2_toiRootIters
        break  if rootIterCount is 50
      b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount)
      break  if newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha
      alpha = newAlpha
      iter++
      ++b2TimeOfImpact.b2_toiIters
      break  if iter is k_maxIterations
    b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter)
    alpha

  Box2D.postDefs.push ->
    Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0
    Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache()
    Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput()
    Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform()
    Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform()
    Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction()
    Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput()
    return

  b2TOIInput.b2TOIInput = ->
    @proxyA = new b2DistanceProxy()
    @proxyB = new b2DistanceProxy()
    @sweepA = new b2Sweep()
    @sweepB = new b2Sweep()
    return

  b2WorldManifold.b2WorldManifold = ->
    @m_normal = new b2Vec2()
    return

  b2WorldManifold::b2WorldManifold = ->
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2Vec2()
      i++
    return

  b2WorldManifold::Initialize = (manifold, xfA, radiusA, xfB, radiusB) ->
    radiusA = 0  if radiusA is `undefined`
    radiusB = 0  if radiusB is `undefined`
    return  if manifold.m_pointCount is 0
    i = 0
    tVec = undefined
    tMat = undefined
    normalX = 0
    normalY = 0
    planePointX = 0
    planePointY = 0
    clipPointX = 0
    clipPointY = 0
    switch manifold.m_type
      when b2Manifold.e_circles
        tMat = xfA.R
        tVec = manifold.m_localPoint
        pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfB.R
        tVec = manifold.m_points[0].m_localPoint
        pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        dX = pointBX - pointAX
        dY = pointBY - pointAY
        d2 = dX * dX + dY * dY
        if d2 > Number.MIN_VALUE * Number.MIN_VALUE
          d = Math.sqrt(d2)
          @m_normal.x = dX / d
          @m_normal.y = dY / d
        else
          @m_normal.x = 1
          @m_normal.y = 0
        cAX = pointAX + radiusA * @m_normal.x
        cAY = pointAY + radiusA * @m_normal.y
        cBX = pointBX - radiusB * @m_normal.x
        cBY = pointBY - radiusB * @m_normal.y
        @m_points[0].x = 0.5 * (cAX + cBX)
        @m_points[0].y = 0.5 * (cAY + cBY)
      when b2Manifold.e_faceA
        tMat = xfA.R
        tVec = manifold.m_localPlaneNormal
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfA.R
        tVec = manifold.m_localPoint
        planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        @m_normal.x = normalX
        @m_normal.y = normalY
        i = 0
        while i < manifold.m_pointCount
          tMat = xfB.R
          tVec = manifold.m_points[i].m_localPoint
          clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
          clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
          @m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX
          @m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY
          i++
      when b2Manifold.e_faceB
        tMat = xfB.R
        tVec = manifold.m_localPlaneNormal
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfB.R
        tVec = manifold.m_localPoint
        planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        @m_normal.x = (-normalX)
        @m_normal.y = (-normalY)
        i = 0
        while i < manifold.m_pointCount
          tMat = xfA.R
          tVec = manifold.m_points[i].m_localPoint
          clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
          clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
          @m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX
          @m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY
          i++

  ClipVertex.ClipVertex = ->
    @v = new b2Vec2()
    @id = new b2ContactID()
    return

  ClipVertex::Set = (other) ->
    @v.SetV other.v
    @id.Set other.id
    return

  Features.Features = ->

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    get: ->
      @_referenceEdge

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_referenceEdge = value
      @_m_id._key = (@_m_id._key & 0xffffff00) | (@_referenceEdge & 0x000000ff)
      return

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    get: ->
      @_incidentEdge

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentEdge = value
      @_m_id._key = (@_m_id._key & 0xffff00ff) | ((@_incidentEdge << 8) & 0x0000ff00)
      return

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    get: ->
      @_incidentVertex

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentVertex = value
      @_m_id._key = (@_m_id._key & 0xff00ffff) | ((@_incidentVertex << 16) & 0x00ff0000)
      return

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    get: ->
      @_flip

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_flip = value
      @_m_id._key = (@_m_id._key & 0x00ffffff) | ((@_flip << 24) & 0xff000000)
      return

  return
)()
