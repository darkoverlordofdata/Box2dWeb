Box2D = require('../../index')

b2Shape = Box2D.Collision.Shapes.b2Shape

class Box2D.Collision.Shapes.b2PolygonShape extends b2Shape

  type          : ''
  width         : 0
  height        : 0
  p1x           : 0
  p1y           : 0
  p2x           : 0
  p2y           : 0
  vertices      : null

  constructor: ->
    super
    @m_type = b2Shape.e_polygonShape
    @m_centroid = new b2Vec2()
    @m_vertices = new Vector()
    @m_normals = new Vector()
    return


  Copy: ->
    s = new b2PolygonShape()
    s.Set this
    s

  Set: (other) ->
    super other
    if Box2D.equals(other, b2PolygonShape)
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

  SetAsArray: (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is undefined
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

  @AsArray: (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is undefined
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsArray vertices, vertexCount
    polygonShape

  SetAsVector: (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is undefined
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

  @AsVector: (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is undefined
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsVector vertices, vertexCount
    polygonShape

  SetAsBox: (hx, hy) ->
    hx = 0  if hx is undefined
    hy = 0  if hy is undefined
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

  @AsBox: (hx, hy) ->
    hx = 0  if hx is undefined
    hy = 0  if hy is undefined
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsBox hx, hy
    polygonShape

  SetAsOrientedBox: (hx, hy, center, angle) ->
    hx = 0  if hx is undefined
    hy = 0  if hy is undefined
    center = null  if center is undefined
    angle = 0.0  if angle is undefined
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

  @AsOrientedBox: (hx, hy, center, angle) ->
    hx = 0  if hx is undefined
    hy = 0  if hy is undefined
    center = null  if center is undefined
    angle = 0.0  if angle is undefined
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsOrientedBox hx, hy, center, angle
    polygonShape

  SetAsEdge: (v1, v2) ->
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

  @AsEdge: (v1, v2) ->
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsEdge v1, v2
    polygonShape

  TestPoint: (xf, p) ->
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

  RayCast: (output, input, transform) ->
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

  ComputeAABB: (aabb, xf) ->
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

  ComputeMass: (massData, density) ->
    density = 0  if density is undefined
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

  ComputeSubmergedArea: (normal, offset, xf, c) ->
    offset = 0  if offset is undefined
    normalL = b2Math.MulTMV(xf.R, normal)
    offsetL = offset - b2Math.Dot(normal, xf.position)
    depths = new Vector()
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

  GetVertexCount: ->
    @m_vertexCount

  GetVertices: ->
    @m_vertices

  GetNormals: ->
    @m_normals

  GetSupport: (d) ->
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

  GetSupportVertex: (d) ->
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

  Validate: ->
    false

  Reserve: (count) ->
    count = 0  if count is undefined
    i = parseInt(@m_vertices.length)

    while i < count
      @m_vertices[i] = new b2Vec2()
      @m_normals[i] = new b2Vec2()
      i++
    return

  @ComputeCentroid = (vs, count) ->
    count = 0  if count is undefined
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

  @ComputeOBB = (obb, vs, count) ->
    count = 0  if count is undefined
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

  @b2PolygonShape.s_mat = new b2Mat22()
