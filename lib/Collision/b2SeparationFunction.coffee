class Box2D.Collision.b2SeparationFunction

  m_localPoint        : null
  m_axis              : null
  m_proxyA            : null
  m_proxyB            : null

  constructor: ->
    @m_localPoint = new b2Vec2()
    @m_axis = new b2Vec2()
    return

  Initialize: (cache, proxyA, transformA, proxyB, transformB) ->
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

  Evaluate: (transformA, transformB) ->
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

  @e_points = 0x01
  @e_faceA = 0x02
  @e_faceB = 0x04
