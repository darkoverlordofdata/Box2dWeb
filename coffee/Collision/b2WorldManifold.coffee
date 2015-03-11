class Box2D.Collision.b2WorldManifold

  m_normal          : null
  m_points          : null

  constructor: ->
    @m_normal = new b2Vec2()
    @m_points = new Array(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2Vec2()
      i++
    return

  Initialize: (manifold, xfA, radiusA, xfB, radiusB) ->
    radiusA = 0  if radiusA is undefined
    radiusB = 0  if radiusB is undefined
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
