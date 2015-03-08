Box2D = require('../../index')

Vector        = Box2D.Vector
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Contacts.b2PositionSolverManifold

  m_normal          : null
  m_separations     : null
  m_points          : null


  constructor: ->
    @m_normal = new b2Vec2()
    @m_separations = new Vector(b2Settings.b2_maxManifoldPoints)
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2Vec2()
      i++
    return

  Initialize: (cc) ->
    b2Settings.b2Assert cc.pointCount > 0
    i = 0
    clipPointX = 0
    clipPointY = 0
    tMat = undefined
    tVec = undefined
    planePointX = 0
    planePointY = 0
    switch cc.type
      when b2Manifold.e_circles
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPoint
        pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyB.m_xf.R
        tVec = cc.points[0].localPoint
        pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        dX = pointBX - pointAX
        dY = pointBY - pointAY
        d2 = dX * dX + dY * dY
        if d2 > Number.MIN_VALUE * Number.MIN_VALUE
          d = Math.sqrt(d2)
          @m_normal.x = dX / d
          @m_normal.y = dY / d
        else
          @m_normal.x = 1.0
          @m_normal.y = 0.0
        @m_points[0].x = 0.5 * (pointAX + pointBX)
        @m_points[0].y = 0.5 * (pointAY + pointBY)
        @m_separations[0] = dX * @m_normal.x + dY * @m_normal.y - cc.radius
      when b2Manifold.e_faceA
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPlaneNormal
        @m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        @m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPoint
        planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyB.m_xf.R
        i = 0
        while i < cc.pointCount
          tVec = cc.points[i].localPoint
          clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
          clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
          @m_separations[i] = (clipPointX - planePointX) * @m_normal.x + (clipPointY - planePointY) * @m_normal.y - cc.radius
          @m_points[i].x = clipPointX
          @m_points[i].y = clipPointY
          ++i
      when b2Manifold.e_faceB
        tMat = cc.bodyB.m_xf.R
        tVec = cc.localPlaneNormal
        @m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        @m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = cc.bodyB.m_xf.R
        tVec = cc.localPoint
        planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyA.m_xf.R
        i = 0
        while i < cc.pointCount
          tVec = cc.points[i].localPoint
          clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
          clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
          @m_separations[i] = (clipPointX - planePointX) * @m_normal.x + (clipPointY - planePointY) * @m_normal.y - cc.radius
          @m_points[i].Set clipPointX, clipPointY
          ++i
        @m_normal.x *= (-1)
        @m_normal.y *= (-1)

  @circlePointA = new b2Vec2()
  @circlePointB = new b2Vec2()
