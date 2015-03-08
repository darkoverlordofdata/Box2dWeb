Box2D = require('../index')


class Box2D.Collision.b2Distance

  constructor: (output, cache, input) ->
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

  @s_simplex = new b2Simplex()
  @s_saveA = new Vector(3)
  @s_saveB = new Vector(3)
