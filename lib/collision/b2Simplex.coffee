Box2D = require('../index')

Vector                    = Box2D.Vector
b2Settings                = Box2d.Common.b2Settings
b2Math                    = Box2d.Common.Math.b2Math
b2Vec2                    = Box2D.Common.Math.b2Vec2
b2SimplexVertex           = Box2D.Collision.b2SimplexVertex


class Box2D.Collision.b2Simplex

  m_v1              : null
  m_v2              : null
  m_v3              : null
  m_vertices        : null
  m_count           : 0



  constructor: ->
    @m_v1 = new b2SimplexVertex()
    @m_v2 = new b2SimplexVertex()
    @m_v3 = new b2SimplexVertex()
    @m_vertices = new Vector(3)
    @m_vertices[0] = @m_v1
    @m_vertices[1] = @m_v2
    @m_vertices[2] = @m_v3
    return

  ReadCache: (cache, proxyA, transformA, proxyB, transformB) ->
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

  WriteCache: (cache) ->
    cache.metric = @GetMetric()
    cache.count = Box2D.parseUInt(@m_count)
    vertices = @m_vertices
    i = 0

    while i < @m_count
      cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA)
      cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB)
      i++
    return

  GetSearchDirection: ->
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

  GetClosestPoint: ->
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

  GetWitnessPoints: (pA, pB) ->
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

  GetMetric: ->
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

  Solve2: ->
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

  Solve3: ->
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
