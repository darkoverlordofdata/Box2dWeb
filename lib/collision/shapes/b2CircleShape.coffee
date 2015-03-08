Box2D = require('../../index')

b2Shape = Box2D.Collision.Shapes.b2Shape

class Box2D.Collision.Shapes.b2CircleShape extends b2Shape

  constructor: (radius) ->
    super radius
    @m_p = new b2Vec2()
    radius = 0  if radius is `undefined`
    @__super.b2Shape.call this
    @m_type = b2Shape.e_circleShape
    @m_radius = radius
    return

  Copy: ->
    s = new b2CircleShape()
    s.Set this
    s

  Set: (other) ->
    @__super.Set.call this, other
    if Box2D.is(other, b2CircleShape)
      other2 = ((if other instanceof b2CircleShape then other else null))
      @m_p.SetV other2.m_p
    return

  TestPoint: (transform, p) ->
    tMat = transform.R
    dX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    dY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    dX = p.x - dX
    dY = p.y - dY
    (dX * dX + dY * dY) <= @m_radius * @m_radius

  RayCast: (output, input, transform) ->
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

  ComputeAABB: (aabb, transform) ->
    tMat = transform.R
    pX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    pY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    aabb.lowerBound.Set pX - @m_radius, pY - @m_radius
    aabb.upperBound.Set pX + @m_radius, pY + @m_radius
    return

  ComputeMass: (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = density * b2Settings.b2_pi * @m_radius * @m_radius
    massData.center.SetV @m_p
    massData.I = massData.mass * (0.5 * @m_radius * @m_radius + (@m_p.x * @m_p.x + @m_p.y * @m_p.y))
    return

  ComputeSubmergedArea: (normal, offset, xf, c) ->
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

  GetLocalPosition: ->
    @m_p

  SetLocalPosition: (position) ->
    @m_p.SetV position
    return

  GetRadius: ->
    @m_radius

  SetRadius: (radius) ->
    radius = 0  if radius is `undefined`
    @m_radius = radius
    return

