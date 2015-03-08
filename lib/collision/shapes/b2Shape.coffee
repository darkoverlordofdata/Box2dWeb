Box2D = require('../../index')

class Box2D.Collision.Shapes.b2Shape

  constructor: ->
    @m_type = b2Shape.e_unknownShape
    @m_radius = b2Settings.b2_linearSlop
    return

  Copy: ->
    null

  Set: (other) ->
    @m_radius = other.m_radius
    return

  GetType: ->
    @m_type

  TestPoint: (xf, p) ->
    false

  RayCast: (output, input, transform) ->
    false

  ComputeAABB: (aabb, xf) ->

  ComputeMass: (massData, density) ->
    density = 0  if density is `undefined`
    return

  ComputeSubmergedArea: (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    0

  @TestOverlap = (shape1, transform1, shape2, transform2) ->
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


  @e_unknownShape = parseInt((-1))
  @e_circleShape = 0
  @e_polygonShape = 1
  @e_edgeShape = 2
  @e_shapeTypeCount = 3
  @e_hitCollide = 1
  @e_missCollide = 0
  @e_startsInsideCollide = parseInt((-1))
