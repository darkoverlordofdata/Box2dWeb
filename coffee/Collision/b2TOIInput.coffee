class Box2D.Collision.b2TOIInput

  proxyA    : null
  proxyB    : null
  sweepA    : null
  sweepB    : null

  constructor: ->
    @proxyA = new b2DistanceProxy()
    @proxyB = new b2DistanceProxy()
    @sweepA = new b2Sweep()
    @sweepB = new b2Sweep()
    return