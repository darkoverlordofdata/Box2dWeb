class Box2D.Collision.b2RayCastInput

  p1            : null
  p2            : null
  maxFraction   : 1

  constructor: (p1, p2, maxFraction) ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    p1 = null  if p1 is undefined
    p2 = null  if p2 is undefined
    maxFraction = 1  if maxFraction is undefined
    @p1.SetV p1  if p1
    @p2.SetV p2  if p2
    @maxFraction = maxFraction
    return

