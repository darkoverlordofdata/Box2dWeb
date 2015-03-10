class Box2D.Common.Math.b2Sweep

  localCenter   : null
  c0            : null
  c             : null
  a0            : null
  a             : null
  t0            : null

  constructor: ->
    @localCenter = new b2Vec2()
    @c0 = new b2Vec2
    @c = new b2Vec2()
    return

  Set: (other) ->
    @localCenter.SetV other.localCenter
    @c0.SetV other.c0
    @c.SetV other.c
    @a0 = other.a0
    @a = other.a
    @t0 = other.t0
    return

  Copy: ->
    copy = new b2Sweep()
    copy.localCenter.SetV @localCenter
    copy.c0.SetV @c0
    copy.c.SetV @c
    copy.a0 = @a0
    copy.a = @a
    copy.t0 = @t0
    return copy

  GetTransform: (xf, alpha) ->
    alpha = 0  if alpha is undefined
    xf.position.x = (1.0 - alpha) * @c0.x + alpha * @c.x
    xf.position.y = (1.0 - alpha) * @c0.y + alpha * @c.y
    angle = (1.0 - alpha) * @a0 + alpha * @a
    xf.R.Set angle
    tMat = xf.R
    xf.position.x -= (tMat.col1.x * @localCenter.x + tMat.col2.x * @localCenter.y)
    xf.position.y -= (tMat.col1.y * @localCenter.x + tMat.col2.y * @localCenter.y)
    return

  Advance: (t) ->
    t = 0  if t is undefined
    if @t0 < t and 1.0 - @t0 > Number.MIN_VALUE
      alpha = (t - @t0) / (1.0 - @t0)
      @c0.x = (1.0 - alpha) * @c0.x + alpha * @c.x
      @c0.y = (1.0 - alpha) * @c0.y + alpha * @c.y
      @a0 = (1.0 - alpha) * @a0 + alpha * @a
      @t0 = t
    return



