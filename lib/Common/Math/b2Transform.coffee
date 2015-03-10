class Box2D.Common.Math.b2Transform

  position: null
  R: null
  
  constructor: (pos, r) ->
    @position = new b2Vec2
    @R = new b2Mat22()
    pos = null  if pos is undefined
    r = null  if r is undefined
    if pos
      @position.SetV pos
      @R.SetM r
    return

  Initialize: (pos, r) ->
    @position.SetV pos
    @R.SetM r
    return

  SetIdentity: ->
    @position.SetZero()
    @R.SetIdentity()
    return

  Set: (x) ->
    @position.SetV x.position
    @R.SetM x.R
    return

  GetAngle: ->
    Math.atan2 @R.col1.y, @R.col1.x