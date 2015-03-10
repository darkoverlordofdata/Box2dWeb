class Box2D.Collision.b2Point

  p: null

  constructor: ->
    @p = new b2Vec2()
    return

  Support: (xf, vX, vY) ->
    vX = 0  if vX is undefined
    vY = 0  if vY is undefined
    return @p

  GetFirstVertex: (xf) ->
    return @p

