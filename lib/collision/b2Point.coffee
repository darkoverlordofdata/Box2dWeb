Box2D = require('../index')


class Box2D.Collision.b2Point

  constructor: ->
    @p = new b2Vec2()
    return

  Support: (xf, vX, vY) ->
    vX = 0  if vX is `undefined`
    vY = 0  if vY is `undefined`
    @p

  GetFirstVertex: (xf) ->
    @p

