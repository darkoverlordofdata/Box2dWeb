Box2D = require('../../index')

b2Vec2 = Box2D.Common.Math.b2Vec2

class Box2D.Common.Math.b2Mat22

  col1: null
  col2: null

  constructor: ->
    @col1 = new b2Vec2()
    @col2 = new b2Vec2()
    @SetIdentity()

  @FromAngle: (angle=0) ->
    mat = new b2Mat22()
    mat.Set angle
    mat

  @FromVV: (c1, c2) ->
    mat = new b2Mat22()
    mat.SetVV c1, c2
    mat

  Set: (angle=0) ->
    c = Math.cos(angle)
    s = Math.sin(angle)
    @col1.x = c
    @col2.x = (-s)
    @col1.y = s
    @col2.y = c
    return

  SetVV: (c1, c2) ->
    @col1.SetV c1
    @col2.SetV c2
    return

  Copy: ->
    mat = new b2Mat22()
    mat.SetM this
    mat

  SetM: (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    return

  AddM: (m) ->
    @col1.x += m.col1.x
    @col1.y += m.col1.y
    @col2.x += m.col2.x
    @col2.y += m.col2.y
    return

  SetIdentity: ->
    @col1.x = 1.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 1.0
    return

  SetZero: ->
    @col1.x = 0.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 0.0
    return

  GetAngle: ->
    Math.atan2 @col1.y, @col1.x

  GetInverse: (out) ->
    a = @col1.x
    b = @col2.x
    c = @col1.y
    d = @col2.y
    det = a * d - b * c
    det = 1.0 / det  if det isnt 0.0
    out.col1.x = det * d
    out.col2.x = (-det * b)
    out.col1.y = (-det * c)
    out.col2.y = det * a
    out

  Solve: (out, bX=0, bY=0) ->
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  if det isnt 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    out

  Abs: ->
    @col1.Abs()
    @col2.Abs()
    return


