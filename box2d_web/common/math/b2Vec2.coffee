Box2D = require('../../index')

class Box2D.Common.Math.b2Vec2

  x: 0
  y: 0

  constructor: (@x=0, @y=0) ->

  SetZero: ->
    @x = 0.0
    @y = 0.0
    return

  Set: (x=0, y=0) ->
    @x = x
    @y = y
    return

  SetV: (v) ->
    console.error "undefined 'v' in b2Vec2.SetV"  if v is undefined
    @x = v.x
    @y = v.y
    return

  @Make = (x=0, y=0) ->
    new b2Vec2(x, y)


  # Functions to mimic Construct2's b2vec2 cache interface so this extension
  # can be included in Construct2 games without modification
  if b2Vec2.Get is undefined
    b2Vec2.Get = b2Vec2.Make
    b2Vec2._freeCache = []
    b2Vec2.Free = ->

  Copy: ->
    new b2Vec2(@x, @y)

  Add: (v) ->
    console.error "undefined 'v' in b2Vec2.Add"  if v is undefined
    @x += v.x
    @y += v.y
    return

  Subtract: (v) ->
    console.error "undefined 'v' in b2Vec2.Subtract"  if v is undefined
    @x -= v.x
    @y -= v.y
    return

  Multiply: (a) ->
    a = 0  if a is undefined
    @x *= a
    @y *= a
    return

  Length: ->
    Math.sqrt @x * @x + @y * @y

  LengthSquared: ->
    @x * @x + @y * @y

  Normalize: ->
    length = Math.sqrt(@x * @x + @y * @y)
    return 0.0  if length < Number.MIN_VALUE
    invLength = 1.0 / length
    @x *= invLength
    @y *= invLength
    length

  NegativeSelf: ->
    @x = (-@x)
    @y = (-@y)
    return


