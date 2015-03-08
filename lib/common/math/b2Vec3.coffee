Box2D = require('../../index')

class Box2D.Common.Math.b2Vec3

  x: 0
  y: 0
  z: 0
  
  
  constructor: (x, y, z) ->
    x = 0  if x is undefined
    y = 0  if y is undefined
    z = 0  if z is undefined
    @x = x
    @y = y
    @z = z
    return

  SetZero: ->
    @x = @y = @z = 0.0
    return

  Set: (x, y, z) ->
    x = 0  if x is undefined
    y = 0  if y is undefined
    z = 0  if z is undefined
    @x = x
    @y = y
    @z = z
    return

  SetV: (v) ->
    @x = v.x
    @y = v.y
    @z = v.z
    return

  GetNegative: ->
    return new b2Vec3((-@x), (-@y), (-@z))

  NegativeSelf: ->
    @x = (-@x)
    @y = (-@y)
    @z = (-@z)
    return

  Copy: ->
    return new b2Vec3(@x, @y, @z)

  Add: (v) ->
    @x += v.x
    @y += v.y
    @z += v.z
    return

  Subtract: (v) ->
    @x -= v.x
    @y -= v.y
    @z -= v.z
    return

  Multiply: (a) ->
    a = 0  if a is undefined
    @x *= a
    @y *= a
    @z *= a
    return

