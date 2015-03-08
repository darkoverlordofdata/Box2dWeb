Box2D = require('../../index')

b2Vec2 = Box2D.Common.Math.b2Vec2

class Box2D.Common.Math.b2Mat33

  col1: null
  col2: null
  col3: null
  
  constructor: (c1, c2, c3) ->
    c1 = null  if c1 is undefined
    c2 = null  if c2 is undefined
    c3 = null  if c3 is undefined

    @col1 = new b2Vec3()
    @col2 = new b2Vec3()
    @col3 = new b2Vec3()
    if not c1 and not c2 and not c3
      @col1.SetZero()
      @col2.SetZero()
      @col3.SetZero()
    else
      @col1.SetV c1
      @col2.SetV c2
      @col3.SetV c3
    return

  SetVVV: (c1, c2, c3) ->
    @col1.SetV c1
    @col2.SetV c2
    @col3.SetV c3
    return

  Copy: ->
    new b2Mat33(@col1, @col2, @col3)
    return

  SetM: (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    @col3.SetV m.col3
    return

  AddM: (m) ->
    @col1.x += m.col1.x
    @col1.y += m.col1.y
    @col1.z += m.col1.z
    @col2.x += m.col2.x
    @col2.y += m.col2.y
    @col2.z += m.col2.z
    @col3.x += m.col3.x
    @col3.y += m.col3.y
    @col3.z += m.col3.z
    return

  SetIdentity: ->
    @col1.x = 1.0
    @col2.x = 0.0
    @col3.x = 0.0
    @col1.y = 0.0
    @col2.y = 1.0
    @col3.y = 0.0
    @col1.z = 0.0
    @col2.z = 0.0
    @col3.z = 1.0
    return

  SetZero: ->
    @col1.x = 0.0
    @col2.x = 0.0
    @col3.x = 0.0
    @col1.y = 0.0
    @col2.y = 0.0
    @col3.y = 0.0
    @col1.z = 0.0
    @col2.z = 0.0
    @col3.z = 0.0
    return

  Solve22: (out, bX, bY) ->
    bX = 0  if bX is undefined
    bY = 0  if bY is undefined
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  unless det is 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    return out

  Solve33: (out, bX, bY, bZ) ->
    bX = 0  if bX is undefined
    bY = 0  if bY is undefined
    bZ = 0  if bZ is undefined
    a11 = @col1.x
    a21 = @col1.y
    a31 = @col1.z
    a12 = @col2.x
    a22 = @col2.y
    a32 = @col2.z
    a13 = @col3.x
    a23 = @col3.y
    a33 = @col3.z
    det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13)
    det = 1.0 / det  unless det is 0.0
    out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13))
    out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13))
    out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX))
    return out
