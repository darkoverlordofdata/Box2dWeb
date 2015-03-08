Box2D = require('../../index')

b2Mat22     = Box2D.Common.Math.b2Mat22
b2Math      = Box2D.Common.Math.b2Math
b2Transform = Box2D.Common.Math.b2Transform
b2Vec2      = Box2D.Common.Math.b2Vec2

class Box2D.Common.Math.b2Math

  @IsValid = (x) ->
    x = 0  if x is undefined
    return isFinite x

  @Dot = (a, b) ->
    return a.x * b.x + a.y * b.y

  @CrossVV = (a, b) ->
    return a.x * b.y - a.y * b.x

  @CrossVF = (a, s) ->
    s = 0  if s is undefined
    v = new b2Vec2(s * a.y, (-s * a.x))
    return v

  @CrossFV = (s, a) ->
    s = 0  if s is undefined
    v = new b2Vec2((-s * a.y), s * a.x)
    return v

  @MulMV = (A, v) ->
    u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y)
    return u

  @MulTMV = (A, v) ->
    u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2))
    return u

  @MulX = (T, v) ->
    a = b2Math.MulMV(T.R, v)
    a.x += T.position.x
    a.y += T.position.y
    return a

  @MulXT = (T, v) ->
    a = b2Math.SubtractVV(v, T.position)
    tX = (a.x * T.R.col1.x + a.y * T.R.col1.y)
    a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y)
    a.x = tX
    return a

  @AddVV = (a, b) ->
    v = new b2Vec2(a.x + b.x, a.y + b.y)
    return v

  @SubtractVV = (a, b) ->
    v = new b2Vec2(a.x - b.x, a.y - b.y)
    return v

  @Distance = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    return Math.sqrt cX * cX + cY * cY

  @DistanceSquared = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    return cX * cX + cY * cY

  @MulFV = (s, a) ->
    s = 0  if s is undefined
    v = new b2Vec2(s * a.x, s * a.y)
    return v

  @AddMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2))
    return C

  @MulMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2))
    return C

  @MulTMM = (A, B) ->
    c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1))
    c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2))
    C = b2Mat22.FromVV(c1, c2)
    return C

  @Abs = (a) ->
    a = 0  if a is undefined
    return (if a > 0.0 then a else (-a))

  @AbsV = (a) ->
    b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y))
    return b

  @AbsM = (A) ->
    B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2))
    return B

  @Min = (a, b) ->
    a = 0  if a is undefined
    b = 0  if b is undefined
    return (if a < b then a else b)

  @MinV = (a, b) ->
    c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y))
    return c

  @Max = (a, b) ->
    a = 0  if a is undefined
    b = 0  if b is undefined
    return (if a > b then a else b)

  @MaxV = (a, b) ->
    c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y))
    return c

  @Clamp = (a, low, high) ->
    a = 0  if a is undefined
    low = 0  if low is undefined
    high = 0  if high is undefined
    return (if a < low then low else (if a > high then high else a))

  @ClampV = (a, low, high) ->
    return b2Math.MaxV low, b2Math.MinV(a, high)

  @Swap = (a, b) ->
    tmp = a[0]
    a[0] = b[0]
    b[0] = tmp
    return

  @Random = ->
    return Math.random() * 2 - 1

  @RandomRange = (lo, hi) ->
    lo = 0  if lo is undefined
    hi = 0  if hi is undefined
    r = Math.random()
    r = (hi - lo) * r + lo
    return r

  @NextPowerOfTwo = (x) ->
    x = 0  if x is undefined
    x |= (x >> 1) & 0x7FFFFFFF
    x |= (x >> 2) & 0x3FFFFFFF
    x |= (x >> 4) & 0x0FFFFFFF
    x |= (x >> 8) & 0x00FFFFFF
    x |= (x >> 16) & 0x0000FFFF
    return x + 1

  @IsPowerOfTwo = (x) ->
    x = 0  if x is undefined
    result = x > 0 and (x & (x - 1)) is 0
    return result

  @b2Vec2_zero = new b2Vec2(0.0, 0.0)
  @b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0))
  @b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity)

