(->
  b2AABB = Box2D.Collision.b2AABB
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2Mat22.b2Mat22 = ->
    @col1 = new b2Vec2()
    @col2 = new b2Vec2()
    return

  b2Mat22::b2Mat22 = ->
    @SetIdentity()
    return

  b2Mat22.FromAngle = (angle) ->
    angle = 0  if angle is `undefined`
    mat = new b2Mat22()
    mat.Set angle
    mat

  b2Mat22.FromVV = (c1, c2) ->
    mat = new b2Mat22()
    mat.SetVV c1, c2
    mat

  b2Mat22::Set = (angle) ->
    angle = 0  if angle is `undefined`
    c = Math.cos(angle)
    s = Math.sin(angle)
    @col1.x = c
    @col2.x = (-s)
    @col1.y = s
    @col2.y = c
    return

  b2Mat22::SetVV = (c1, c2) ->
    @col1.SetV c1
    @col2.SetV c2
    return

  b2Mat22::Copy = ->
    mat = new b2Mat22()
    mat.SetM this
    mat

  b2Mat22::SetM = (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    return

  b2Mat22::AddM = (m) ->
    @col1.x += m.col1.x
    @col1.y += m.col1.y
    @col2.x += m.col2.x
    @col2.y += m.col2.y
    return

  b2Mat22::SetIdentity = ->
    @col1.x = 1.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 1.0
    return

  b2Mat22::SetZero = ->
    @col1.x = 0.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 0.0
    return

  b2Mat22::GetAngle = ->
    Math.atan2 @col1.y, @col1.x

  b2Mat22::GetInverse = (out) ->
    a = @col1.x
    b = @col2.x
    c = @col1.y
    d = @col2.y
    det = a * d - b * c
    det = 1.0 / det  unless det is 0.0
    out.col1.x = det * d
    out.col2.x = (-det * b)
    out.col1.y = (-det * c)
    out.col2.y = det * a
    out

  b2Mat22::Solve = (out, bX, bY) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  unless det is 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    out

  b2Mat22::Abs = ->
    @col1.Abs()
    @col2.Abs()
    return

  b2Mat33.b2Mat33 = ->
    @col1 = new b2Vec3()
    @col2 = new b2Vec3()
    @col3 = new b2Vec3()
    return

  b2Mat33::b2Mat33 = (c1, c2, c3) ->
    c1 = null  if c1 is `undefined`
    c2 = null  if c2 is `undefined`
    c3 = null  if c3 is `undefined`
    if not c1 and not c2 and not c3
      @col1.SetZero()
      @col2.SetZero()
      @col3.SetZero()
    else
      @col1.SetV c1
      @col2.SetV c2
      @col3.SetV c3
    return

  b2Mat33::SetVVV = (c1, c2, c3) ->
    @col1.SetV c1
    @col2.SetV c2
    @col3.SetV c3
    return

  b2Mat33::Copy = ->
    new b2Mat33(@col1, @col2, @col3)

  b2Mat33::SetM = (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    @col3.SetV m.col3
    return

  b2Mat33::AddM = (m) ->
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

  b2Mat33::SetIdentity = ->
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

  b2Mat33::SetZero = ->
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

  b2Mat33::Solve22 = (out, bX, bY) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  unless det is 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    out

  b2Mat33::Solve33 = (out, bX, bY, bZ) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    bZ = 0  if bZ is `undefined`
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
    out

  b2Math.b2Math = ->

  b2Math.IsValid = (x) ->
    x = 0  if x is `undefined`
    isFinite x

  b2Math.Dot = (a, b) ->
    a.x * b.x + a.y * b.y

  b2Math.CrossVV = (a, b) ->
    a.x * b.y - a.y * b.x

  b2Math.CrossVF = (a, s) ->
    s = 0  if s is `undefined`
    v = new b2Vec2(s * a.y, (-s * a.x))
    v

  b2Math.CrossFV = (s, a) ->
    s = 0  if s is `undefined`
    v = new b2Vec2((-s * a.y), s * a.x)
    v

  b2Math.MulMV = (A, v) ->
    u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y)
    u

  b2Math.MulTMV = (A, v) ->
    u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2))
    u

  b2Math.MulX = (T, v) ->
    a = b2Math.MulMV(T.R, v)
    a.x += T.position.x
    a.y += T.position.y
    a

  b2Math.MulXT = (T, v) ->
    a = b2Math.SubtractVV(v, T.position)
    tX = (a.x * T.R.col1.x + a.y * T.R.col1.y)
    a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y)
    a.x = tX
    a

  b2Math.AddVV = (a, b) ->
    v = new b2Vec2(a.x + b.x, a.y + b.y)
    v

  b2Math.SubtractVV = (a, b) ->
    v = new b2Vec2(a.x - b.x, a.y - b.y)
    v

  b2Math.Distance = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    Math.sqrt cX * cX + cY * cY

  b2Math.DistanceSquared = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    cX * cX + cY * cY

  b2Math.MulFV = (s, a) ->
    s = 0  if s is `undefined`
    v = new b2Vec2(s * a.x, s * a.y)
    v

  b2Math.AddMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2))
    C

  b2Math.MulMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2))
    C

  b2Math.MulTMM = (A, B) ->
    c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1))
    c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2))
    C = b2Mat22.FromVV(c1, c2)
    C

  b2Math.Abs = (a) ->
    a = 0  if a is `undefined`
    (if a > 0.0 then a else (-a))

  b2Math.AbsV = (a) ->
    b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y))
    b

  b2Math.AbsM = (A) ->
    B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2))
    B

  b2Math.Min = (a, b) ->
    a = 0  if a is `undefined`
    b = 0  if b is `undefined`
    (if a < b then a else b)

  b2Math.MinV = (a, b) ->
    c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y))
    c

  b2Math.Max = (a, b) ->
    a = 0  if a is `undefined`
    b = 0  if b is `undefined`
    (if a > b then a else b)

  b2Math.MaxV = (a, b) ->
    c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y))
    c

  b2Math.Clamp = (a, low, high) ->
    a = 0  if a is `undefined`
    low = 0  if low is `undefined`
    high = 0  if high is `undefined`
    (if a < low then low else (if a > high then high else a))

  b2Math.ClampV = (a, low, high) ->
    b2Math.MaxV low, b2Math.MinV(a, high)

  b2Math.Swap = (a, b) ->
    tmp = a[0]
    a[0] = b[0]
    b[0] = tmp
    return

  b2Math.Random = ->
    Math.random() * 2 - 1

  b2Math.RandomRange = (lo, hi) ->
    lo = 0  if lo is `undefined`
    hi = 0  if hi is `undefined`
    r = Math.random()
    r = (hi - lo) * r + lo
    r

  b2Math.NextPowerOfTwo = (x) ->
    x = 0  if x is `undefined`
    x |= (x >> 1) & 0x7FFFFFFF
    x |= (x >> 2) & 0x3FFFFFFF
    x |= (x >> 4) & 0x0FFFFFFF
    x |= (x >> 8) & 0x00FFFFFF
    x |= (x >> 16) & 0x0000FFFF
    x + 1

  b2Math.IsPowerOfTwo = (x) ->
    x = 0  if x is `undefined`
    result = x > 0 and (x & (x - 1)) is 0
    result

  Box2D.postDefs.push ->
    Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0)
    Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0))
    Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity)
    return

  b2Sweep.b2Sweep = ->
    @localCenter = new b2Vec2()
    @c0 = new b2Vec2
    @c = new b2Vec2()
    return

  b2Sweep::Set = (other) ->
    @localCenter.SetV other.localCenter
    @c0.SetV other.c0
    @c.SetV other.c
    @a0 = other.a0
    @a = other.a
    @t0 = other.t0
    return

  b2Sweep::Copy = ->
    copy = new b2Sweep()
    copy.localCenter.SetV @localCenter
    copy.c0.SetV @c0
    copy.c.SetV @c
    copy.a0 = @a0
    copy.a = @a
    copy.t0 = @t0
    copy

  b2Sweep::GetTransform = (xf, alpha) ->
    alpha = 0  if alpha is `undefined`
    xf.position.x = (1.0 - alpha) * @c0.x + alpha * @c.x
    xf.position.y = (1.0 - alpha) * @c0.y + alpha * @c.y
    angle = (1.0 - alpha) * @a0 + alpha * @a
    xf.R.Set angle
    tMat = xf.R
    xf.position.x -= (tMat.col1.x * @localCenter.x + tMat.col2.x * @localCenter.y)
    xf.position.y -= (tMat.col1.y * @localCenter.x + tMat.col2.y * @localCenter.y)
    return

  b2Sweep::Advance = (t) ->
    t = 0  if t is `undefined`
    if @t0 < t and 1.0 - @t0 > Number.MIN_VALUE
      alpha = (t - @t0) / (1.0 - @t0)
      @c0.x = (1.0 - alpha) * @c0.x + alpha * @c.x
      @c0.y = (1.0 - alpha) * @c0.y + alpha * @c.y
      @a0 = (1.0 - alpha) * @a0 + alpha * @a
      @t0 = t
    return

  b2Transform.b2Transform = ->
    @position = new b2Vec2
    @R = new b2Mat22()
    return

  b2Transform::b2Transform = (pos, r) ->
    pos = null  if pos is `undefined`
    r = null  if r is `undefined`
    if pos
      @position.SetV pos
      @R.SetM r
    return

  b2Transform::Initialize = (pos, r) ->
    @position.SetV pos
    @R.SetM r
    return

  b2Transform::SetIdentity = ->
    @position.SetZero()
    @R.SetIdentity()
    return

  b2Transform::Set = (x) ->
    @position.SetV x.position
    @R.SetM x.R
    return

  b2Transform::GetAngle = ->
    Math.atan2 @R.col1.y, @R.col1.x

  b2Vec2.b2Vec2 = ->

  b2Vec2::b2Vec2 = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    @x = x_
    @y = y_
    return

  b2Vec2::SetZero = ->
    @x = 0.0
    @y = 0.0
    return

  b2Vec2::Set = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    @x = x_
    @y = y_
    return

  b2Vec2::SetV = (v) ->
    @x = v.x
    @y = v.y
    return

  b2Vec2::GetNegative = ->
    new b2Vec2((-@x), (-@y))

  b2Vec2::NegativeSelf = ->
    @x = (-@x)
    @y = (-@y)
    return

  b2Vec2.Make = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    new b2Vec2(x_, y_)

  b2Vec2::Copy = ->
    new b2Vec2(@x, @y)

  b2Vec2::Add = (v) ->
    @x += v.x
    @y += v.y
    return

  b2Vec2::Subtract = (v) ->
    @x -= v.x
    @y -= v.y
    return

  b2Vec2::Multiply = (a) ->
    a = 0  if a is `undefined`
    @x *= a
    @y *= a
    return

  b2Vec2::MulM = (A) ->
    tX = @x
    @x = A.col1.x * tX + A.col2.x * @y
    @y = A.col1.y * tX + A.col2.y * @y
    return

  b2Vec2::MulTM = (A) ->
    tX = b2Math.Dot(this, A.col1)
    @y = b2Math.Dot(this, A.col2)
    @x = tX
    return

  b2Vec2::CrossVF = (s) ->
    s = 0  if s is `undefined`
    tX = @x
    @x = s * @y
    @y = (-s * tX)
    return

  b2Vec2::CrossFV = (s) ->
    s = 0  if s is `undefined`
    tX = @x
    @x = (-s * @y)
    @y = s * tX
    return

  b2Vec2::MinV = (b) ->
    @x = (if @x < b.x then @x else b.x)
    @y = (if @y < b.y then @y else b.y)
    return

  b2Vec2::MaxV = (b) ->
    @x = (if @x > b.x then @x else b.x)
    @y = (if @y > b.y then @y else b.y)
    return

  b2Vec2::Abs = ->
    @x = (-@x)  if @x < 0
    @y = (-@y)  if @y < 0
    return

  b2Vec2::Length = ->
    Math.sqrt @x * @x + @y * @y

  b2Vec2::LengthSquared = ->
    @x * @x + @y * @y

  b2Vec2::Normalize = ->
    length = Math.sqrt(@x * @x + @y * @y)
    return 0.0  if length < Number.MIN_VALUE
    invLength = 1.0 / length
    @x *= invLength
    @y *= invLength
    length

  b2Vec2::IsValid = ->
    b2Math.IsValid(@x) and b2Math.IsValid(@y)

  b2Vec3.b2Vec3 = ->

  b2Vec3::b2Vec3 = (x, y, z) ->
    x = 0  if x is `undefined`
    y = 0  if y is `undefined`
    z = 0  if z is `undefined`
    @x = x
    @y = y
    @z = z
    return

  b2Vec3::SetZero = ->
    @x = @y = @z = 0.0
    return

  b2Vec3::Set = (x, y, z) ->
    x = 0  if x is `undefined`
    y = 0  if y is `undefined`
    z = 0  if z is `undefined`
    @x = x
    @y = y
    @z = z
    return

  b2Vec3::SetV = (v) ->
    @x = v.x
    @y = v.y
    @z = v.z
    return

  b2Vec3::GetNegative = ->
    new b2Vec3((-@x), (-@y), (-@z))

  b2Vec3::NegativeSelf = ->
    @x = (-@x)
    @y = (-@y)
    @z = (-@z)
    return

  b2Vec3::Copy = ->
    new b2Vec3(@x, @y, @z)

  b2Vec3::Add = (v) ->
    @x += v.x
    @y += v.y
    @z += v.z
    return

  b2Vec3::Subtract = (v) ->
    @x -= v.x
    @y -= v.y
    @z -= v.z
    return

  b2Vec3::Multiply = (a) ->
    a = 0  if a is `undefined`
    @x *= a
    @y *= a
    @z *= a
    return

  return
)()
