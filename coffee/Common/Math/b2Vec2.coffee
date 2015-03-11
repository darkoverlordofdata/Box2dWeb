class Box2D.Common.Math.b2Vec2
  
  x: 0
  y: 0
  
  constructor: (x_, y_) ->
    x_ = 0  if x_ is undefined
    y_ = 0  if y_ is undefined
    @x = x_
    @y = y_
    return

  SetZero: ->
    @x = 0.0
    @y = 0.0
    return

  Set: (x_, y_) ->
    x_ = 0  if x_ is undefined
    y_ = 0  if y_ is undefined
    @x = x_
    @y = y_
    return

  SetV: (v) ->
    @x = v.x
    @y = v.y
    return

  GetNegative: ->
    new b2Vec2((-@x), (-@y))

  NegativeSelf: ->
    @x = (-@x)
    @y = (-@y)
    return

  @Make: (x_, y_) ->
    x_ = 0  if x_ is undefined
    y_ = 0  if y_ is undefined
    return new b2Vec2(x_, y_)

  Copy: ->
    return new b2Vec2(@x, @y)

  Add: (v) ->
    @x += v.x
    @y += v.y
    return

  Subtract: (v) ->
    @x -= v.x
    @y -= v.y
    return

  Multiply: (a) ->
    a = 0  if a is undefined
    @x *= a
    @y *= a
    return

  MulM: (A) ->
    tX = @x
    @x = A.col1.x * tX + A.col2.x * @y
    @y = A.col1.y * tX + A.col2.y * @y
    return

  MulTM: (A) ->
    tX = b2Math.Dot(this, A.col1)
    @y = b2Math.Dot(this, A.col2)
    @x = tX
    return

  CrossVF: (s) ->
    s = 0  if s is undefined
    tX = @x
    @x = s * @y
    @y = (-s * tX)
    return

  CrossFV: (s) ->
    s = 0  if s is undefined
    tX = @x
    @x = (-s * @y)
    @y = s * tX
    return

  MinV: (b) ->
    @x = (if @x < b.x then @x else b.x)
    @y = (if @y < b.y then @y else b.y)
    return

  MaxV: (b) ->
    @x = (if @x > b.x then @x else b.x)
    @y = (if @y > b.y then @y else b.y)
    return

  Abs: ->
    @x = (-@x)  if @x < 0
    @y = (-@y)  if @y < 0
    return

  Length: ->
    return Math.sqrt @x * @x + @y * @y

  LengthSquared: ->
    return @x * @x + @y * @y

  Normalize: ->
    length = Math.sqrt(@x * @x + @y * @y)
    return 0.0  if length < Number.MIN_VALUE
    invLength = 1.0 / length
    @x *= invLength
    @y *= invLength
    return length

  IsValid: ->
    return b2Math.IsValid(@x) and b2Math.IsValid(@y)
