Box2D = require('../index')

b2Math = Box2D.Common.Math.b2Math
parseUInt = (v) -> Math.abs(parseInt(v))


class Box2D.Common.b2Color

  _r: 0
  _g: 0
  _b: 0

  constructor: (rr=0, gg=0, bb=0) ->
    @_r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))

  Set: (rr=0, gg=0, bb=0) ->
    @_r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
    return

  Object.defineProperties b2Color::,
    r:
      enumerable: false
      configurable: true
      get: -> @_r
      set: (rr=0) ->
        @_r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))


    g:
      enumerable: false
      configurable: true
      get: -> @_g
      set: (gg=0) ->
        @_g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))


    b:
      enumerable: false
      configurable: true
      get: -> @_b
      set: (bb=0) ->
        @_b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))


    color:
      enumerable: false
      configurable: true
      get: () -> (@_r << 16) | (@_g << 8) | (@_b)


