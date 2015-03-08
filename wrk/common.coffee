(->
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
  b2Color.b2Color = ->
    @_r = 0
    @_g = 0
    @_b = 0
    return

  b2Color::b2Color = (rr, gg, bb) ->
    rr = 0  if rr is `undefined`
    gg = 0  if gg is `undefined`
    bb = 0  if bb is `undefined`
    @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
    return

  b2Color::Set = (rr, gg, bb) ->
    rr = 0  if rr is `undefined`
    gg = 0  if gg is `undefined`
    bb = 0  if bb is `undefined`
    @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
    return

  Object.defineProperty b2Color::, "r",
    enumerable: false
    configurable: true
    set: (rr) ->
      rr = 0  if rr is `undefined`
      @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "g",
    enumerable: false
    configurable: true
    set: (gg) ->
      gg = 0  if gg is `undefined`
      @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "b",
    enumerable: false
    configurable: true
    set: (bb) ->
      bb = 0  if bb is `undefined`
      @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "color",
    enumerable: false
    configurable: true
    get: ->
      (@_r << 16) | (@_g << 8) | (@_b)

  b2Settings.b2Settings = ->

  b2Settings.b2MixFriction = (friction1, friction2) ->
    friction1 = 0  if friction1 is `undefined`
    friction2 = 0  if friction2 is `undefined`
    Math.sqrt friction1 * friction2

  b2Settings.b2MixRestitution = (restitution1, restitution2) ->
    restitution1 = 0  if restitution1 is `undefined`
    restitution2 = 0  if restitution2 is `undefined`
    (if restitution1 > restitution2 then restitution1 else restitution2)

  b2Settings.b2Assert = (a) ->
    throw "Assertion Failed"  unless a
    return

  Box2D.postDefs.push ->
    Box2D.Common.b2Settings.VERSION = "2.1alpha"
    Box2D.Common.b2Settings.USHRT_MAX = 0x0000ffff
    Box2D.Common.b2Settings.b2_pi = Math.PI
    Box2D.Common.b2Settings.b2_maxManifoldPoints = 2
    Box2D.Common.b2Settings.b2_aabbExtension = 0.1
    Box2D.Common.b2Settings.b2_aabbMultiplier = 2.0
    Box2D.Common.b2Settings.b2_polygonRadius = 2.0 * b2Settings.b2_linearSlop
    Box2D.Common.b2Settings.b2_linearSlop = 0.005
    Box2D.Common.b2Settings.b2_angularSlop = 2.0 / 180.0 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_toiSlop = 8.0 * b2Settings.b2_linearSlop
    Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland = 32
    Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland = 32
    Box2D.Common.b2Settings.b2_velocityThreshold = 1.0
    Box2D.Common.b2Settings.b2_maxLinearCorrection = 0.2
    Box2D.Common.b2Settings.b2_maxAngularCorrection = 8.0 / 180.0 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_maxTranslation = 2.0
    Box2D.Common.b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation
    Box2D.Common.b2Settings.b2_maxRotation = 0.5 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation
    Box2D.Common.b2Settings.b2_contactBaumgarte = 0.2
    Box2D.Common.b2Settings.b2_timeToSleep = 0.5
    Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.01
    Box2D.Common.b2Settings.b2_angularSleepTolerance = 2.0 / 180.0 * b2Settings.b2_pi
    return

  return
)()
