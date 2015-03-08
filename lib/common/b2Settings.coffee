Box2D = require('../index')


class Box2D.Common.b2Settings

  @VERSION = "2.1alpha"
  @USHRT_MAX = 0x0000ffff
  @b2_pi = Math.PI
  @b2_maxManifoldPoints = 2
  @b2_aabbExtension = 0.1
  @b2_aabbMultiplier = 2.0
  @b2_polygonRadius = 2.0 * b2Settings.b2_linearSlop
  @b2_linearSlop = 0.005
  @b2_angularSlop = 2.0 / 180.0 * b2Settings.b2_pi
  @b2_toiSlop = 8.0 * b2Settings.b2_linearSlop
  @b2_maxTOIContactsPerIsland = 32
  @b2_maxTOIJointsPerIsland = 32
  @b2_velocityThreshold = 1.0
  @b2_maxLinearCorrection = 0.2
  @b2_maxAngularCorrection = 8.0 / 180.0 * b2Settings.b2_pi
  @b2_maxTranslation = 2.0
  @b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation
  @b2_maxRotation = 0.5 * b2Settings.b2_pi
  @b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation
  @b2_contactBaumgarte = 0.2
  @b2_timeToSleep = 0.5
  @b2_linearSleepTolerance = 0.01
  @b2_angularSleepTolerance = 2.0 / 180.0 * b2Settings.b2_pi

  @b2MixFriction: (friction1, friction2) ->
    friction1 = 0  if friction1 is undefined
    friction2 = 0  if friction2 is undefined
    return Math.sqrt(friction1 * friction2)

  @b2MixRestitution: (restitution1, restitution2) ->
    restitution1 = 0  if restitution1 is undefined
    restitution2 = 0  if restitution2 is undefined
    return restitution1 > restitution2 ? restitution1 : restitution2

  @b2Assert: (a) ->
    throw "Assertion Failed" unless a