Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2
b2Body      = Box2D.Dynamics.b2Body

Vector_a2j_Number = Box2D.NVector

class Box2D.Dynamics.b2ContactImpulse

  @b2ContactImpulse = ->
    @normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints)
    @tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints)
    return
