class Box2D.Dynamics.b2ContactImpulse

  normalImpulses:   null
  tangentImpulses:  null
  
  @b2ContactImpulse = ->
    @normalImpulses = new Vector(b2Settings.b2_maxManifoldPoints)
    @tangentImpulses = new Vector(b2Settings.b2_maxManifoldPoints)
    return
