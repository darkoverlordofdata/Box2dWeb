class Box2D.Dynamics.Contacts.b2ContactConstraintPoint

  localPoint  : null
  rA          : null
  rB          : null

  constructor: ->
    @localPoint = new b2Vec2()
    @rA = new b2Vec2()
    @rB = new b2Vec2()
    return
