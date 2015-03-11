class Box2D.Dynamics.b2BodyDef

  position            : null
  linearVelocity      : null
  userData            : null
  angle               : 0.0
  linearVelocity      : null
  angularVelocity     : 0.0
  linearDamping       : 0.0
  angularDamping      : 0.0
  allowSleep          : true
  awake               : true
  fixedRotation       : false
  bullet              : false
  type                : b2Body.b2_staticBody
  active              : true
  inertiaScale        : 1.0

  constructor: ->
    @position = new b2Vec2()
    @position.Set 0.0, 0.0
    @linearVelocity = new b2Vec2()
    @linearVelocity.Set 0, 0
    return