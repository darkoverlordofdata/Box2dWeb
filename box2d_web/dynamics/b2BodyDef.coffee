Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2
b2Body      = Box2D.Dynamics.b2Body

class Box2D.Dynamics.b2BodyDef

  position            : null
  linearVelocity      : null
  userData            : null
  angle               : 0.0
  linearVelocity      : null
  angularVelocity     : 0.0
  angularDamping      : 0.0
  allowSleep          : true
  awake               : true
  fixedRotation       : false
  bullet              : false
  type                : 0
  active              : true
  inertiaScale        : 1.0
  
  constructor: ->
    @position = new b2Vec2(0, 0)
    @linearVelocity = new b2Vec2()
    @userData = null
    @angle = 0.0
    @linearVelocity.Set 0, 0
    @angularVelocity = 0.0
    @linearDamping = 0.0
    @angularDamping = 0.0
    @allowSleep = true
    @awake = true
    @fixedRotation = false
    @bullet = false
    @type = b2Body.b2_staticBody
    @active = true
    @inertiaScale = 1.0


