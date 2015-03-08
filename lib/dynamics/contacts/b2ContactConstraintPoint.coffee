Box2D = require('../index')

b2Controller  = Box2D.Common.b2Settings
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22

class Box2D.Dynamics.Contacts.b2ContactConstraintPoint

  localPoint  : null
  rA          : null
  rB          : null

  constructor: ->
    @localPoint = new b2Vec2()
    @rA = new b2Vec2()
    @rB = new b2Vec2()
    return
