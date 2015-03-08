Box2D = require('../index')

b2Controller  = Box2D.Dynamic.Contacts.b2ContactID
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22

class Box2D.Dynamics.Contacts.b2ContactResult

  position: null
  normal: null
  id: null

  constructor: ->
    @position = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return
