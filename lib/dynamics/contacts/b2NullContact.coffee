Box2D = require('../index')

b2Controller  = Box2D.Dynamics.Contacts.b2Contact
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Contacts.b2NullContact extends b2Contact

  constructor: ->
    super
    return

  Evaluate: ->

