class Box2D.Dynamics.Contacts.b2ContactResult

  position    : null
  normal      : null
  id          : null

  constructor: ->
    @position = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return
