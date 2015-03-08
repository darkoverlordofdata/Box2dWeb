Box2D = require('../index')

class Box2D.Dynamics.b2ContactListener


  BeginContact: (contact) -> #contact
    # NOTE: Only this one is called at the moment

  EndContact: (contact) -> #contact

  PreSolve: (contact, oldManifold) -> #contact, oldManifold

  PostSolve: (contact, impulse) -> #contact, impulse

  @b2_defaultListener =  new b2ContactListener()
