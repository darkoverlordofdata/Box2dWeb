Box2D = require('../../index')

class Box2D.Dynamics.b2ContactListener


  BeginContact: -> #contact
    # NOTE: Only this one is called at the moment

  EndContact: -> #contact

  PreSolve: -> #contact, oldManifold

  PostSolve: -> #contact, impulse

  @b2_defaultListener =  new b2ContactListener()
