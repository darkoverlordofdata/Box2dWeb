Box2D = require('../index')

class Box2D.Dynamics.b2TimeStep

  dt: 0
  inv_dt: 0
  positionIterations: 0
  velocityIterations: 0
  warmStarting: 0


  Set: (step) ->
    @dt = step.dt
    @inv_dt = step.inv_dt
    @positionIterations = step.positionIterations
    @velocityIterations = step.velocityIterations
    @warmStarting = step.warmStarting
    return



