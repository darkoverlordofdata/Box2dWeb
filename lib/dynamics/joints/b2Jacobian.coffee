Box2D = require('../../index')


class Box2D.Dynamics.Joints.b2Jacobian

  constructor: ->
    @linearA = new b2Vec2()
    @linearB = new b2Vec2()
    return

  SetZero: ->
    @linearA.SetZero()
    @angularA = 0.0
    @linearB.SetZero()
    @angularB = 0.0
    return

  Set: (x1, a1, x2, a2) ->
    a1 = 0  if a1 is undefined
    a2 = 0  if a2 is undefined
    @linearA.SetV x1
    @angularA = a1
    @linearB.SetV x2
    @angularB = a2
    return

  Compute: (x1, a1, x2, a2) ->
    a1 = 0  if a1 is undefined
    a2 = 0  if a2 is undefined
    (@linearA.x * x1.x + @linearA.y * x1.y) + @angularA * a1 + (@linearB.x * x2.x + @linearB.y * x2.y) + @angularB * a2
