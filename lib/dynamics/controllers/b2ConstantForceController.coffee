Box2D = require('../../index')

b2Controller  = Box2D.Dynamics.Controllers.b2Controller
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Controllers.b2ConstantForceController extends b2Controller

  F: null
  
  constructor: ->
    super
    @F = new b2Vec2(0, 0)
    return

  Step: (step) ->
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.ApplyForce @F, body.GetWorldCenter()
      i = i.nextBody
    return

