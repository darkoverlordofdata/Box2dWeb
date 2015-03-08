Box2D = require('../../index')

b2Controller  = Box2D.Dynamics.Controllers.b2Controller
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Controllers.b2ConstantAccelController extends b2Controller

  A: null

  constructor: ->
    super
    @A = new b2Vec2(0, 0)
    return

  Step: (step) ->
    smallA = new b2Vec2(@A.x * step.dt, @A.y * step.dt)
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.SetLinearVelocity new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y)
      i = i.nextBody
    return


