Box2D = require('../../index')

b2Controller  = Box2D.Dynamics.Controllers.b2Controller
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22

class Box2D.Dynamics.Controllers.b2TensorDampingController extends b2Controller

  T             : null
  maxTimestep   : 0

  constructor: ->
    super
    @T = new b2Mat22()
    @maxTimestep = 0
    return

  SetAxisAligned: (xDamping, yDamping) ->
    xDamping = 0  if xDamping is undefined
    yDamping = 0  if yDamping is undefined
    @T.col1.x = (-xDamping)
    @T.col1.y = 0
    @T.col2.x = 0
    @T.col2.y = (-yDamping)
    if xDamping > 0 or yDamping > 0
      @maxTimestep = 1 / Math.max(xDamping, yDamping)
    else
      @maxTimestep = 0
    return

  Step: (step) ->
    timestep = step.dt
    return  if timestep <= Number.MIN_VALUE
    timestep = @maxTimestep  if timestep > @maxTimestep and @maxTimestep > 0
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      damping = body.GetWorldVector(b2Math.MulMV(@T, body.GetLocalVector(body.GetLinearVelocity())))
      body.SetLinearVelocity new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep)
      i = i.nextBody
    return

