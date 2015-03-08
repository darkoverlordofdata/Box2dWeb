(->
  b2Body = Box2D.Dynamics.b2Body
  b2BodyDef = Box2D.Dynamics.b2BodyDef
  b2ContactFilter = Box2D.Dynamics.b2ContactFilter
  b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse
  b2ContactListener = Box2D.Dynamics.b2ContactListener
  b2ContactManager = Box2D.Dynamics.b2ContactManager
  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
  b2DestructionListener = Box2D.Dynamics.b2DestructionListener
  b2FilterData = Box2D.Dynamics.b2FilterData
  b2Fixture = Box2D.Dynamics.b2Fixture
  b2FixtureDef = Box2D.Dynamics.b2FixtureDef
  b2Island = Box2D.Dynamics.b2Island
  b2TimeStep = Box2D.Dynamics.b2TimeStep
  b2World = Box2D.Dynamics.b2World
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController
  b2ConstantAccelController = Box2D.Dynamics.Controllers.b2ConstantAccelController
  b2ConstantForceController = Box2D.Dynamics.Controllers.b2ConstantForceController
  b2Controller = Box2D.Dynamics.Controllers.b2Controller
  b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge
  b2GravityController = Box2D.Dynamics.Controllers.b2GravityController
  b2TensorDampingController = Box2D.Dynamics.Controllers.b2TensorDampingController
  Box2D.inherit b2BuoyancyController, Box2D.Dynamics.Controllers.b2Controller
  b2BuoyancyController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2BuoyancyController.b2BuoyancyController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @normal = new b2Vec2(0, (-1))
    @offset = 0
    @density = 0
    @velocity = new b2Vec2(0, 0)
    @linearDrag = 2
    @angularDrag = 1
    @useDensity = false
    @useWorldGravity = true
    @gravity = null
    return

  b2BuoyancyController::Step = (step) ->
    return  unless @m_bodyList
    @gravity = @GetWorld().GetGravity().Copy()  if @useWorldGravity
    i = @m_bodyList

    while i
      body = i.body
      continue  if body.IsAwake() is false
      areac = new b2Vec2()
      massc = new b2Vec2()
      area = 0.0
      mass = 0.0
      fixture = body.GetFixtureList()

      while fixture
        sc = new b2Vec2()
        sarea = fixture.GetShape().ComputeSubmergedArea(@normal, @offset, body.GetTransform(), sc)
        area += sarea
        areac.x += sarea * sc.x
        areac.y += sarea * sc.y
        shapeDensity = 0
        if @useDensity
          shapeDensity = 1
        else
          shapeDensity = 1
        mass += sarea * shapeDensity
        massc.x += sarea * sc.x * shapeDensity
        massc.y += sarea * sc.y * shapeDensity
        fixture = fixture.GetNext()
      areac.x /= area
      areac.y /= area
      massc.x /= mass
      massc.y /= mass
      continue  if area < Number.MIN_VALUE
      buoyancyForce = @gravity.GetNegative()
      buoyancyForce.Multiply @density * area
      body.ApplyForce buoyancyForce, massc
      dragForce = body.GetLinearVelocityFromWorldPoint(areac)
      dragForce.Subtract @velocity
      dragForce.Multiply (-@linearDrag * area)
      body.ApplyForce dragForce, areac
      body.ApplyTorque (-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * @angularDrag)
      i = i.nextBody
    return

  b2BuoyancyController::Draw = (debugDraw) ->
    r = 1000
    p1 = new b2Vec2()
    p2 = new b2Vec2()
    p1.x = @normal.x * @offset + @normal.y * r
    p1.y = @normal.y * @offset - @normal.x * r
    p2.x = @normal.x * @offset - @normal.y * r
    p2.y = @normal.y * @offset + @normal.x * r
    color = new b2Color(0, 0, 1)
    debugDraw.DrawSegment p1, p2, color
    return

  Box2D.inherit b2ConstantAccelController, Box2D.Dynamics.Controllers.b2Controller
  b2ConstantAccelController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2ConstantAccelController.b2ConstantAccelController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @A = new b2Vec2(0, 0)
    return

  b2ConstantAccelController::Step = (step) ->
    smallA = new b2Vec2(@A.x * step.dt, @A.y * step.dt)
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.SetLinearVelocity new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y)
      i = i.nextBody
    return

  Box2D.inherit b2ConstantForceController, Box2D.Dynamics.Controllers.b2Controller
  b2ConstantForceController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2ConstantForceController.b2ConstantForceController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @F = new b2Vec2(0, 0)
    return

  b2ConstantForceController::Step = (step) ->
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.ApplyForce @F, body.GetWorldCenter()
      i = i.nextBody
    return

  b2Controller.b2Controller = ->

  b2Controller::Step = (step) ->

  b2Controller::Draw = (debugDraw) ->

  b2Controller::AddBody = (body) ->
    edge = new b2ControllerEdge()
    edge.controller = this
    edge.body = body
    edge.nextBody = @m_bodyList
    edge.prevBody = null
    @m_bodyList = edge
    edge.nextBody.prevBody = edge  if edge.nextBody
    @m_bodyCount++
    edge.nextController = body.m_controllerList
    edge.prevController = null
    body.m_controllerList = edge
    edge.nextController.prevController = edge  if edge.nextController
    body.m_controllerCount++
    return

  b2Controller::RemoveBody = (body) ->
    edge = body.m_controllerList
    edge = edge.nextController  while edge and edge.controller isnt this
    edge.prevBody.nextBody = edge.nextBody  if edge.prevBody
    edge.nextBody.prevBody = edge.prevBody  if edge.nextBody
    edge.nextController.prevController = edge.prevController  if edge.nextController
    edge.prevController.nextController = edge.nextController  if edge.prevController
    @m_bodyList = edge.nextBody  if @m_bodyList is edge
    body.m_controllerList = edge.nextController  if body.m_controllerList is edge
    body.m_controllerCount--
    @m_bodyCount--
    return

  b2Controller::Clear = ->
    @RemoveBody @m_bodyList.body  while @m_bodyList
    return

  b2Controller::GetNext = ->
    @m_next

  b2Controller::GetWorld = ->
    @m_world

  b2Controller::GetBodyList = ->
    @m_bodyList

  b2ControllerEdge.b2ControllerEdge = ->

  Box2D.inherit b2GravityController, Box2D.Dynamics.Controllers.b2Controller
  b2GravityController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2GravityController.b2GravityController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @G = 1
    @invSqr = true
    return

  b2GravityController::Step = (step) ->
    i = null
    body1 = null
    p1 = null
    mass1 = 0
    j = null
    body2 = null
    p2 = null
    dx = 0
    dy = 0
    r2 = 0
    f = null
    if @invSqr
      i = @m_bodyList
      while i
        body1 = i.body
        p1 = body1.GetWorldCenter()
        mass1 = body1.GetMass()
        j = @m_bodyList
        while j isnt i
          body2 = j.body
          p2 = body2.GetWorldCenter()
          dx = p2.x - p1.x
          dy = p2.y - p1.y
          r2 = dx * dx + dy * dy
          continue  if r2 < Number.MIN_VALUE
          f = new b2Vec2(dx, dy)
          f.Multiply @G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass()
          body1.ApplyForce f, p1  if body1.IsAwake()
          f.Multiply (-1)
          body2.ApplyForce f, p2  if body2.IsAwake()
          j = j.nextBody
        i = i.nextBody
    else
      i = @m_bodyList
      while i
        body1 = i.body
        p1 = body1.GetWorldCenter()
        mass1 = body1.GetMass()
        j = @m_bodyList
        while j isnt i
          body2 = j.body
          p2 = body2.GetWorldCenter()
          dx = p2.x - p1.x
          dy = p2.y - p1.y
          r2 = dx * dx + dy * dy
          continue  if r2 < Number.MIN_VALUE
          f = new b2Vec2(dx, dy)
          f.Multiply @G / r2 * mass1 * body2.GetMass()
          body1.ApplyForce f, p1  if body1.IsAwake()
          f.Multiply (-1)
          body2.ApplyForce f, p2  if body2.IsAwake()
          j = j.nextBody
        i = i.nextBody
    return

  Box2D.inherit b2TensorDampingController, Box2D.Dynamics.Controllers.b2Controller
  b2TensorDampingController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2TensorDampingController.b2TensorDampingController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @T = new b2Mat22()
    @maxTimestep = 0
    return

  b2TensorDampingController::SetAxisAligned = (xDamping, yDamping) ->
    xDamping = 0  if xDamping is `undefined`
    yDamping = 0  if yDamping is `undefined`
    @T.col1.x = (-xDamping)
    @T.col1.y = 0
    @T.col2.x = 0
    @T.col2.y = (-yDamping)
    if xDamping > 0 or yDamping > 0
      @maxTimestep = 1 / Math.max(xDamping, yDamping)
    else
      @maxTimestep = 0
    return

  b2TensorDampingController::Step = (step) ->
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

  return
)()
