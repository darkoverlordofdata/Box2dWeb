Box2D = require('../index')

b2Controller  = Box2D.Dynamics.Controllers.b2Controller
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Controllers.b2BuoyancyController extends b2Controller

  normal            : null
  offset            : 0
  density           : 0
  velocity          : null
  linearDrag        : 2
  angularDrag       : 1
  useDensity        : false
  useWorldGravity   : true
  gravity           : null

  constructor: ->
    super
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

  Step: (step) ->
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

  Draw: (debugDraw) ->
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
 