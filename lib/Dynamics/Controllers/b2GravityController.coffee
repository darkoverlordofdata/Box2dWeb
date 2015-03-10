class Box2D.Dynamics.Controllers.b2GravityController extends b2Controller

  G             : 1
  invSqr        : true

  constructor: ->
    super
    @G = 1
    @invSqr = true
    return

  Step: (step) ->
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

