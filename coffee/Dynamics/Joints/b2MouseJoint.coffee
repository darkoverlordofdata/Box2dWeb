class Box2D.Dynamics.Joints.b2MouseJoint extends b2Joint

  K               : null
  K1              : null
  K2              : null
  m_localAnchor   : null
  m_target        : null
  m_impulse       : null
  m_mass          : null
  m_C             : null
  m_frequencyHz   : 0.0
  m_dampingRatio  : 0.0
  m_beta          : 0.0
  m_gamma         : 0.0


  constructor: (def) ->
    super def
    @K = new b2Mat22()
    @K1 = new b2Mat22()
    @K2 = new b2Mat22()
    @m_localAnchor = new b2Vec2()
    @m_target = new b2Vec2()
    @m_impulse = new b2Vec2()
    @m_mass = new b2Mat22()
    @m_C = new b2Vec2()
    @m_target.SetV def.target
    tX = @m_target.x - @m_bodyB.m_xf.position.x
    tY = @m_target.y - @m_bodyB.m_xf.position.y
    tMat = @m_bodyB.m_xf.R
    @m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y)
    @m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y)
    @m_maxForce = def.maxForce
    @m_impulse.SetZero()
    @m_frequencyHz = def.frequencyHz
    @m_dampingRatio = def.dampingRatio
    return

  GetAnchorA: ->
    return @m_target

  GetAnchorB: ->
    return @m_bodyB.GetWorldPoint @m_localAnchor

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return 0.0

  GetTarget: ->
    return @m_target

  SetTarget: (target) ->
    @m_bodyB.SetAwake true  if @m_bodyB.IsAwake() is false
    @m_target = target
    return

  GetMaxForce: ->
    return @m_maxForce

  SetMaxForce: (maxForce) ->
    maxForce = 0  if maxForce is undefined
    @m_maxForce = maxForce
    return

  GetFrequency: ->
    return @m_frequencyHz

  SetFrequency: (hz) ->
    hz = 0  if hz is undefined
    @m_frequencyHz = hz
    return

  GetDampingRatio: ->
    return @m_dampingRatio

  SetDampingRatio: (ratio) ->
    ratio = 0  if ratio is undefined
    @m_dampingRatio = ratio
    return


  InitVelocityConstraints: (step) ->
    b = @m_bodyB
    mass = b.GetMass()
    omega = 2.0 * Math.PI * @m_frequencyHz
    d = 2.0 * mass * @m_dampingRatio * omega
    k = mass * omega * omega
    @m_gamma = step.dt * (d + step.dt * k)
    @m_gamma = (if @m_gamma isnt 0 then 1 / @m_gamma else 0.0)
    @m_beta = step.dt * k * @m_gamma
    tMat = undefined
    tMat = b.m_xf.R
    rX = @m_localAnchor.x - b.m_sweep.localCenter.x
    rY = @m_localAnchor.y - b.m_sweep.localCenter.y
    tX = (tMat.col1.x * rX + tMat.col2.x * rY)
    rY = (tMat.col1.y * rX + tMat.col2.y * rY)
    rX = tX
    invMass = b.m_invMass
    invI = b.m_invI
    @K1.col1.x = invMass
    @K1.col2.x = 0.0
    @K1.col1.y = 0.0
    @K1.col2.y = invMass
    @K2.col1.x = invI * rY * rY
    @K2.col2.x = (-invI * rX * rY)
    @K2.col1.y = (-invI * rX * rY)
    @K2.col2.y = invI * rX * rX
    @K.SetM @K1
    @K.AddM @K2
    @K.col1.x += @m_gamma
    @K.col2.y += @m_gamma
    @K.GetInverse @m_mass
    @m_C.x = b.m_sweep.c.x + rX - @m_target.x
    @m_C.y = b.m_sweep.c.y + rY - @m_target.y
    b.m_angularVelocity *= 0.98
    @m_impulse.x *= step.dtRatio
    @m_impulse.y *= step.dtRatio
    b.m_linearVelocity.x += invMass * @m_impulse.x
    b.m_linearVelocity.y += invMass * @m_impulse.y
    b.m_angularVelocity += invI * (rX * @m_impulse.y - rY * @m_impulse.x)
    return

  SolveVelocityConstraints: (step) ->
    b = @m_bodyB
    tMat = undefined
    tX = 0
    tY = 0
    tMat = b.m_xf.R
    rX = @m_localAnchor.x - b.m_sweep.localCenter.x
    rY = @m_localAnchor.y - b.m_sweep.localCenter.y
    tX = (tMat.col1.x * rX + tMat.col2.x * rY)
    rY = (tMat.col1.y * rX + tMat.col2.y * rY)
    rX = tX
    CdotX = b.m_linearVelocity.x + (-b.m_angularVelocity * rY)
    CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX)
    tMat = @m_mass
    tX = CdotX + @m_beta * @m_C.x + @m_gamma * @m_impulse.x
    tY = CdotY + @m_beta * @m_C.y + @m_gamma * @m_impulse.y
    impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY))
    impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY))
    oldImpulseX = @m_impulse.x
    oldImpulseY = @m_impulse.y
    @m_impulse.x += impulseX
    @m_impulse.y += impulseY
    maxImpulse = step.dt * @m_maxForce
    @m_impulse.Multiply maxImpulse / @m_impulse.Length()  if @m_impulse.LengthSquared() > maxImpulse * maxImpulse
    impulseX = @m_impulse.x - oldImpulseX
    impulseY = @m_impulse.y - oldImpulseY
    b.m_linearVelocity.x += b.m_invMass * impulseX
    b.m_linearVelocity.y += b.m_invMass * impulseY
    b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX)
    return

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
    return true

