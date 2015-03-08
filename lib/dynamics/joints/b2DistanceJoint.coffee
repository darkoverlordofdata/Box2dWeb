Box2D = require('../../index')

b2Joint       = Box2D.Dynamics.Joints.b2Joint
b2Vec2        = Box2D.Common.Math.b2Vec2

class Box2D.Dynamics.Joints.b2DistanceJoint extends b2Joint

  m_u                     : null
  m_length                : 0
  m_frequencyHz           : 0
  m_dampingRatio          : 0
  m_impulse               : 0.0
  m_gamma                 : 0.0
  m_bias                  : 0.0

  constructor: (def) ->
    super def
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u = new b2Vec2()
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_length = def.length
    @m_frequencyHz = def.frequencyHz
    @m_dampingRatio = def.dampingRatio
    return


  GetAnchorA: ->
    return @m_bodyA.GetWorldPoint @m_localAnchor1

  GetAnchorB: ->
    return @m_bodyB.GetWorldPoint @m_localAnchor2

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return new b2Vec2(inv_dt * @m_impulse * @m_u.x, inv_dt * @m_impulse * @m_u.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    0.0

  GetLength: ->
    return @m_length

  SetLength: (length) ->
    length = 0  if length is undefined
    @m_length = length
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
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    @m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    @m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    length = Math.sqrt(@m_u.x * @m_u.x + @m_u.y * @m_u.y)
    if length > b2Settings.b2_linearSlop
      @m_u.Multiply 1.0 / length
    else
      @m_u.SetZero()
    cr1u = (r1X * @m_u.y - r1Y * @m_u.x)
    cr2u = (r2X * @m_u.y - r2Y * @m_u.x)
    invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u
    @m_mass = (if invMass isnt 0.0 then 1.0 / invMass else 0.0)
    if @m_frequencyHz > 0.0
      C = length - @m_length
      omega = 2.0 * Math.PI * @m_frequencyHz
      d = 2.0 * @m_mass * @m_dampingRatio * omega
      k = @m_mass * omega * omega
      @m_gamma = step.dt * (d + step.dt * k)
      @m_gamma = (if @m_gamma isnt 0.0 then 1 / @m_gamma else 0.0)
      @m_bias = C * step.dt * k * @m_gamma
      @m_mass = invMass + @m_gamma
      @m_mass = (if @m_mass isnt 0.0 then 1.0 / @m_mass else 0.0)
    if step.warmStarting
      @m_impulse *= step.dtRatio
      PX = @m_impulse * @m_u.x
      PY = @m_impulse * @m_u.y
      bA.m_linearVelocity.x -= bA.m_invMass * PX
      bA.m_linearVelocity.y -= bA.m_invMass * PY
      bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX)
      bB.m_linearVelocity.x += bB.m_invMass * PX
      bB.m_linearVelocity.y += bB.m_invMass * PY
      bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX)
    else
      @m_impulse = 0.0
    return

  SolveVelocityConstraints: (step) ->
    tMat = undefined
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
    v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
    v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
    v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
    Cdot = (@m_u.x * (v2X - v1X) + @m_u.y * (v2Y - v1Y))
    impulse = (-@m_mass * (Cdot + @m_bias + @m_gamma * @m_impulse))
    @m_impulse += impulse
    PX = impulse * @m_u.x
    PY = impulse * @m_u.y
    bA.m_linearVelocity.x -= bA.m_invMass * PX
    bA.m_linearVelocity.y -= bA.m_invMass * PY
    bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX)
    bB.m_linearVelocity.x += bB.m_invMass * PX
    bB.m_linearVelocity.y += bB.m_invMass * PY
    bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX)
    return

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
    tMat = undefined
    return true  if @m_frequencyHz > 0.0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    length = Math.sqrt(dX * dX + dY * dY)
    dX /= length
    dY /= length
    C = length - @m_length
    C = b2Math.Clamp(C, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
    impulse = (-@m_mass * C)
    @m_u.Set dX, dY
    PX = impulse * @m_u.x
    PY = impulse * @m_u.y
    bA.m_sweep.c.x -= bA.m_invMass * PX
    bA.m_sweep.c.y -= bA.m_invMass * PY
    bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX)
    bB.m_sweep.c.x += bB.m_invMass * PX
    bB.m_sweep.c.y += bB.m_invMass * PY
    bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    b2Math.Abs(C) < b2Settings.b2_linearSlop
