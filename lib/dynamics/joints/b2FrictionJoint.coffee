Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint

class Box2D.Dynamics.Joints.b2FrictionJoint extends b2Joint

  constructor: (def) ->
    super def
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_linearMass = new b2Mat22()
    @m_linearImpulse = new b2Vec2()
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_linearMass.SetZero()
    @m_angularMass = 0.0
    @m_linearImpulse.SetZero()
    @m_angularImpulse = 0.0
    @m_maxForce = def.maxForce
    @m_maxTorque = def.maxTorque
    return

  GetAnchorA: ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  GetAnchorB: ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_linearImpulse.x, inv_dt * @m_linearImpulse.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_angularImpulse

  SetMaxForce: (force) ->
    force = 0  if force is `undefined`
    @m_maxForce = force
    return

  GetMaxForce: ->
    @m_maxForce

  SetMaxTorque: (torque) ->
    torque = 0  if torque is `undefined`
    @m_maxTorque = torque
    return

  GetMaxTorque: ->
    @m_maxTorque

  InitVelocityConstraints: (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    K = new b2Mat22()
    K.col1.x = mA + mB
    K.col2.x = 0.0
    K.col1.y = 0.0
    K.col2.y = mA + mB
    K.col1.x += iA * rAY * rAY
    K.col2.x += (-iA * rAX * rAY)
    K.col1.y += (-iA * rAX * rAY)
    K.col2.y += iA * rAX * rAX
    K.col1.x += iB * rBY * rBY
    K.col2.x += (-iB * rBX * rBY)
    K.col1.y += (-iB * rBX * rBY)
    K.col2.y += iB * rBX * rBX
    K.GetInverse @m_linearMass
    @m_angularMass = iA + iB
    @m_angularMass = 1.0 / @m_angularMass  if @m_angularMass > 0.0
    if step.warmStarting
      @m_linearImpulse.x *= step.dtRatio
      @m_linearImpulse.y *= step.dtRatio
      @m_angularImpulse *= step.dtRatio
      P = @m_linearImpulse
      bA.m_linearVelocity.x -= mA * P.x
      bA.m_linearVelocity.y -= mA * P.y
      bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + @m_angularImpulse)
      bB.m_linearVelocity.x += mB * P.x
      bB.m_linearVelocity.y += mB * P.y
      bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + @m_angularImpulse)
    else
      @m_linearImpulse.SetZero()
      @m_angularImpulse = 0.0
    return

  SolveVelocityConstraints: (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    vA = bA.m_linearVelocity
    wA = bA.m_angularVelocity
    vB = bB.m_linearVelocity
    wB = bB.m_angularVelocity
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    maxImpulse = 0
    Cdot = wB - wA
    impulse = (-@m_angularMass * Cdot)
    oldImpulse = @m_angularImpulse
    maxImpulse = step.dt * @m_maxTorque
    @m_angularImpulse = b2Math.Clamp(@m_angularImpulse + impulse, (-maxImpulse), maxImpulse)
    impulse = @m_angularImpulse - oldImpulse
    wA -= iA * impulse
    wB += iB * impulse
    CdotX = vB.x - wB * rBY - vA.x + wA * rAY
    CdotY = vB.y + wB * rBX - vA.y - wA * rAX
    impulseV = b2Math.MulMV(@m_linearMass, new b2Vec2((-CdotX), (-CdotY)))
    oldImpulseV = @m_linearImpulse.Copy()
    @m_linearImpulse.Add impulseV
    maxImpulse = step.dt * @m_maxForce
    if @m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse
      @m_linearImpulse.Normalize()
      @m_linearImpulse.Multiply maxImpulse
    impulseV = b2Math.SubtractVV(@m_linearImpulse, oldImpulseV)
    vA.x -= mA * impulseV.x
    vA.y -= mA * impulseV.y
    wA -= iA * (rAX * impulseV.y - rAY * impulseV.x)
    vB.x += mB * impulseV.x
    vB.y += mB * impulseV.y
    wB += iB * (rBX * impulseV.y - rBY * impulseV.x)
    bA.m_angularVelocity = wA
    bB.m_angularVelocity = wB
    return

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    true
 