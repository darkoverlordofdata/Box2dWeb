Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint

class Box2D.Dynamics.Joints.b2WeldJoint extends b2Joint

  constructor: (def) ->
    super def
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_impulse = new b2Vec3()
    @m_mass = new b2Mat33()
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_mass = new b2Mat33()
    return


  GetAnchorA: ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  GetAnchorB: ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    inv_dt * @m_impulse.z

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
    @m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB
    @m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB
    @m_mass.col3.x = (-rAY * iA) - rBY * iB
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB
    @m_mass.col3.y = rAX * iA + rBX * iB
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = iA + iB
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_impulse.z *= step.dtRatio
      bA.m_linearVelocity.x -= mA * @m_impulse.x
      bA.m_linearVelocity.y -= mA * @m_impulse.y
      bA.m_angularVelocity -= iA * (rAX * @m_impulse.y - rAY * @m_impulse.x + @m_impulse.z)
      bB.m_linearVelocity.x += mB * @m_impulse.x
      bB.m_linearVelocity.y += mB * @m_impulse.y
      bB.m_angularVelocity += iB * (rBX * @m_impulse.y - rBY * @m_impulse.x + @m_impulse.z)
    else
      @m_impulse.SetZero()
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
    Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY
    Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX
    Cdot2 = wB - wA
    impulse = new b2Vec3()
    @m_mass.Solve33 impulse, (-Cdot1X), (-Cdot1Y), (-Cdot2)
    @m_impulse.Add impulse
    vA.x -= mA * impulse.x
    vA.y -= mA * impulse.y
    wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z)
    vB.x += mB * impulse.x
    vB.y += mB * impulse.y
    wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z)
    bA.m_angularVelocity = wA
    bB.m_angularVelocity = wB
    return

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
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
    C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX
    C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY
    C2 = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop
    positionError = Math.sqrt(C1X * C1X + C1Y * C1Y)
    angularError = b2Math.Abs(C2)
    if positionError > k_allowedStretch
      iA *= 1.0
      iB *= 1.0
    @m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB
    @m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB
    @m_mass.col3.x = (-rAY * iA) - rBY * iB
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB
    @m_mass.col3.y = rAX * iA + rBX * iB
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = iA + iB
    impulse = new b2Vec3()
    @m_mass.Solve33 impulse, (-C1X), (-C1Y), (-C2)
    bA.m_sweep.c.x -= mA * impulse.x
    bA.m_sweep.c.y -= mA * impulse.y
    bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z)
    bB.m_sweep.c.x += mB * impulse.x
    bB.m_sweep.c.y += mB * impulse.y
    bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    positionError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop
