(->
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint
  b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef
  b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint
  b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef
  b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint
  b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef
  b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian
  b2Joint = Box2D.Dynamics.Joints.b2Joint
  b2JointDef = Box2D.Dynamics.Joints.b2JointDef
  b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge
  b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint
  b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef
  b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint
  b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
  b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint
  b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef
  b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint
  b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef
  b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint
  b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
  b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint
  b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
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
  Box2D.inherit b2DistanceJoint, Box2D.Dynamics.Joints.b2Joint
  b2DistanceJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2DistanceJoint.b2DistanceJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u = new b2Vec2()
    return

  b2DistanceJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2DistanceJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2DistanceJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_u.x, inv_dt * @m_impulse * @m_u.y)

  b2DistanceJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2DistanceJoint::GetLength = ->
    @m_length

  b2DistanceJoint::SetLength = (length) ->
    length = 0  if length is `undefined`
    @m_length = length
    return

  b2DistanceJoint::GetFrequency = ->
    @m_frequencyHz

  b2DistanceJoint::SetFrequency = (hz) ->
    hz = 0  if hz is `undefined`
    @m_frequencyHz = hz
    return

  b2DistanceJoint::GetDampingRatio = ->
    @m_dampingRatio

  b2DistanceJoint::SetDampingRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_dampingRatio = ratio
    return

  b2DistanceJoint::b2DistanceJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_length = def.length
    @m_frequencyHz = def.frequencyHz
    @m_dampingRatio = def.dampingRatio
    @m_impulse = 0.0
    @m_gamma = 0.0
    @m_bias = 0.0
    return

  b2DistanceJoint::InitVelocityConstraints = (step) ->
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

  b2DistanceJoint::SolveVelocityConstraints = (step) ->
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

  b2DistanceJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
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

  Box2D.inherit b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2DistanceJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2DistanceJointDef.b2DistanceJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2DistanceJointDef::b2DistanceJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_distanceJoint
    @length = 1.0
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

  b2DistanceJointDef::Initialize = (bA, bB, anchorA, anchorB) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchorA)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchorB)
    dX = anchorB.x - anchorA.x
    dY = anchorB.y - anchorA.y
    @length = Math.sqrt(dX * dX + dY * dY)
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

  Box2D.inherit b2FrictionJoint, Box2D.Dynamics.Joints.b2Joint
  b2FrictionJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2FrictionJoint.b2FrictionJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_linearMass = new b2Mat22()
    @m_linearImpulse = new b2Vec2()
    return

  b2FrictionJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  b2FrictionJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  b2FrictionJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_linearImpulse.x, inv_dt * @m_linearImpulse.y)

  b2FrictionJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_angularImpulse

  b2FrictionJoint::SetMaxForce = (force) ->
    force = 0  if force is `undefined`
    @m_maxForce = force
    return

  b2FrictionJoint::GetMaxForce = ->
    @m_maxForce

  b2FrictionJoint::SetMaxTorque = (torque) ->
    torque = 0  if torque is `undefined`
    @m_maxTorque = torque
    return

  b2FrictionJoint::GetMaxTorque = ->
    @m_maxTorque

  b2FrictionJoint::b2FrictionJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_linearMass.SetZero()
    @m_angularMass = 0.0
    @m_linearImpulse.SetZero()
    @m_angularImpulse = 0.0
    @m_maxForce = def.maxForce
    @m_maxTorque = def.maxTorque
    return

  b2FrictionJoint::InitVelocityConstraints = (step) ->
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

  b2FrictionJoint::SolveVelocityConstraints = (step) ->
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

  b2FrictionJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    true

  Box2D.inherit b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2FrictionJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2FrictionJointDef.b2FrictionJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2FrictionJointDef::b2FrictionJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_frictionJoint
    @maxForce = 0.0
    @maxTorque = 0.0
    return

  b2FrictionJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    return

  Box2D.inherit b2GearJoint, Box2D.Dynamics.Joints.b2Joint
  b2GearJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2GearJoint.b2GearJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_J = new b2Jacobian()
    return

  b2GearJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2GearJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2GearJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_J.linearB.x, inv_dt * @m_impulse * @m_J.linearB.y)

  b2GearJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    tMat = @m_bodyB.m_xf.R
    rX = @m_localAnchor1.x - @m_bodyB.m_sweep.localCenter.x
    rY = @m_localAnchor1.y - @m_bodyB.m_sweep.localCenter.y
    tX = tMat.col1.x * rX + tMat.col2.x * rY
    rY = tMat.col1.y * rX + tMat.col2.y * rY
    rX = tX
    PX = @m_impulse * @m_J.linearB.x
    PY = @m_impulse * @m_J.linearB.y
    inv_dt * (@m_impulse * @m_J.angularB - rX * PY + rY * PX)

  b2GearJoint::GetRatio = ->
    @m_ratio

  b2GearJoint::SetRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_ratio = ratio
    return

  b2GearJoint::b2GearJoint = (def) ->
    @__super.b2Joint.call this, def
    type1 = parseInt(def.joint1.m_type)
    type2 = parseInt(def.joint2.m_type)
    @m_revolute1 = null
    @m_prismatic1 = null
    @m_revolute2 = null
    @m_prismatic2 = null
    coordinate1 = 0
    coordinate2 = 0
    @m_ground1 = def.joint1.GetBodyA()
    @m_bodyA = def.joint1.GetBodyB()
    if type1 is b2Joint.e_revoluteJoint
      @m_revolute1 = ((if def.joint1 instanceof b2RevoluteJoint then def.joint1 else null))
      @m_groundAnchor1.SetV @m_revolute1.m_localAnchor1
      @m_localAnchor1.SetV @m_revolute1.m_localAnchor2
      coordinate1 = @m_revolute1.GetJointAngle()
    else
      @m_prismatic1 = ((if def.joint1 instanceof b2PrismaticJoint then def.joint1 else null))
      @m_groundAnchor1.SetV @m_prismatic1.m_localAnchor1
      @m_localAnchor1.SetV @m_prismatic1.m_localAnchor2
      coordinate1 = @m_prismatic1.GetJointTranslation()
    @m_ground2 = def.joint2.GetBodyA()
    @m_bodyB = def.joint2.GetBodyB()
    if type2 is b2Joint.e_revoluteJoint
      @m_revolute2 = ((if def.joint2 instanceof b2RevoluteJoint then def.joint2 else null))
      @m_groundAnchor2.SetV @m_revolute2.m_localAnchor1
      @m_localAnchor2.SetV @m_revolute2.m_localAnchor2
      coordinate2 = @m_revolute2.GetJointAngle()
    else
      @m_prismatic2 = ((if def.joint2 instanceof b2PrismaticJoint then def.joint2 else null))
      @m_groundAnchor2.SetV @m_prismatic2.m_localAnchor1
      @m_localAnchor2.SetV @m_prismatic2.m_localAnchor2
      coordinate2 = @m_prismatic2.GetJointTranslation()
    @m_ratio = def.ratio
    @m_constant = coordinate1 + @m_ratio * coordinate2
    @m_impulse = 0.0
    return

  b2GearJoint::InitVelocityConstraints = (step) ->
    g1 = @m_ground1
    g2 = @m_ground2
    bA = @m_bodyA
    bB = @m_bodyB
    ugX = 0
    ugY = 0
    rX = 0
    rY = 0
    tMat = undefined
    tVec = undefined
    crug = 0
    tX = 0
    K = 0.0
    @m_J.SetZero()
    if @m_revolute1
      @m_J.angularA = (-1.0)
      K += bA.m_invI
    else
      tMat = g1.m_xf.R
      tVec = @m_prismatic1.m_localXAxis1
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tMat = bA.m_xf.R
      rX = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      rY = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = tMat.col1.x * rX + tMat.col2.x * rY
      rY = tMat.col1.y * rX + tMat.col2.y * rY
      rX = tX
      crug = rX * ugY - rY * ugX
      @m_J.linearA.Set (-ugX), (-ugY)
      @m_J.angularA = (-crug)
      K += bA.m_invMass + bA.m_invI * crug * crug
    if @m_revolute2
      @m_J.angularB = (-@m_ratio)
      K += @m_ratio * @m_ratio * bB.m_invI
    else
      tMat = g2.m_xf.R
      tVec = @m_prismatic2.m_localXAxis1
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tMat = bB.m_xf.R
      rX = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      rY = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = tMat.col1.x * rX + tMat.col2.x * rY
      rY = tMat.col1.y * rX + tMat.col2.y * rY
      rX = tX
      crug = rX * ugY - rY * ugX
      @m_J.linearB.Set (-@m_ratio * ugX), (-@m_ratio * ugY)
      @m_J.angularB = (-@m_ratio * crug)
      K += @m_ratio * @m_ratio * (bB.m_invMass + bB.m_invI * crug * crug)
    @m_mass = (if K > 0.0 then 1.0 / K else 0.0)
    if step.warmStarting
      bA.m_linearVelocity.x += bA.m_invMass * @m_impulse * @m_J.linearA.x
      bA.m_linearVelocity.y += bA.m_invMass * @m_impulse * @m_J.linearA.y
      bA.m_angularVelocity += bA.m_invI * @m_impulse * @m_J.angularA
      bB.m_linearVelocity.x += bB.m_invMass * @m_impulse * @m_J.linearB.x
      bB.m_linearVelocity.y += bB.m_invMass * @m_impulse * @m_J.linearB.y
      bB.m_angularVelocity += bB.m_invI * @m_impulse * @m_J.angularB
    else
      @m_impulse = 0.0
    return

  b2GearJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    Cdot = @m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity)
    impulse = (-@m_mass * Cdot)
    @m_impulse += impulse
    bA.m_linearVelocity.x += bA.m_invMass * impulse * @m_J.linearA.x
    bA.m_linearVelocity.y += bA.m_invMass * impulse * @m_J.linearA.y
    bA.m_angularVelocity += bA.m_invI * impulse * @m_J.angularA
    bB.m_linearVelocity.x += bB.m_invMass * impulse * @m_J.linearB.x
    bB.m_linearVelocity.y += bB.m_invMass * impulse * @m_J.linearB.y
    bB.m_angularVelocity += bB.m_invI * impulse * @m_J.angularB
    return

  b2GearJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    linearError = 0.0
    bA = @m_bodyA
    bB = @m_bodyB
    coordinate1 = 0
    coordinate2 = 0
    if @m_revolute1
      coordinate1 = @m_revolute1.GetJointAngle()
    else
      coordinate1 = @m_prismatic1.GetJointTranslation()
    if @m_revolute2
      coordinate2 = @m_revolute2.GetJointAngle()
    else
      coordinate2 = @m_prismatic2.GetJointTranslation()
    C = @m_constant - (coordinate1 + @m_ratio * coordinate2)
    impulse = (-@m_mass * C)
    bA.m_sweep.c.x += bA.m_invMass * impulse * @m_J.linearA.x
    bA.m_sweep.c.y += bA.m_invMass * impulse * @m_J.linearA.y
    bA.m_sweep.a += bA.m_invI * impulse * @m_J.angularA
    bB.m_sweep.c.x += bB.m_invMass * impulse * @m_J.linearB.x
    bB.m_sweep.c.y += bB.m_invMass * impulse * @m_J.linearB.y
    bB.m_sweep.a += bB.m_invI * impulse * @m_J.angularB
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError < b2Settings.b2_linearSlop

  Box2D.inherit b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2GearJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2GearJointDef.b2GearJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    return

  b2GearJointDef::b2GearJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_gearJoint
    @joint1 = null
    @joint2 = null
    @ratio = 1.0
    return

  b2Jacobian.b2Jacobian = ->
    @linearA = new b2Vec2()
    @linearB = new b2Vec2()
    return

  b2Jacobian::SetZero = ->
    @linearA.SetZero()
    @angularA = 0.0
    @linearB.SetZero()
    @angularB = 0.0
    return

  b2Jacobian::Set = (x1, a1, x2, a2) ->
    a1 = 0  if a1 is `undefined`
    a2 = 0  if a2 is `undefined`
    @linearA.SetV x1
    @angularA = a1
    @linearB.SetV x2
    @angularB = a2
    return

  b2Jacobian::Compute = (x1, a1, x2, a2) ->
    a1 = 0  if a1 is `undefined`
    a2 = 0  if a2 is `undefined`
    (@linearA.x * x1.x + @linearA.y * x1.y) + @angularA * a1 + (@linearB.x * x2.x + @linearB.y * x2.y) + @angularB * a2

  b2Joint.b2Joint = ->
    @m_edgeA = new b2JointEdge()
    @m_edgeB = new b2JointEdge()
    @m_localCenterA = new b2Vec2()
    @m_localCenterB = new b2Vec2()
    return

  b2Joint::GetType = ->
    @m_type

  b2Joint::GetAnchorA = ->
    null

  b2Joint::GetAnchorB = ->
    null

  b2Joint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    null

  b2Joint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2Joint::GetBodyA = ->
    @m_bodyA

  b2Joint::GetBodyB = ->
    @m_bodyB

  b2Joint::GetNext = ->
    @m_next

  b2Joint::GetUserData = ->
    @m_userData

  b2Joint::SetUserData = (data) ->
    @m_userData = data
    return

  b2Joint::IsActive = ->
    @m_bodyA.IsActive() and @m_bodyB.IsActive()

  b2Joint.Create = (def, allocator) ->
    joint = null
    switch def.type
      when b2Joint.e_distanceJoint
        joint = new b2DistanceJoint(((if def instanceof b2DistanceJointDef then def else null)))
      when b2Joint.e_mouseJoint
        joint = new b2MouseJoint(((if def instanceof b2MouseJointDef then def else null)))
      when b2Joint.e_prismaticJoint
        joint = new b2PrismaticJoint(((if def instanceof b2PrismaticJointDef then def else null)))
      when b2Joint.e_revoluteJoint
        joint = new b2RevoluteJoint(((if def instanceof b2RevoluteJointDef then def else null)))
      when b2Joint.e_pulleyJoint
        joint = new b2PulleyJoint(((if def instanceof b2PulleyJointDef then def else null)))
      when b2Joint.e_gearJoint
        joint = new b2GearJoint(((if def instanceof b2GearJointDef then def else null)))
      when b2Joint.e_lineJoint
        joint = new b2LineJoint(((if def instanceof b2LineJointDef then def else null)))
      when b2Joint.e_weldJoint
        joint = new b2WeldJoint(((if def instanceof b2WeldJointDef then def else null)))
      when b2Joint.e_frictionJoint
        joint = new b2FrictionJoint(((if def instanceof b2FrictionJointDef then def else null)))
      else
    joint

  b2Joint.Destroy = (joint, allocator) ->

  b2Joint::b2Joint = (def) ->
    b2Settings.b2Assert def.bodyA isnt def.bodyB
    @m_type = def.type
    @m_prev = null
    @m_next = null
    @m_bodyA = def.bodyA
    @m_bodyB = def.bodyB
    @m_collideConnected = def.collideConnected
    @m_islandFlag = false
    @m_userData = def.userData
    return

  b2Joint::InitVelocityConstraints = (step) ->

  b2Joint::SolveVelocityConstraints = (step) ->

  b2Joint::FinalizeVelocityConstraints = ->

  b2Joint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    false

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0
    Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1
    Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2
    Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3
    Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4
    Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5
    Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6
    Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7
    Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8
    Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9
    Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0
    Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1
    Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2
    Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3
    return

  b2JointDef.b2JointDef = ->

  b2JointDef::b2JointDef = ->
    @type = b2Joint.e_unknownJoint
    @userData = null
    @bodyA = null
    @bodyB = null
    @collideConnected = false
    return

  b2JointEdge.b2JointEdge = ->

  Box2D.inherit b2LineJoint, Box2D.Dynamics.Joints.b2Joint
  b2LineJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2LineJoint.b2LineJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_localXAxis1 = new b2Vec2()
    @m_localYAxis1 = new b2Vec2()
    @m_axis = new b2Vec2()
    @m_perp = new b2Vec2()
    @m_K = new b2Mat22()
    @m_impulse = new b2Vec2()
    return

  b2LineJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2LineJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2LineJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * (@m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.y) * @m_axis.x), inv_dt * (@m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.y) * @m_axis.y))

  b2LineJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.y

  b2LineJoint::GetJointTranslation = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    p1 = bA.GetWorldPoint(@m_localAnchor1)
    p2 = bB.GetWorldPoint(@m_localAnchor2)
    dX = p2.x - p1.x
    dY = p2.y - p1.y
    axis = bA.GetWorldVector(@m_localXAxis1)
    translation = axis.x * dX + axis.y * dY
    translation

  b2LineJoint::GetJointSpeed = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
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
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    dX = p2X - p1X
    dY = p2Y - p1Y
    axis = bA.GetWorldVector(@m_localXAxis1)
    v1 = bA.m_linearVelocity
    v2 = bB.m_linearVelocity
    w1 = bA.m_angularVelocity
    w2 = bB.m_angularVelocity
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)))
    speed

  b2LineJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2LineJoint::EnableLimit = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableLimit = flag
    return

  b2LineJoint::GetLowerLimit = ->
    @m_lowerTranslation

  b2LineJoint::GetUpperLimit = ->
    @m_upperTranslation

  b2LineJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_lowerTranslation = lower
    @m_upperTranslation = upper
    return

  b2LineJoint::IsMotorEnabled = ->
    @m_enableMotor

  b2LineJoint::EnableMotor = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor = flag
    return

  b2LineJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2LineJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2LineJoint::SetMaxMotorForce = (force) ->
    force = 0  if force is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_maxMotorForce = force
    return

  b2LineJoint::GetMaxMotorForce = ->
    @m_maxMotorForce

  b2LineJoint::GetMotorForce = ->
    @m_motorImpulse

  b2LineJoint::b2LineJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_localXAxis1.SetV def.localAxisA
    @m_localYAxis1.x = (-@m_localXAxis1.y)
    @m_localYAxis1.y = @m_localXAxis1.x
    @m_impulse.SetZero()
    @m_motorMass = 0.0
    @m_motorImpulse = 0.0
    @m_lowerTranslation = def.lowerTranslation
    @m_upperTranslation = def.upperTranslation
    @m_maxMotorForce = def.maxMotorForce
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    @m_axis.SetZero()
    @m_perp.SetZero()
    return

  b2LineJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    @m_localCenterA.SetV bA.GetLocalCenter()
    @m_localCenterB.SetV bB.GetLocalCenter()
    xf1 = bA.GetTransform()
    xf2 = bB.GetTransform()
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @m_invMassA = bA.m_invMass
    @m_invMassB = bB.m_invMass
    @m_invIA = bA.m_invI
    @m_invIB = bB.m_invI
    @m_axis.SetV b2Math.MulMV(xf1.R, @m_localXAxis1)
    @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
    @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
    @m_motorMass = @m_invMassA + @m_invMassB + @m_invIA * @m_a1 * @m_a1 + @m_invIB * @m_a2 * @m_a2
    @m_motorMass = (if @m_motorMass > Number.MIN_VALUE then 1.0 / @m_motorMass else 0.0)
    @m_perp.SetV b2Math.MulMV(xf1.R, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    m1 = @m_invMassA
    m2 = @m_invMassB
    i1 = @m_invIA
    i2 = @m_invIB
    @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
    @m_K.col1.y = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
    @m_K.col2.x = @m_K.col1.y
    @m_K.col2.y = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
    if @m_enableLimit
      jointTransition = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointTransition <= @m_lowerTranslation
        unless @m_limitState is b2Joint.e_atLowerLimit
          @m_limitState = b2Joint.e_atLowerLimit
          @m_impulse.y = 0.0
      else if jointTransition >= @m_upperTranslation
        unless @m_limitState is b2Joint.e_atUpperLimit
          @m_limitState = b2Joint.e_atUpperLimit
          @m_impulse.y = 0.0
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.y = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.y) * @m_axis.x
      PY = @m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.y) * @m_axis.y
      L1 = @m_impulse.x * @m_s1 + (@m_motorImpulse + @m_impulse.y) * @m_a1
      L2 = @m_impulse.x * @m_s2 + (@m_motorImpulse + @m_impulse.y) * @m_a2
      bA.m_linearVelocity.x -= @m_invMassA * PX
      bA.m_linearVelocity.y -= @m_invMassA * PY
      bA.m_angularVelocity -= @m_invIA * L1
      bB.m_linearVelocity.x += @m_invMassB * PX
      bB.m_linearVelocity.y += @m_invMassB * PY
      bB.m_angularVelocity += @m_invIB * L2
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2LineJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    PX = 0
    PY = 0
    L1 = 0
    L2 = 0
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      impulse = @m_motorMass * (@m_motorSpeed - Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorForce
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      PX = impulse * @m_axis.x
      PY = impulse * @m_axis.y
      L1 = impulse * @m_a1
      L2 = impulse * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    Cdot1 = @m_perp.x * (v2.x - v1.x) + @m_perp.y * (v2.y - v1.y) + @m_s2 * w2 - @m_s1 * w1
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      Cdot2 = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      f1 = @m_impulse.Copy()
      df = @m_K.Solve(new b2Vec2(), (-Cdot1), (-Cdot2))
      @m_impulse.Add df
      if @m_limitState is b2Joint.e_atLowerLimit
        @m_impulse.y = b2Math.Max(@m_impulse.y, 0.0)
      else @m_impulse.y = b2Math.Min(@m_impulse.y, 0.0)  if @m_limitState is b2Joint.e_atUpperLimit
      b = (-Cdot1) - (@m_impulse.y - f1.y) * @m_K.col2.x
      f2r = 0
      unless @m_K.col1.x is 0.0
        f2r = b / @m_K.col1.x + f1.x
      else
        f2r = f1.x
      @m_impulse.x = f2r
      df.x = @m_impulse.x - f1.x
      df.y = @m_impulse.y - f1.y
      PX = df.x * @m_perp.x + df.y * @m_axis.x
      PY = df.x * @m_perp.y + df.y * @m_axis.y
      L1 = df.x * @m_s1 + df.y * @m_a1
      L2 = df.x * @m_s2 + df.y * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    else
      df2 = 0
      unless @m_K.col1.x is 0.0
        df2 = (-Cdot1) / @m_K.col1.x
      else
        df2 = 0.0
      @m_impulse.x += df2
      PX = df2 * @m_perp.x
      PY = df2 * @m_perp.y
      L1 = df2 * @m_s1
      L2 = df2 * @m_s2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2LineJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    limitC = 0
    oldLimitImpulse = 0
    bA = @m_bodyA
    bB = @m_bodyB
    c1 = bA.m_sweep.c
    a1 = bA.m_sweep.a
    c2 = bB.m_sweep.c
    a2 = bB.m_sweep.a
    tMat = undefined
    tX = 0
    m1 = 0
    m2 = 0
    i1 = 0
    i2 = 0
    linearError = 0.0
    angularError = 0.0
    active = false
    C2 = 0.0
    R1 = b2Mat22.FromAngle(a1)
    R2 = b2Mat22.FromAngle(a2)
    tMat = R1
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = R2
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = c2.x + r2X - c1.x - r1X
    dY = c2.y + r2Y - c1.y - r1Y
    if @m_enableLimit
      @m_axis = b2Math.MulMV(R1, @m_localXAxis1)
      @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
      @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
      translation = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
        linearError = b2Math.Abs(translation)
        active = true
      else if translation <= @m_lowerTranslation
        C2 = b2Math.Clamp(translation - @m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
        linearError = @m_lowerTranslation - translation
        active = true
      else if translation >= @m_upperTranslation
        C2 = b2Math.Clamp(translation - @m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection)
        linearError = translation - @m_upperTranslation
        active = true
    @m_perp = b2Math.MulMV(R1, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    impulse = new b2Vec2()
    C1 = @m_perp.x * dX + @m_perp.y * dY
    linearError = b2Math.Max(linearError, b2Math.Abs(C1))
    angularError = 0.0
    if active
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      @m_K.col1.y = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
      @m_K.col2.x = @m_K.col1.y
      @m_K.col2.y = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
      @m_K.Solve impulse, (-C1), (-C2)
    else
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      k11 = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      impulse1 = 0
      unless k11 is 0.0
        impulse1 = (-C1) / k11
      else
        impulse1 = 0.0
      impulse.x = impulse1
      impulse.y = 0.0
    PX = impulse.x * @m_perp.x + impulse.y * @m_axis.x
    PY = impulse.x * @m_perp.y + impulse.y * @m_axis.y
    L1 = impulse.x * @m_s1 + impulse.y * @m_a1
    L2 = impulse.x * @m_s2 + impulse.y * @m_a2
    c1.x -= @m_invMassA * PX
    c1.y -= @m_invMassA * PY
    a1 -= @m_invIA * L1
    c2.x += @m_invMassB * PX
    c2.y += @m_invMassB * PY
    a2 += @m_invIB * L2
    bA.m_sweep.a = a1
    bB.m_sweep.a = a2
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.inherit b2LineJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2LineJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2LineJointDef.b2LineJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAxisA = new b2Vec2()
    return

  b2LineJointDef::b2LineJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_lineJoint
    @localAxisA.Set 1.0, 0.0
    @enableLimit = false
    @lowerTranslation = 0.0
    @upperTranslation = 0.0
    @enableMotor = false
    @maxMotorForce = 0.0
    @motorSpeed = 0.0
    return

  b2LineJointDef::Initialize = (bA, bB, anchor, axis) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @localAxisA = @bodyA.GetLocalVector(axis)
    return

  Box2D.inherit b2MouseJoint, Box2D.Dynamics.Joints.b2Joint
  b2MouseJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2MouseJoint.b2MouseJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @K = new b2Mat22()
    @K1 = new b2Mat22()
    @K2 = new b2Mat22()
    @m_localAnchor = new b2Vec2()
    @m_target = new b2Vec2()
    @m_impulse = new b2Vec2()
    @m_mass = new b2Mat22()
    @m_C = new b2Vec2()
    return

  b2MouseJoint::GetAnchorA = ->
    @m_target

  b2MouseJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor

  b2MouseJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2MouseJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2MouseJoint::GetTarget = ->
    @m_target

  b2MouseJoint::SetTarget = (target) ->
    @m_bodyB.SetAwake true  if @m_bodyB.IsAwake() is false
    @m_target = target
    return

  b2MouseJoint::GetMaxForce = ->
    @m_maxForce

  b2MouseJoint::SetMaxForce = (maxForce) ->
    maxForce = 0  if maxForce is `undefined`
    @m_maxForce = maxForce
    return

  b2MouseJoint::GetFrequency = ->
    @m_frequencyHz

  b2MouseJoint::SetFrequency = (hz) ->
    hz = 0  if hz is `undefined`
    @m_frequencyHz = hz
    return

  b2MouseJoint::GetDampingRatio = ->
    @m_dampingRatio

  b2MouseJoint::SetDampingRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_dampingRatio = ratio
    return

  b2MouseJoint::b2MouseJoint = (def) ->
    @__super.b2Joint.call this, def
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
    @m_beta = 0.0
    @m_gamma = 0.0
    return

  b2MouseJoint::InitVelocityConstraints = (step) ->
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

  b2MouseJoint::SolveVelocityConstraints = (step) ->
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

  b2MouseJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    true

  Box2D.inherit b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2MouseJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2MouseJointDef.b2MouseJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @target = new b2Vec2()
    return

  b2MouseJointDef::b2MouseJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_mouseJoint
    @maxForce = 0.0
    @frequencyHz = 5.0
    @dampingRatio = 0.7
    return

  Box2D.inherit b2PrismaticJoint, Box2D.Dynamics.Joints.b2Joint
  b2PrismaticJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2PrismaticJoint.b2PrismaticJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_localXAxis1 = new b2Vec2()
    @m_localYAxis1 = new b2Vec2()
    @m_axis = new b2Vec2()
    @m_perp = new b2Vec2()
    @m_K = new b2Mat33()
    @m_impulse = new b2Vec3()
    return

  b2PrismaticJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2PrismaticJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2PrismaticJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * (@m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.z) * @m_axis.x), inv_dt * (@m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.z) * @m_axis.y))

  b2PrismaticJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.y

  b2PrismaticJoint::GetJointTranslation = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    p1 = bA.GetWorldPoint(@m_localAnchor1)
    p2 = bB.GetWorldPoint(@m_localAnchor2)
    dX = p2.x - p1.x
    dY = p2.y - p1.y
    axis = bA.GetWorldVector(@m_localXAxis1)
    translation = axis.x * dX + axis.y * dY
    translation

  b2PrismaticJoint::GetJointSpeed = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
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
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    dX = p2X - p1X
    dY = p2Y - p1Y
    axis = bA.GetWorldVector(@m_localXAxis1)
    v1 = bA.m_linearVelocity
    v2 = bB.m_linearVelocity
    w1 = bA.m_angularVelocity
    w2 = bB.m_angularVelocity
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)))
    speed

  b2PrismaticJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2PrismaticJoint::EnableLimit = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableLimit = flag
    return

  b2PrismaticJoint::GetLowerLimit = ->
    @m_lowerTranslation

  b2PrismaticJoint::GetUpperLimit = ->
    @m_upperTranslation

  b2PrismaticJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_lowerTranslation = lower
    @m_upperTranslation = upper
    return

  b2PrismaticJoint::IsMotorEnabled = ->
    @m_enableMotor

  b2PrismaticJoint::EnableMotor = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor = flag
    return

  b2PrismaticJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2PrismaticJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2PrismaticJoint::SetMaxMotorForce = (force) ->
    force = 0  if force is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_maxMotorForce = force
    return

  b2PrismaticJoint::GetMotorForce = ->
    @m_motorImpulse

  b2PrismaticJoint::b2PrismaticJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_localXAxis1.SetV def.localAxisA
    @m_localYAxis1.x = (-@m_localXAxis1.y)
    @m_localYAxis1.y = @m_localXAxis1.x
    @m_refAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_motorMass = 0.0
    @m_motorImpulse = 0.0
    @m_lowerTranslation = def.lowerTranslation
    @m_upperTranslation = def.upperTranslation
    @m_maxMotorForce = def.maxMotorForce
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    @m_axis.SetZero()
    @m_perp.SetZero()
    return

  b2PrismaticJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    @m_localCenterA.SetV bA.GetLocalCenter()
    @m_localCenterB.SetV bB.GetLocalCenter()
    xf1 = bA.GetTransform()
    xf2 = bB.GetTransform()
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @m_invMassA = bA.m_invMass
    @m_invMassB = bB.m_invMass
    @m_invIA = bA.m_invI
    @m_invIB = bB.m_invI
    @m_axis.SetV b2Math.MulMV(xf1.R, @m_localXAxis1)
    @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
    @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
    @m_motorMass = @m_invMassA + @m_invMassB + @m_invIA * @m_a1 * @m_a1 + @m_invIB * @m_a2 * @m_a2
    @m_motorMass = 1.0 / @m_motorMass  if @m_motorMass > Number.MIN_VALUE
    @m_perp.SetV b2Math.MulMV(xf1.R, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    m1 = @m_invMassA
    m2 = @m_invMassB
    i1 = @m_invIA
    i2 = @m_invIB
    @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
    @m_K.col1.y = i1 * @m_s1 + i2 * @m_s2
    @m_K.col1.z = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
    @m_K.col2.x = @m_K.col1.y
    @m_K.col2.y = i1 + i2
    @m_K.col2.z = i1 * @m_a1 + i2 * @m_a2
    @m_K.col3.x = @m_K.col1.z
    @m_K.col3.y = @m_K.col2.z
    @m_K.col3.z = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
    if @m_enableLimit
      jointTransition = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointTransition <= @m_lowerTranslation
        unless @m_limitState is b2Joint.e_atLowerLimit
          @m_limitState = b2Joint.e_atLowerLimit
          @m_impulse.z = 0.0
      else if jointTransition >= @m_upperTranslation
        unless @m_limitState is b2Joint.e_atUpperLimit
          @m_limitState = b2Joint.e_atUpperLimit
          @m_impulse.z = 0.0
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.z = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.z) * @m_axis.x
      PY = @m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.z) * @m_axis.y
      L1 = @m_impulse.x * @m_s1 + @m_impulse.y + (@m_motorImpulse + @m_impulse.z) * @m_a1
      L2 = @m_impulse.x * @m_s2 + @m_impulse.y + (@m_motorImpulse + @m_impulse.z) * @m_a2
      bA.m_linearVelocity.x -= @m_invMassA * PX
      bA.m_linearVelocity.y -= @m_invMassA * PY
      bA.m_angularVelocity -= @m_invIA * L1
      bB.m_linearVelocity.x += @m_invMassB * PX
      bB.m_linearVelocity.y += @m_invMassB * PY
      bB.m_angularVelocity += @m_invIB * L2
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2PrismaticJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    PX = 0
    PY = 0
    L1 = 0
    L2 = 0
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      impulse = @m_motorMass * (@m_motorSpeed - Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorForce
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      PX = impulse * @m_axis.x
      PY = impulse * @m_axis.y
      L1 = impulse * @m_a1
      L2 = impulse * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    Cdot1X = @m_perp.x * (v2.x - v1.x) + @m_perp.y * (v2.y - v1.y) + @m_s2 * w2 - @m_s1 * w1
    Cdot1Y = w2 - w1
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      Cdot2 = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      f1 = @m_impulse.Copy()
      df = @m_K.Solve33(new b2Vec3(), (-Cdot1X), (-Cdot1Y), (-Cdot2))
      @m_impulse.Add df
      if @m_limitState is b2Joint.e_atLowerLimit
        @m_impulse.z = b2Math.Max(@m_impulse.z, 0.0)
      else @m_impulse.z = b2Math.Min(@m_impulse.z, 0.0)  if @m_limitState is b2Joint.e_atUpperLimit
      bX = (-Cdot1X) - (@m_impulse.z - f1.z) * @m_K.col3.x
      bY = (-Cdot1Y) - (@m_impulse.z - f1.z) * @m_K.col3.y
      f2r = @m_K.Solve22(new b2Vec2(), bX, bY)
      f2r.x += f1.x
      f2r.y += f1.y
      @m_impulse.x = f2r.x
      @m_impulse.y = f2r.y
      df.x = @m_impulse.x - f1.x
      df.y = @m_impulse.y - f1.y
      df.z = @m_impulse.z - f1.z
      PX = df.x * @m_perp.x + df.z * @m_axis.x
      PY = df.x * @m_perp.y + df.z * @m_axis.y
      L1 = df.x * @m_s1 + df.y + df.z * @m_a1
      L2 = df.x * @m_s2 + df.y + df.z * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    else
      df2 = @m_K.Solve22(new b2Vec2(), (-Cdot1X), (-Cdot1Y))
      @m_impulse.x += df2.x
      @m_impulse.y += df2.y
      PX = df2.x * @m_perp.x
      PY = df2.x * @m_perp.y
      L1 = df2.x * @m_s1 + df2.y
      L2 = df2.x * @m_s2 + df2.y
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2PrismaticJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    limitC = 0
    oldLimitImpulse = 0
    bA = @m_bodyA
    bB = @m_bodyB
    c1 = bA.m_sweep.c
    a1 = bA.m_sweep.a
    c2 = bB.m_sweep.c
    a2 = bB.m_sweep.a
    tMat = undefined
    tX = 0
    m1 = 0
    m2 = 0
    i1 = 0
    i2 = 0
    linearError = 0.0
    angularError = 0.0
    active = false
    C2 = 0.0
    R1 = b2Mat22.FromAngle(a1)
    R2 = b2Mat22.FromAngle(a2)
    tMat = R1
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = R2
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = c2.x + r2X - c1.x - r1X
    dY = c2.y + r2Y - c1.y - r1Y
    if @m_enableLimit
      @m_axis = b2Math.MulMV(R1, @m_localXAxis1)
      @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
      @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
      translation = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
        linearError = b2Math.Abs(translation)
        active = true
      else if translation <= @m_lowerTranslation
        C2 = b2Math.Clamp(translation - @m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
        linearError = @m_lowerTranslation - translation
        active = true
      else if translation >= @m_upperTranslation
        C2 = b2Math.Clamp(translation - @m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection)
        linearError = translation - @m_upperTranslation
        active = true
    @m_perp = b2Math.MulMV(R1, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    impulse = new b2Vec3()
    C1X = @m_perp.x * dX + @m_perp.y * dY
    C1Y = a2 - a1 - @m_refAngle
    linearError = b2Math.Max(linearError, b2Math.Abs(C1X))
    angularError = b2Math.Abs(C1Y)
    if active
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      @m_K.col1.y = i1 * @m_s1 + i2 * @m_s2
      @m_K.col1.z = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
      @m_K.col2.x = @m_K.col1.y
      @m_K.col2.y = i1 + i2
      @m_K.col2.z = i1 * @m_a1 + i2 * @m_a2
      @m_K.col3.x = @m_K.col1.z
      @m_K.col3.y = @m_K.col2.z
      @m_K.col3.z = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
      @m_K.Solve33 impulse, (-C1X), (-C1Y), (-C2)
    else
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      k11 = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      k12 = i1 * @m_s1 + i2 * @m_s2
      k22 = i1 + i2
      @m_K.col1.Set k11, k12, 0.0
      @m_K.col2.Set k12, k22, 0.0
      impulse1 = @m_K.Solve22(new b2Vec2(), (-C1X), (-C1Y))
      impulse.x = impulse1.x
      impulse.y = impulse1.y
      impulse.z = 0.0
    PX = impulse.x * @m_perp.x + impulse.z * @m_axis.x
    PY = impulse.x * @m_perp.y + impulse.z * @m_axis.y
    L1 = impulse.x * @m_s1 + impulse.y + impulse.z * @m_a1
    L2 = impulse.x * @m_s2 + impulse.y + impulse.z * @m_a2
    c1.x -= @m_invMassA * PX
    c1.y -= @m_invMassA * PY
    a1 -= @m_invIA * L1
    c2.x += @m_invMassB * PX
    c2.y += @m_invMassB * PY
    a2 += @m_invIB * L2
    bA.m_sweep.a = a1
    bB.m_sweep.a = a2
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.inherit b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2PrismaticJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2PrismaticJointDef.b2PrismaticJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAxisA = new b2Vec2()
    return

  b2PrismaticJointDef::b2PrismaticJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_prismaticJoint
    @localAxisA.Set 1.0, 0.0
    @referenceAngle = 0.0
    @enableLimit = false
    @lowerTranslation = 0.0
    @upperTranslation = 0.0
    @enableMotor = false
    @maxMotorForce = 0.0
    @motorSpeed = 0.0
    return

  b2PrismaticJointDef::Initialize = (bA, bB, anchor, axis) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @localAxisA = @bodyA.GetLocalVector(axis)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  Box2D.inherit b2PulleyJoint, Box2D.Dynamics.Joints.b2Joint
  b2PulleyJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2PulleyJoint.b2PulleyJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u1 = new b2Vec2()
    @m_u2 = new b2Vec2()
    return

  b2PulleyJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2PulleyJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2PulleyJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_u2.x, inv_dt * @m_impulse * @m_u2.y)

  b2PulleyJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2PulleyJoint::GetGroundAnchorA = ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor1
    a

  b2PulleyJoint::GetGroundAnchorB = ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor2
    a

  b2PulleyJoint::GetLength1 = ->
    p = @m_bodyA.GetWorldPoint(@m_localAnchor1)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    dX = p.x - sX
    dY = p.y - sY
    Math.sqrt dX * dX + dY * dY

  b2PulleyJoint::GetLength2 = ->
    p = @m_bodyB.GetWorldPoint(@m_localAnchor2)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    dX = p.x - sX
    dY = p.y - sY
    Math.sqrt dX * dX + dY * dY

  b2PulleyJoint::GetRatio = ->
    @m_ratio

  b2PulleyJoint::b2PulleyJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_ground = @m_bodyA.m_world.m_groundBody
    @m_groundAnchor1.x = def.groundAnchorA.x - @m_ground.m_xf.position.x
    @m_groundAnchor1.y = def.groundAnchorA.y - @m_ground.m_xf.position.y
    @m_groundAnchor2.x = def.groundAnchorB.x - @m_ground.m_xf.position.x
    @m_groundAnchor2.y = def.groundAnchorB.y - @m_ground.m_xf.position.y
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_ratio = def.ratio
    @m_constant = def.lengthA + @m_ratio * def.lengthB
    @m_maxLength1 = b2Math.Min(def.maxLengthA, @m_constant - @m_ratio * b2PulleyJoint.b2_minPulleyLength)
    @m_maxLength2 = b2Math.Min(def.maxLengthB, (@m_constant - b2PulleyJoint.b2_minPulleyLength) / @m_ratio)
    @m_impulse = 0.0
    @m_limitImpulse1 = 0.0
    @m_limitImpulse2 = 0.0
    return

  b2PulleyJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
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
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    s1X = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    s1Y = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    s2X = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    s2Y = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    @m_u1.Set p1X - s1X, p1Y - s1Y
    @m_u2.Set p2X - s2X, p2Y - s2Y
    length1 = @m_u1.Length()
    length2 = @m_u2.Length()
    if length1 > b2Settings.b2_linearSlop
      @m_u1.Multiply 1.0 / length1
    else
      @m_u1.SetZero()
    if length2 > b2Settings.b2_linearSlop
      @m_u2.Multiply 1.0 / length2
    else
      @m_u2.SetZero()
    C = @m_constant - length1 - @m_ratio * length2
    if C > 0.0
      @m_state = b2Joint.e_inactiveLimit
      @m_impulse = 0.0
    else
      @m_state = b2Joint.e_atUpperLimit
    if length1 < @m_maxLength1
      @m_limitState1 = b2Joint.e_inactiveLimit
      @m_limitImpulse1 = 0.0
    else
      @m_limitState1 = b2Joint.e_atUpperLimit
    if length2 < @m_maxLength2
      @m_limitState2 = b2Joint.e_inactiveLimit
      @m_limitImpulse2 = 0.0
    else
      @m_limitState2 = b2Joint.e_atUpperLimit
    cr1u1 = r1X * @m_u1.y - r1Y * @m_u1.x
    cr2u2 = r2X * @m_u2.y - r2Y * @m_u2.x
    @m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1
    @m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2
    @m_pulleyMass = @m_limitMass1 + @m_ratio * @m_ratio * @m_limitMass2
    @m_limitMass1 = 1.0 / @m_limitMass1
    @m_limitMass2 = 1.0 / @m_limitMass2
    @m_pulleyMass = 1.0 / @m_pulleyMass
    if step.warmStarting
      @m_impulse *= step.dtRatio
      @m_limitImpulse1 *= step.dtRatio
      @m_limitImpulse2 *= step.dtRatio
      P1X = ((-@m_impulse) - @m_limitImpulse1) * @m_u1.x
      P1Y = ((-@m_impulse) - @m_limitImpulse1) * @m_u1.y
      P2X = ((-@m_ratio * @m_impulse) - @m_limitImpulse2) * @m_u2.x
      P2Y = ((-@m_ratio * @m_impulse) - @m_limitImpulse2) * @m_u2.y
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    else
      @m_impulse = 0.0
      @m_limitImpulse1 = 0.0
      @m_limitImpulse2 = 0.0
    return

  b2PulleyJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
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
    v1X = 0
    v1Y = 0
    v2X = 0
    v2Y = 0
    P1X = 0
    P1Y = 0
    P2X = 0
    P2Y = 0
    Cdot = 0
    impulse = 0
    oldImpulse = 0
    if @m_state is b2Joint.e_atUpperLimit
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
      Cdot = (-(@m_u1.x * v1X + @m_u1.y * v1Y)) - @m_ratio * (@m_u2.x * v2X + @m_u2.y * v2Y)
      impulse = @m_pulleyMass * (-Cdot)
      oldImpulse = @m_impulse
      @m_impulse = b2Math.Max(0.0, @m_impulse + impulse)
      impulse = @m_impulse - oldImpulse
      P1X = (-impulse * @m_u1.x)
      P1Y = (-impulse * @m_u1.y)
      P2X = (-@m_ratio * impulse * @m_u2.x)
      P2Y = (-@m_ratio * impulse * @m_u2.y)
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    if @m_limitState1 is b2Joint.e_atUpperLimit
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
      Cdot = (-(@m_u1.x * v1X + @m_u1.y * v1Y))
      impulse = (-@m_limitMass1 * Cdot)
      oldImpulse = @m_limitImpulse1
      @m_limitImpulse1 = b2Math.Max(0.0, @m_limitImpulse1 + impulse)
      impulse = @m_limitImpulse1 - oldImpulse
      P1X = (-impulse * @m_u1.x)
      P1Y = (-impulse * @m_u1.y)
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
    if @m_limitState2 is b2Joint.e_atUpperLimit
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
      Cdot = (-(@m_u2.x * v2X + @m_u2.y * v2Y))
      impulse = (-@m_limitMass2 * Cdot)
      oldImpulse = @m_limitImpulse2
      @m_limitImpulse2 = b2Math.Max(0.0, @m_limitImpulse2 + impulse)
      impulse = @m_limitImpulse2 - oldImpulse
      P2X = (-impulse * @m_u2.x)
      P2Y = (-impulse * @m_u2.y)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    return

  b2PulleyJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    s1X = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    s1Y = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    s2X = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    s2Y = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    r1X = 0
    r1Y = 0
    r2X = 0
    r2Y = 0
    p1X = 0
    p1Y = 0
    p2X = 0
    p2Y = 0
    length1 = 0
    length2 = 0
    C = 0
    impulse = 0
    oldImpulse = 0
    oldLimitPositionImpulse = 0
    tX = 0
    linearError = 0.0
    if @m_state is b2Joint.e_atUpperLimit
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
      p1X = bA.m_sweep.c.x + r1X
      p1Y = bA.m_sweep.c.y + r1Y
      p2X = bB.m_sweep.c.x + r2X
      p2Y = bB.m_sweep.c.y + r2Y
      @m_u1.Set p1X - s1X, p1Y - s1Y
      @m_u2.Set p2X - s2X, p2Y - s2Y
      length1 = @m_u1.Length()
      length2 = @m_u2.Length()
      if length1 > b2Settings.b2_linearSlop
        @m_u1.Multiply 1.0 / length1
      else
        @m_u1.SetZero()
      if length2 > b2Settings.b2_linearSlop
        @m_u2.Multiply 1.0 / length2
      else
        @m_u2.SetZero()
      C = @m_constant - length1 - @m_ratio * length2
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_pulleyMass * C)
      p1X = (-impulse * @m_u1.x)
      p1Y = (-impulse * @m_u1.y)
      p2X = (-@m_ratio * impulse * @m_u2.x)
      p2Y = (-@m_ratio * impulse * @m_u2.y)
      bA.m_sweep.c.x += bA.m_invMass * p1X
      bA.m_sweep.c.y += bA.m_invMass * p1Y
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X)
      bB.m_sweep.c.x += bB.m_invMass * p2X
      bB.m_sweep.c.y += bB.m_invMass * p2Y
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X)
      bA.SynchronizeTransform()
      bB.SynchronizeTransform()
    if @m_limitState1 is b2Joint.e_atUpperLimit
      tMat = bA.m_xf.R
      r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
      r1X = tX
      p1X = bA.m_sweep.c.x + r1X
      p1Y = bA.m_sweep.c.y + r1Y
      @m_u1.Set p1X - s1X, p1Y - s1Y
      length1 = @m_u1.Length()
      if length1 > b2Settings.b2_linearSlop
        @m_u1.x *= 1.0 / length1
        @m_u1.y *= 1.0 / length1
      else
        @m_u1.SetZero()
      C = @m_maxLength1 - length1
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_limitMass1 * C)
      p1X = (-impulse * @m_u1.x)
      p1Y = (-impulse * @m_u1.y)
      bA.m_sweep.c.x += bA.m_invMass * p1X
      bA.m_sweep.c.y += bA.m_invMass * p1Y
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X)
      bA.SynchronizeTransform()
    if @m_limitState2 is b2Joint.e_atUpperLimit
      tMat = bB.m_xf.R
      r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
      r2X = tX
      p2X = bB.m_sweep.c.x + r2X
      p2Y = bB.m_sweep.c.y + r2Y
      @m_u2.Set p2X - s2X, p2Y - s2Y
      length2 = @m_u2.Length()
      if length2 > b2Settings.b2_linearSlop
        @m_u2.x *= 1.0 / length2
        @m_u2.y *= 1.0 / length2
      else
        @m_u2.SetZero()
      C = @m_maxLength2 - length2
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_limitMass2 * C)
      p2X = (-impulse * @m_u2.x)
      p2Y = (-impulse * @m_u2.y)
      bB.m_sweep.c.x += bB.m_invMass * p2X
      bB.m_sweep.c.y += bB.m_invMass * p2Y
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X)
      bB.SynchronizeTransform()
    linearError < b2Settings.b2_linearSlop

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength = 2.0
    return

  Box2D.inherit b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2PulleyJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2PulleyJointDef.b2PulleyJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @groundAnchorA = new b2Vec2()
    @groundAnchorB = new b2Vec2()
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2PulleyJointDef::b2PulleyJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_pulleyJoint
    @groundAnchorA.Set (-1.0), 1.0
    @groundAnchorB.Set 1.0, 1.0
    @localAnchorA.Set (-1.0), 0.0
    @localAnchorB.Set 1.0, 0.0
    @lengthA = 0.0
    @maxLengthA = 0.0
    @lengthB = 0.0
    @maxLengthB = 0.0
    @ratio = 1.0
    @collideConnected = true
    return

  b2PulleyJointDef::Initialize = (bA, bB, gaA, gaB, anchorA, anchorB, r) ->
    r = 0  if r is `undefined`
    @bodyA = bA
    @bodyB = bB
    @groundAnchorA.SetV gaA
    @groundAnchorB.SetV gaB
    @localAnchorA = @bodyA.GetLocalPoint(anchorA)
    @localAnchorB = @bodyB.GetLocalPoint(anchorB)
    d1X = anchorA.x - gaA.x
    d1Y = anchorA.y - gaA.y
    @lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y)
    d2X = anchorB.x - gaB.x
    d2Y = anchorB.y - gaB.y
    @lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y)
    @ratio = r
    C = @lengthA + @ratio * @lengthB
    @maxLengthA = C - @ratio * b2PulleyJoint.b2_minPulleyLength
    @maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / @ratio
    return

  Box2D.inherit b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint
  b2RevoluteJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2RevoluteJoint.b2RevoluteJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @K = new b2Mat22()
    @K1 = new b2Mat22()
    @K2 = new b2Mat22()
    @K3 = new b2Mat22()
    @impulse3 = new b2Vec3()
    @impulse2 = new b2Vec2()
    @reduced = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_impulse = new b2Vec3()
    @m_mass = new b2Mat33()
    return

  b2RevoluteJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2RevoluteJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2RevoluteJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2RevoluteJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.z

  b2RevoluteJoint::GetJointAngle = ->
    @m_bodyB.m_sweep.a - @m_bodyA.m_sweep.a - @m_referenceAngle

  b2RevoluteJoint::GetJointSpeed = ->
    @m_bodyB.m_angularVelocity - @m_bodyA.m_angularVelocity

  b2RevoluteJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2RevoluteJoint::EnableLimit = (flag) ->
    @m_enableLimit = flag
    return

  b2RevoluteJoint::GetLowerLimit = ->
    @m_lowerAngle

  b2RevoluteJoint::GetUpperLimit = ->
    @m_upperAngle

  b2RevoluteJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_lowerAngle = lower
    @m_upperAngle = upper
    return

  b2RevoluteJoint::IsMotorEnabled = ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor

  b2RevoluteJoint::EnableMotor = (flag) ->
    @m_enableMotor = flag
    return

  b2RevoluteJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2RevoluteJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2RevoluteJoint::SetMaxMotorTorque = (torque) ->
    torque = 0  if torque is `undefined`
    @m_maxMotorTorque = torque
    return

  b2RevoluteJoint::GetMotorTorque = ->
    @m_maxMotorTorque

  b2RevoluteJoint::b2RevoluteJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_motorImpulse = 0.0
    @m_lowerAngle = def.lowerAngle
    @m_upperAngle = def.upperAngle
    @m_maxMotorTorque = def.maxMotorTorque
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    return

  b2RevoluteJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    
    ###*
    todo: fix this
    ###
    
    #if (this.m_enableMotor || this.m_enableLimit) {}
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
    m1 = bA.m_invMass
    m2 = bB.m_invMass
    i1 = bA.m_invI
    i2 = bB.m_invI
    @m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2
    @m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2
    @m_mass.col3.x = (-r1Y * i1) - r2Y * i2
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2
    @m_mass.col3.y = r1X * i1 + r2X * i2
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = i1 + i2
    @m_motorMass = 1.0 / (i1 + i2)
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if @m_enableLimit
      jointAngle = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
      if b2Math.Abs(@m_upperAngle - @m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointAngle <= @m_lowerAngle
        @m_impulse.z = 0.0  unless @m_limitState is b2Joint.e_atLowerLimit
        @m_limitState = b2Joint.e_atLowerLimit
      else if jointAngle >= @m_upperAngle
        @m_impulse.z = 0.0  unless @m_limitState is b2Joint.e_atUpperLimit
        @m_limitState = b2Joint.e_atUpperLimit
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.z = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x
      PY = @m_impulse.y
      bA.m_linearVelocity.x -= m1 * PX
      bA.m_linearVelocity.y -= m1 * PY
      bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + @m_motorImpulse + @m_impulse.z)
      bB.m_linearVelocity.x += m2 * PX
      bB.m_linearVelocity.y += m2 * PY
      bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + @m_motorImpulse + @m_impulse.z)
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2RevoluteJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    newImpulse = 0
    r1X = 0
    r1Y = 0
    r2X = 0
    r2Y = 0
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    m1 = bA.m_invMass
    m2 = bB.m_invMass
    i1 = bA.m_invI
    i2 = bB.m_invI
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = w2 - w1 - @m_motorSpeed
      impulse = @m_motorMass * (-Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorTorque
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      w1 -= i1 * impulse
      w2 += i2 * impulse
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
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
      Cdot1X = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y)
      Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X)
      Cdot2 = w2 - w1
      @m_mass.Solve33 @impulse3, (-Cdot1X), (-Cdot1Y), (-Cdot2)
      if @m_limitState is b2Joint.e_equalLimits
        @m_impulse.Add @impulse3
      else if @m_limitState is b2Joint.e_atLowerLimit
        newImpulse = @m_impulse.z + @impulse3.z
        if newImpulse < 0.0
          @m_mass.Solve22 @reduced, (-Cdot1X), (-Cdot1Y)
          @impulse3.x = @reduced.x
          @impulse3.y = @reduced.y
          @impulse3.z = (-@m_impulse.z)
          @m_impulse.x += @reduced.x
          @m_impulse.y += @reduced.y
          @m_impulse.z = 0.0
      else if @m_limitState is b2Joint.e_atUpperLimit
        newImpulse = @m_impulse.z + @impulse3.z
        if newImpulse > 0.0
          @m_mass.Solve22 @reduced, (-Cdot1X), (-Cdot1Y)
          @impulse3.x = @reduced.x
          @impulse3.y = @reduced.y
          @impulse3.z = (-@m_impulse.z)
          @m_impulse.x += @reduced.x
          @m_impulse.y += @reduced.y
          @m_impulse.z = 0.0
      v1.x -= m1 * @impulse3.x
      v1.y -= m1 * @impulse3.y
      w1 -= i1 * (r1X * @impulse3.y - r1Y * @impulse3.x + @impulse3.z)
      v2.x += m2 * @impulse3.x
      v2.y += m2 * @impulse3.y
      w2 += i2 * (r2X * @impulse3.y - r2Y * @impulse3.x + @impulse3.z)
    else
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
      CdotX = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y)
      CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X)
      @m_mass.Solve22 @impulse2, (-CdotX), (-CdotY)
      @m_impulse.x += @impulse2.x
      @m_impulse.y += @impulse2.y
      v1.x -= m1 * @impulse2.x
      v1.y -= m1 * @impulse2.y
      w1 -= i1 * (r1X * @impulse2.y - r1Y * @impulse2.x)
      v2.x += m2 * @impulse2.x
      v2.y += m2 * @impulse2.y
      w2 += i2 * (r2X * @impulse2.y - r2Y * @impulse2.x)
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2RevoluteJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    oldLimitImpulse = 0
    C = 0
    tMat = undefined
    bA = @m_bodyA
    bB = @m_bodyB
    angularError = 0.0
    positionError = 0.0
    tX = 0
    impulseX = 0
    impulseY = 0
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      angle = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
      limitImpulse = 0.0
      if @m_limitState is b2Joint.e_equalLimits
        C = b2Math.Clamp(angle - @m_lowerAngle, (-b2Settings.b2_maxAngularCorrection), b2Settings.b2_maxAngularCorrection)
        limitImpulse = (-@m_motorMass * C)
        angularError = b2Math.Abs(C)
      else if @m_limitState is b2Joint.e_atLowerLimit
        C = angle - @m_lowerAngle
        angularError = (-C)
        C = b2Math.Clamp(C + b2Settings.b2_angularSlop, (-b2Settings.b2_maxAngularCorrection), 0.0)
        limitImpulse = (-@m_motorMass * C)
      else if @m_limitState is b2Joint.e_atUpperLimit
        C = angle - @m_upperAngle
        angularError = C
        C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection)
        limitImpulse = (-@m_motorMass * C)
      bA.m_sweep.a -= bA.m_invI * limitImpulse
      bB.m_sweep.a += bB.m_invI * limitImpulse
      bA.SynchronizeTransform()
      bB.SynchronizeTransform()
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
    CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    CLengthSquared = CX * CX + CY * CY
    CLength = Math.sqrt(CLengthSquared)
    positionError = CLength
    invMass1 = bA.m_invMass
    invMass2 = bB.m_invMass
    invI1 = bA.m_invI
    invI2 = bB.m_invI
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop
    if CLengthSquared > k_allowedStretch * k_allowedStretch
      uX = CX / CLength
      uY = CY / CLength
      k = invMass1 + invMass2
      m = 1.0 / k
      impulseX = m * (-CX)
      impulseY = m * (-CY)
      k_beta = 0.5
      bA.m_sweep.c.x -= k_beta * invMass1 * impulseX
      bA.m_sweep.c.y -= k_beta * invMass1 * impulseY
      bB.m_sweep.c.x += k_beta * invMass2 * impulseX
      bB.m_sweep.c.y += k_beta * invMass2 * impulseY
      CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
      CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @K1.col1.x = invMass1 + invMass2
    @K1.col2.x = 0.0
    @K1.col1.y = 0.0
    @K1.col2.y = invMass1 + invMass2
    @K2.col1.x = invI1 * r1Y * r1Y
    @K2.col2.x = (-invI1 * r1X * r1Y)
    @K2.col1.y = (-invI1 * r1X * r1Y)
    @K2.col2.y = invI1 * r1X * r1X
    @K3.col1.x = invI2 * r2Y * r2Y
    @K3.col2.x = (-invI2 * r2X * r2Y)
    @K3.col1.y = (-invI2 * r2X * r2Y)
    @K3.col2.y = invI2 * r2X * r2X
    @K.SetM @K1
    @K.AddM @K2
    @K.AddM @K3
    @K.Solve b2RevoluteJoint.tImpulse, (-CX), (-CY)
    impulseX = b2RevoluteJoint.tImpulse.x
    impulseY = b2RevoluteJoint.tImpulse.y
    bA.m_sweep.c.x -= bA.m_invMass * impulseX
    bA.m_sweep.c.y -= bA.m_invMass * impulseY
    bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX)
    bB.m_sweep.c.x += bB.m_invMass * impulseX
    bB.m_sweep.c.y += bB.m_invMass * impulseY
    bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    positionError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2()
    return

  Box2D.inherit b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2RevoluteJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2RevoluteJointDef.b2RevoluteJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2RevoluteJointDef::b2RevoluteJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_revoluteJoint
    @localAnchorA.Set 0.0, 0.0
    @localAnchorB.Set 0.0, 0.0
    @referenceAngle = 0.0
    @lowerAngle = 0.0
    @upperAngle = 0.0
    @maxMotorTorque = 0.0
    @motorSpeed = 0.0
    @enableLimit = false
    @enableMotor = false
    return

  b2RevoluteJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  Box2D.inherit b2WeldJoint, Box2D.Dynamics.Joints.b2Joint
  b2WeldJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2WeldJoint.b2WeldJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_impulse = new b2Vec3()
    @m_mass = new b2Mat33()
    return

  b2WeldJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  b2WeldJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  b2WeldJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2WeldJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.z

  b2WeldJoint::b2WeldJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_mass = new b2Mat33()
    return

  b2WeldJoint::InitVelocityConstraints = (step) ->
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

  b2WeldJoint::SolveVelocityConstraints = (step) ->
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

  b2WeldJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
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

  Box2D.inherit b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2WeldJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2WeldJointDef.b2WeldJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2WeldJointDef::b2WeldJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_weldJoint
    @referenceAngle = 0.0
    return

  b2WeldJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  return
)()
