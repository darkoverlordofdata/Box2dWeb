class Box2D.Dynamics.Joints.b2GearJoint extends b2Joint

  m_groundAnchor1         : null
  m_groundAnchor2         : null
  m_J                     : null
  m_revolute1             : null
  m_prismatic1            : null
  m_revolute2             : null
  m_prismatic2            : null
  m_ground1               : null
  m_ground2               : null
  m_ratio                 : 0
  m_constant              : 0
  m_impulse               : 0.0


  constructor: (def) ->
    super(def)
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_J = new b2Jacobian()
    type1 = parseInt(def.joint1.m_type)
    type2 = parseInt(def.joint2.m_type)
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
    return

  GetAnchorA: ->
    return @m_bodyA.GetWorldPoint @m_localAnchor1

  GetAnchorB: ->
    return @m_bodyB.GetWorldPoint @m_localAnchor2

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return new b2Vec2(inv_dt * @m_impulse * @m_J.linearB.x, inv_dt * @m_impulse * @m_J.linearB.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    tMat = @m_bodyB.m_xf.R
    rX = @m_localAnchor1.x - @m_bodyB.m_sweep.localCenter.x
    rY = @m_localAnchor1.y - @m_bodyB.m_sweep.localCenter.y
    tX = tMat.col1.x * rX + tMat.col2.x * rY
    rY = tMat.col1.y * rX + tMat.col2.y * rY
    rX = tX
    PX = @m_impulse * @m_J.linearB.x
    PY = @m_impulse * @m_J.linearB.y
    return inv_dt * (@m_impulse * @m_J.angularB - rX * PY + rY * PX)

  GetRatio: ->
    return @m_ratio

  SetRatio: (ratio) ->
    ratio = 0  if ratio is undefined
    @m_ratio = ratio
    return

  InitVelocityConstraints: (step) ->
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

  SolveVelocityConstraints: (step) ->
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

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
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
    return linearError < b2Settings.b2_linearSlop

