Box2D = require('../../index')

b2Joint = Box2D.Dynamics.Joints.b2Joint

class Box2D.Dynamics.Joints.b2LineJoint extends b2Joint
  
  constructor: (def) ->
    super def
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_localXAxis1 = new b2Vec2()
    @m_localYAxis1 = new b2Vec2()
    @m_axis = new b2Vec2()
    @m_perp = new b2Vec2()
    @m_K = new b2Mat22()
    @m_impulse = new b2Vec2()
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

  GetAnchorA: ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  GetAnchorB: ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    new b2Vec2(inv_dt * (@m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.y) * @m_axis.x), inv_dt * (@m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.y) * @m_axis.y))

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    inv_dt * @m_impulse.y

  GetJointTranslation: ->
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

  GetJointSpeed: ->
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

  IsLimitEnabled: ->
    @m_enableLimit

  EnableLimit: (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableLimit = flag
    return

  GetLowerLimit: ->
    @m_lowerTranslation

  GetUpperLimit: ->
    @m_upperTranslation

  SetLimits: (lower, upper) ->
    lower = 0  if lower is undefined
    upper = 0  if upper is undefined
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_lowerTranslation = lower
    @m_upperTranslation = upper
    return

  IsMotorEnabled: ->
    @m_enableMotor

  EnableMotor: (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor = flag
    return

  SetMotorSpeed: (speed) ->
    speed = 0  if speed is undefined
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  GetMotorSpeed: ->
    @m_motorSpeed

  SetMaxMotorForce: (force) ->
    force = 0  if force is undefined
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_maxMotorForce = force
    return

  GetMaxMotorForce: ->
    @m_maxMotorForce

  GetMotorForce: ->
    @m_motorImpulse


  InitVelocityConstraints: (step) ->
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

  SolveVelocityConstraints: (step) ->
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

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
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

