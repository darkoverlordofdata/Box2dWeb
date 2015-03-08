Box2D = require('../../index')

b2Joint       = Box2D.Dynamics.Joints.b2Joint
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22
b2Vec3        = Box2D.Common.Math.b2Vec3
b2Math        = Box2D.Common.Math.b2Math

class Box2D.Dynamics.Joints.b2RevoluteJoint extends b2Joint

  @tImpulse               = new b2Vec2()

  K                       : null
  K1                      : null
  K2                      : null
  K3                      : null
  impulse2                : null
  impulse3                : null
  reduced                 : null
  m_impulse               : null
  m_mass                  : 0.0
  m_referenceAngle        : 0.0
  m_motorImpulse          : 0.0
  m_lowerAngle            : 0.0
  m_upperAngle            : 0.0
  m_maxMotorTorque        : 0.0
  m_motorSpeed            : 0.0
  m_enableLimit           : 0.0
  m_enableMotor           : false
  m_limitState            : b2Joint.e_inactiveLimit

  constructor: (def) ->
    super def
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
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_lowerAngle = def.lowerAngle
    @m_upperAngle = def.upperAngle
    @m_maxMotorTorque = def.maxMotorTorque
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    return

  GetAnchorA: ->
    return @m_bodyA.GetWorldPoint @m_localAnchor1

  GetAnchorB: ->
    return @m_bodyB.GetWorldPoint @m_localAnchor2

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return inv_dt * @m_impulse.z

  GetJointAngle: ->
    return @m_bodyB.m_sweep.a - @m_bodyA.m_sweep.a - @m_referenceAngle

  GetJointSpeed: ->
    return @m_bodyB.m_angularVelocity - @m_bodyA.m_angularVelocity

  IsLimitEnabled: ->
    return @m_enableLimit

  EnableLimit: (flag) ->
    @m_enableLimit = flag
    return

  GetLowerLimit: ->
    return @m_lowerAngle

  GetUpperLimit: ->
    return @m_upperAngle

  SetLimits: (lower, upper) ->
    lower = 0  if lower is undefined
    upper = 0  if upper is undefined
    @m_lowerAngle = lower
    @m_upperAngle = upper
    return

  IsMotorEnabled: ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    return @m_enableMotor

  EnableMotor: (flag) ->
    @m_enableMotor = flag
    return

  SetMotorSpeed: (speed) ->
    speed = 0  if speed is undefined
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  GetMotorSpeed: ->
    return @m_motorSpeed

  SetMaxMotorTorque: (torque) ->
    torque = 0  if torque is undefined
    @m_maxMotorTorque = torque
    return

  GetMotorTorque: ->
    return @m_maxMotorTorque


  InitVelocityConstraints: (step) ->
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

  SolveVelocityConstraints: (step) ->
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

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
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
    return positionError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

