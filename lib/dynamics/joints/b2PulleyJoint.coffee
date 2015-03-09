Box2D = require('../../index')

b2Joint       = Box2D.Dynamics.Joints.b2Joint
b2Vec2        = Box2D.Common.Math.b2Vec2
b2Mat22       = Box2D.Common.Math.b2Mat22
b2Vec3        = Box2D.Common.Math.b2Vec3
b2Math        = Box2D.Common.Math.b2Math

class Box2D.Dynamics.Joints.b2PulleyJoint extends b2Joint

  @b2_minPulleyLength       = 2.0


  m_localXAxis1             : null
  m_localYAxis1             : null
  m_u1                      : null
  m_u2                      : null
  m_ground                  : null
  m_ratio                   : 0.0
  m_constant                : 0.0
  m_maxLength1              : 0.0
  m_maxLength2              : 0.0
  m_impulse                 : 0.0
  m_limitImpulse1           : 0.0
  m_limitImpulse2           : 0.0


  constructor: (def) ->
    super(def)
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u1 = new b2Vec2()
    @m_u2 = new b2Vec2()
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

  GetAnchorA: ->
    return @m_bodyA.GetWorldPoint @m_localAnchor1

  GetAnchorB: ->
    return @m_bodyB.GetWorldPoint @m_localAnchor2

  GetReactionForce: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return new b2Vec2(inv_dt * @m_impulse * @m_u2.x, inv_dt * @m_impulse * @m_u2.y)

  GetReactionTorque: (inv_dt) ->
    inv_dt = 0  if inv_dt is undefined
    return 0.0

  GetGroundAnchorA: ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor1
    return a

  GetGroundAnchorB: ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor2
    return a

  GetLength1: ->
    p = @m_bodyA.GetWorldPoint(@m_localAnchor1)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    dX = p.x - sX
    dY = p.y - sY
    return Math.sqrt dX * dX + dY * dY

  GetLength2: ->
    p = @m_bodyB.GetWorldPoint(@m_localAnchor2)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    dX = p.x - sX
    dY = p.y - sY
    Math.sqrt dX * dX + dY * dY

  GetRatio: ->
    return @m_ratio


  InitVelocityConstraints: (step) ->
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

  SolveVelocityConstraints: (step) ->
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

  SolvePositionConstraints: (baumgarte) ->
    baumgarte = 0  if baumgarte is undefined
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
    return linearError < b2Settings.b2_linearSlop

