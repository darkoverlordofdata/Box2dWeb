class Box2D.Dynamics.b2Island

  m_bodies            : null
  m_contacts          : null
  m_joints            : null
  m_bodyCapacity      : 0
  m_contactCapacity   : 0
  m_jointCapacity     : 0
  m_bodyCount         : 0
  m_contactCount      : 0
  m_jointCount        : 0
  m_allocator         : null
  m_listener          : null
  m_contactSolver     : null
  
  
  constructor: ->
    @m_bodies = new Array()
    @m_contacts = new Array()
    @m_joints = new Array()
    return

  Initialize: (bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) ->
    bodyCapacity = 0  if bodyCapacity is undefined
    contactCapacity = 0  if contactCapacity is undefined
    jointCapacity = 0  if jointCapacity is undefined
    i = 0
    @m_bodyCapacity = bodyCapacity
    @m_contactCapacity = contactCapacity
    @m_jointCapacity = jointCapacity
    @m_bodyCount = 0
    @m_contactCount = 0
    @m_jointCount = 0
    @m_allocator = allocator
    @m_listener = listener
    @m_contactSolver = contactSolver
    i = @m_bodies.length
    while i < bodyCapacity
      @m_bodies[i] = null
      i++
    i = @m_contacts.length
    while i < contactCapacity
      @m_contacts[i] = null
      i++
    i = @m_joints.length
    while i < jointCapacity
      @m_joints[i] = null
      i++
    return

  Clear: ->
    @m_bodyCount = 0
    @m_contactCount = 0
    @m_jointCount = 0
    return

  Solve: (step, gravity, allowSleep) ->
    i = 0
    j = 0
    b = undefined
    joint = undefined
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  unless b.GetType() is b2Body.b2_dynamicBody
      b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x)
      b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y)
      b.m_angularVelocity += step.dt * b.m_invI * b.m_torque
      b.m_linearVelocity.Multiply b2Math.Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0)
      b.m_angularVelocity *= b2Math.Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0)
      ++i
    @m_contactSolver.Initialize step, @m_contacts, @m_contactCount, @m_allocator
    contactSolver = @m_contactSolver
    contactSolver.InitVelocityConstraints step
    i = 0
    while i < @m_jointCount
      joint = @m_joints[i]
      joint.InitVelocityConstraints step
      ++i
    i = 0
    while i < step.velocityIterations
      j = 0
      while j < @m_jointCount
        joint = @m_joints[j]
        joint.SolveVelocityConstraints step
        ++j
      contactSolver.SolveVelocityConstraints()
      ++i
    i = 0
    while i < @m_jointCount
      joint = @m_joints[i]
      joint.FinalizeVelocityConstraints()
      ++i
    contactSolver.FinalizeVelocityConstraints()
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  if b.GetType() is b2Body.b2_staticBody
      translationX = step.dt * b.m_linearVelocity.x
      translationY = step.dt * b.m_linearVelocity.y
      if (translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared
        b.m_linearVelocity.Normalize()
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt
      rotation = step.dt * b.m_angularVelocity
      if rotation * rotation > b2Settings.b2_maxRotationSquared
        if b.m_angularVelocity < 0.0
          b.m_angularVelocity = (-b2Settings.b2_maxRotation * step.inv_dt)
        else
          b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt
      b.m_sweep.c0.SetV b.m_sweep.c
      b.m_sweep.a0 = b.m_sweep.a
      b.m_sweep.c.x += step.dt * b.m_linearVelocity.x
      b.m_sweep.c.y += step.dt * b.m_linearVelocity.y
      b.m_sweep.a += step.dt * b.m_angularVelocity
      b.SynchronizeTransform()
      ++i
    i = 0
    while i < step.positionIterations
      contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
      jointsOkay = true
      j = 0
      while j < @m_jointCount
        joint = @m_joints[j]
        jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
        jointsOkay = jointsOkay and jointOkay
        ++j
      break  if contactsOkay and jointsOkay
      ++i
    @Report contactSolver.m_constraints
    if allowSleep
      minSleepTime = Number.MAX_VALUE
      linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance
      angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance
      i = 0
      while i < @m_bodyCount
        b = @m_bodies[i]
        continue  if b.GetType() is b2Body.b2_staticBody
        if (b.m_flags & b2Body.e_allowSleepFlag) is 0
          b.m_sleepTime = 0.0
          minSleepTime = 0.0
        if (b.m_flags & b2Body.e_allowSleepFlag) is 0 or b.m_angularVelocity * b.m_angularVelocity > angTolSqr or b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr
          b.m_sleepTime = 0.0
          minSleepTime = 0.0
        else
          b.m_sleepTime += step.dt
          minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime)
        ++i
      if minSleepTime >= b2Settings.b2_timeToSleep
        i = 0
        while i < @m_bodyCount
          b = @m_bodies[i]
          b.SetAwake false
          ++i
    return

  SolveTOI: (subStep) ->
    i = 0
    j = 0
    @m_contactSolver.Initialize subStep, @m_contacts, @m_contactCount, @m_allocator
    contactSolver = @m_contactSolver
    i = 0
    while i < @m_jointCount
      @m_joints[i].InitVelocityConstraints subStep
      ++i
    i = 0
    while i < subStep.velocityIterations
      contactSolver.SolveVelocityConstraints()
      j = 0
      while j < @m_jointCount
        @m_joints[j].SolveVelocityConstraints subStep
        ++j
      ++i
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  if b.GetType() is b2Body.b2_staticBody
      translationX = subStep.dt * b.m_linearVelocity.x
      translationY = subStep.dt * b.m_linearVelocity.y
      if (translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared
        b.m_linearVelocity.Normalize()
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt
      rotation = subStep.dt * b.m_angularVelocity
      if rotation * rotation > b2Settings.b2_maxRotationSquared
        if b.m_angularVelocity < 0.0
          b.m_angularVelocity = (-b2Settings.b2_maxRotation * subStep.inv_dt)
        else
          b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt
      b.m_sweep.c0.SetV b.m_sweep.c
      b.m_sweep.a0 = b.m_sweep.a
      b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x
      b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y
      b.m_sweep.a += subStep.dt * b.m_angularVelocity
      b.SynchronizeTransform()
      ++i
    k_toiBaumgarte = 0.75
    i = 0
    while i < subStep.positionIterations
      contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte)
      jointsOkay = true
      j = 0
      while j < @m_jointCount
        jointOkay = @m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
        jointsOkay = jointsOkay and jointOkay
        ++j
      break  if contactsOkay and jointsOkay
      ++i
    @Report contactSolver.m_constraints
    return

  Report: (constraints) ->
    return  unless @m_listener?
    i = 0

    while i < @m_contactCount
      c = @m_contacts[i]
      cc = constraints[i]
      j = 0

      while j < cc.pointCount
        b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse
        b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse
        ++j
      @m_listener.PostSolve c, b2Island.s_impulse
      ++i
    return

  AddBody: (body) ->
    body.m_islandIndex = @m_bodyCount
    @m_bodies[@m_bodyCount++] = body
    return

  AddContact: (contact) ->
    @m_contacts[@m_contactCount++] = contact
    return

  AddJoint: (joint) ->
    @m_joints[@m_jointCount++] = joint
    return

  @s_impulse = new b2ContactImpulse()

