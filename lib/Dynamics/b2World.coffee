class Box2D.Dynamics.b2World

  @s_timestep2            = new b2TimeStep()
  @s_xf                   = new b2Transform()
  @s_backupA              = new b2Sweep()
  @s_backupB              = new b2Sweep()
  @s_timestep             = new b2TimeStep()
  @s_queue                = new Array()
  @s_jointColor           = new b2Color(0.5, 0.8, 0.8)
  @e_newFixture           = 0x0001
  @e_locked               = 0x0002


  s_stack                 : null
  m_contactManager        : null
  m_contactSolver         : null
  m_island                : null
  m_destructionListener   : null
  m_debugDraw             : null
  m_bodyList              : null
  m_contactList           : null
  m_jointList             : null
  m_controllerList        : null
  m_bodyCount             : 0
  m_contactCount          : 0
  m_jointCount            : 0
  m_controllerCount       : 0
  m_allowSleep            : null
  m_gravity               : null
  m_inv_dt0               : 0.0
  m_groundBody            : null

  constructor: (gravity, doSleep) ->
    @s_stack = new Array()
    @m_contactManager = new b2ContactManager()
    @m_contactSolver = new b2ContactSolver()
    @m_island = new b2Island()
    b2World.m_warmStarting = true
    b2World.m_continuousPhysics = true
    @m_allowSleep = doSleep
    @m_gravity = gravity
    @m_contactManager.m_world = this
    bd = new b2BodyDef()
    @m_groundBody = @CreateBody(bd)
    return

  SetDestructionListener: (listener) ->
    @m_destructionListener = listener
    return

  SetContactFilter: (filter) ->
    @m_contactManager.m_contactFilter = filter
    return

  SetContactListener: (listener) ->
    @m_contactManager.m_contactListener = listener
    return

  SetDebugDraw: (debugDraw) ->
    @m_debugDraw = debugDraw
    return

  SetBroadPhase: (broadPhase) ->
    oldBroadPhase = @m_contactManager.m_broadPhase
    @m_contactManager.m_broadPhase = broadPhase
    b = @m_bodyList

    while b
      f = b.m_fixtureList

      while f
        f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f)
        f = f.m_next
      b = b.m_next
    return

  Validate: ->
    @m_contactManager.m_broadPhase.Validate()
    return

  GetProxyCount: ->
    @m_contactManager.m_broadPhase.GetProxyCount()

  CreateBody: (def) ->
    return null  if @IsLocked() is true
    b = new b2Body(def, this)
    b.m_prev = null
    b.m_next = @m_bodyList
    @m_bodyList.m_prev = b  if @m_bodyList
    @m_bodyList = b
    ++@m_bodyCount
    b

  DestroyBody: (b) ->
    return  if @IsLocked() is true
    jn = b.m_jointList
    while jn
      jn0 = jn
      jn = jn.next
      @m_destructionListener.SayGoodbyeJoint jn0.joint  if @m_destructionListener
      @DestroyJoint jn0.joint
    coe = b.m_controllerList
    while coe
      coe0 = coe
      coe = coe.nextController
      coe0.controller.RemoveBody b
    ce = b.m_contactList
    while ce
      ce0 = ce
      ce = ce.next
      @m_contactManager.Destroy ce0.contact
    b.m_contactList = null
    f = b.m_fixtureList
    while f
      f0 = f
      f = f.m_next
      @m_destructionListener.SayGoodbyeFixture f0  if @m_destructionListener
      f0.DestroyProxy @m_contactManager.m_broadPhase
      f0.Destroy()
    b.m_fixtureList = null
    b.m_fixtureCount = 0
    b.m_prev.m_next = b.m_next  if b.m_prev
    b.m_next.m_prev = b.m_prev  if b.m_next
    @m_bodyList = b.m_next  if b is @m_bodyList
    --@m_bodyCount
    return

  CreateJoint: (def) ->
    j = b2Joint.Create(def, null)
    j.m_prev = null
    j.m_next = @m_jointList
    @m_jointList.m_prev = j  if @m_jointList
    @m_jointList = j
    ++@m_jointCount
    j.m_edgeA.joint = j
    j.m_edgeA.other = j.m_bodyB
    j.m_edgeA.prev = null
    j.m_edgeA.next = j.m_bodyA.m_jointList
    j.m_bodyA.m_jointList.prev = j.m_edgeA  if j.m_bodyA.m_jointList
    j.m_bodyA.m_jointList = j.m_edgeA
    j.m_edgeB.joint = j
    j.m_edgeB.other = j.m_bodyA
    j.m_edgeB.prev = null
    j.m_edgeB.next = j.m_bodyB.m_jointList
    j.m_bodyB.m_jointList.prev = j.m_edgeB  if j.m_bodyB.m_jointList
    j.m_bodyB.m_jointList = j.m_edgeB
    bodyA = def.bodyA
    bodyB = def.bodyB
    if def.collideConnected is false
      edge = bodyB.GetContactList()
      while edge
        edge.contact.FlagForFiltering()  if edge.other is bodyA
        edge = edge.next
    j

  DestroyJoint: (j) ->
    collideConnected = j.m_collideConnected
    j.m_prev.m_next = j.m_next  if j.m_prev
    j.m_next.m_prev = j.m_prev  if j.m_next
    @m_jointList = j.m_next  if j is @m_jointList
    bodyA = j.m_bodyA
    bodyB = j.m_bodyB
    bodyA.SetAwake true
    bodyB.SetAwake true
    j.m_edgeA.prev.next = j.m_edgeA.next  if j.m_edgeA.prev
    j.m_edgeA.next.prev = j.m_edgeA.prev  if j.m_edgeA.next
    bodyA.m_jointList = j.m_edgeA.next  if j.m_edgeA is bodyA.m_jointList
    j.m_edgeA.prev = null
    j.m_edgeA.next = null
    j.m_edgeB.prev.next = j.m_edgeB.next  if j.m_edgeB.prev
    j.m_edgeB.next.prev = j.m_edgeB.prev  if j.m_edgeB.next
    bodyB.m_jointList = j.m_edgeB.next  if j.m_edgeB is bodyB.m_jointList
    j.m_edgeB.prev = null
    j.m_edgeB.next = null
    b2Joint.Destroy j, null
    --@m_jointCount
    if collideConnected is false
      edge = bodyB.GetContactList()
      while edge
        edge.contact.FlagForFiltering()  if edge.other is bodyA
        edge = edge.next
    return

  AddController: (c) ->
    c.m_next = @m_controllerList
    c.m_prev = null
    @m_controllerList = c
    c.m_world = this
    @m_controllerCount++
    c

  RemoveController: (c) ->
    c.m_prev.m_next = c.m_next  if c.m_prev
    c.m_next.m_prev = c.m_prev  if c.m_next
    @m_controllerList = c.m_next  if @m_controllerList is c
    @m_controllerCount--
    return

  CreateController: (controller) ->
    throw new Error("Controller can only be a member of one world")  unless controller.m_world is this
    controller.m_next = @m_controllerList
    controller.m_prev = null
    @m_controllerList.m_prev = controller  if @m_controllerList
    @m_controllerList = controller
    ++@m_controllerCount
    controller.m_world = this
    controller

  DestroyController: (controller) ->
    controller.Clear()
    controller.m_next.m_prev = controller.m_prev  if controller.m_next
    controller.m_prev.m_next = controller.m_next  if controller.m_prev
    @m_controllerList = controller.m_next  if controller is @m_controllerList
    --@m_controllerCount
    return

  SetWarmStarting: (flag) ->
    b2World.m_warmStarting = flag
    return

  SetContinuousPhysics: (flag) ->
    b2World.m_continuousPhysics = flag
    return

  GetBodyCount: ->
    @m_bodyCount

  GetJointCount: ->
    @m_jointCount

  GetContactCount: ->
    @m_contactCount

  SetGravity: (gravity) ->
    @m_gravity = gravity
    return

  GetGravity: ->
    @m_gravity

  GetGroundBody: ->
    @m_groundBody

  Step: (dt, velocityIterations, positionIterations) ->
    dt = 0  if dt is undefined
    velocityIterations = 0  if velocityIterations is undefined
    positionIterations = 0  if positionIterations is undefined
    if @m_flags & b2World.e_newFixture
      @m_contactManager.FindNewContacts()
      @m_flags &= ~b2World.e_newFixture

    @m_flags |= b2World.e_locked
    step = b2World.s_timestep2
    step.dt = dt
    step.velocityIterations = velocityIterations
    step.positionIterations = positionIterations
    if dt > 0.0
      step.inv_dt = 1.0 / dt
    else
      step.inv_dt = 0.0
    step.dtRatio = @m_inv_dt0 * dt
    step.warmStarting = b2World.m_warmStarting
    @m_contactManager.Collide()
    @Solve step  if step.dt > 0.0
    @SolveTOI step  if b2World.m_continuousPhysics and step.dt > 0.0
    @m_inv_dt0 = step.inv_dt  if step.dt > 0.0
    @m_flags &= ~b2World.e_locked
    return

  ClearForces: ->
    body = @m_bodyList

    while body
      body.m_force.SetZero()
      body.m_torque = 0.0
      body = body.m_next
    return

  DrawDebugData: ->
    return  unless @m_debugDraw?
    @m_debugDraw.m_sprite.graphics.clear()
    flags = @m_debugDraw.GetFlags()
    i = 0
    b = undefined
    f = undefined
    s = undefined
    j = undefined
    bp = undefined
    invQ = new b2Vec2
    x1 = new b2Vec2
    x2 = new b2Vec2
    xf = undefined
    b1 = new b2AABB()
    b2 = new b2AABB()
    vs = [
      new b2Vec2()
      new b2Vec2()
      new b2Vec2()
      new b2Vec2()
    ]
    color = new b2Color(0, 0, 0)
    if flags & b2DebugDraw.e_shapeBit
      b = @m_bodyList
      while b
        xf = b.m_xf
        f = b.GetFixtureList()
        while f
          s = f.GetShape()
          if b.IsActive() is false
            color.Set 0.5, 0.5, 0.3
            @DrawShape s, xf, color
          else if b.GetType() is b2Body.b2_staticBody
            color.Set 0.5, 0.9, 0.5
            @DrawShape s, xf, color
          else if b.GetType() is b2Body.b2_kinematicBody
            color.Set 0.5, 0.5, 0.9
            @DrawShape s, xf, color
          else if b.IsAwake() is false
            color.Set 0.6, 0.6, 0.6
            @DrawShape s, xf, color
          else
            color.Set 0.9, 0.7, 0.7
            @DrawShape s, xf, color
          f = f.m_next
        b = b.m_next
    if flags & b2DebugDraw.e_jointBit
      j = @m_jointList
      while j
        @DrawJoint j
        j = j.m_next
    if flags & b2DebugDraw.e_controllerBit
      c = @m_controllerList

      while c
        c.Draw @m_debugDraw
        c = c.m_next
    if flags & b2DebugDraw.e_pairBit
      color.Set 0.3, 0.9, 0.9
      contact = @m_contactManager.m_contactList

      while contact
        fixtureA = contact.GetFixtureA()
        fixtureB = contact.GetFixtureB()
        cA = fixtureA.GetAABB().GetCenter()
        cB = fixtureB.GetAABB().GetCenter()
        @m_debugDraw.DrawSegment cA, cB, color
        contact = contact.GetNext()
    if flags & b2DebugDraw.e_aabbBit
      bp = @m_contactManager.m_broadPhase
      vs = [
        new b2Vec2()
        new b2Vec2()
        new b2Vec2()
        new b2Vec2()
      ]
      b = @m_bodyList
      while b
        continue  if b.IsActive() is false
        f = b.GetFixtureList()
        while f
          aabb = bp.GetFatAABB(f.m_proxy)
          vs[0].Set aabb.lowerBound.x, aabb.lowerBound.y
          vs[1].Set aabb.upperBound.x, aabb.lowerBound.y
          vs[2].Set aabb.upperBound.x, aabb.upperBound.y
          vs[3].Set aabb.lowerBound.x, aabb.upperBound.y
          @m_debugDraw.DrawPolygon vs, 4, color
          f = f.GetNext()
        b = b.GetNext()
    if flags & b2DebugDraw.e_centerOfMassBit
      b = @m_bodyList
      while b
        xf = b2World.s_xf
        xf.R = b.m_xf.R
        xf.position = b.GetWorldCenter()
        @m_debugDraw.DrawTransform xf
        b = b.m_next
    return

  QueryAABB: (callback, aabb) ->
    WorldQueryWrapper = (proxy) ->
      callback broadPhase.GetUserData(proxy)
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    broadPhase.Query WorldQueryWrapper, aabb
    return

  QueryShape: (callback, shape, transform) ->
    WorldQueryWrapper = (proxy) ->
      fixture = ((if broadPhase.GetUserData(proxy) instanceof b2Fixture then broadPhase.GetUserData(proxy) else null))
      return callback(fixture)  if b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())
      true
    __this = this
    transform = null  if transform is undefined
    unless transform?
      transform = new b2Transform()
      transform.SetIdentity()
    broadPhase = __this.m_contactManager.m_broadPhase
    aabb = new b2AABB()
    shape.ComputeAABB aabb, transform
    broadPhase.Query WorldQueryWrapper, aabb
    return

  QueryPoint: (callback, p) ->
    WorldQueryWrapper = (proxy) ->
      fixture = ((if broadPhase.GetUserData(proxy) instanceof b2Fixture then broadPhase.GetUserData(proxy) else null))
      return callback(fixture)  if fixture.TestPoint(p)
      true
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    aabb = new b2AABB()
    aabb.lowerBound.Set p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop
    aabb.upperBound.Set p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop
    broadPhase.Query WorldQueryWrapper, aabb
    return

  RayCast: (callback, point1, point2) ->
    RayCastWrapper = (input, proxy) ->
      userData = broadPhase.GetUserData(proxy)
      fixture = ((if userData instanceof b2Fixture then userData else null))
      hit = fixture.RayCast(output, input)
      if hit
        fraction = output.fraction
        point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y)
        return callback(fixture, point, output.normal, fraction)
      input.maxFraction
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    output = new b2RayCastOutput
    input = new b2RayCastInput(point1, point2)
    broadPhase.RayCast RayCastWrapper, input
    return

  RayCastOne: (point1, point2) ->
    RayCastOneWrapper = (fixture, point, normal, fraction) ->
      fraction = 0  if fraction is undefined
      result = fixture
      fraction
    __this = this
    result = undefined
    __this.RayCast RayCastOneWrapper, point1, point2
    result

  RayCastAll: (point1, point2) ->
    RayCastAllWrapper = (fixture, point, normal, fraction) ->
      fraction = 0  if fraction is undefined
      result[result.length] = fixture
      1
    __this = this
    result = new Array()
    __this.RayCast RayCastAllWrapper, point1, point2
    result

  GetBodyList: ->
    @m_bodyList

  GetJointList: ->
    @m_jointList

  GetContactList: ->
    @m_contactList

  IsLocked: ->
    (@m_flags & b2World.e_locked) > 0

  Solve: (step) ->
    b = undefined
    controller = @m_controllerList

    while controller
      controller.Step step
      controller = controller.m_next
    island = @m_island
    island.Initialize @m_bodyCount, @m_contactCount, @m_jointCount, null, @m_contactManager.m_contactListener, @m_contactSolver
    b = @m_bodyList
    while b
      b.m_flags &= ~b2Body.e_islandFlag
      b = b.m_next
    c = @m_contactList

    while c
      c.m_flags &= ~b2Contact.e_islandFlag
      c = c.m_next
    j = @m_jointList

    while j
      j.m_islandFlag = false
      j = j.m_next
    stackSize = parseInt(@m_bodyCount)
    stack = @s_stack
    seed = @m_bodyList

#    `debugger;`

    while seed
      if seed.m_flags & b2Body.e_islandFlag
        seed = seed.m_next
        continue

      if seed.IsAwake() is false or seed.IsActive() is false
        seed = seed.m_next
        continue

      if seed.GetType() is b2Body.b2_staticBody
        seed = seed.m_next
        continue

      island.Clear()
      stackCount = 0
      stack[stackCount++] = seed
      seed.m_flags |= b2Body.e_islandFlag
      while stackCount > 0
        b = stack[--stackCount]
        island.AddBody b
        b.SetAwake true  if b.IsAwake() is false
        continue  if b.GetType() is b2Body.b2_staticBody
        other = undefined
        ce = b.m_contactList

        while ce
          if ce.contact.m_flags & b2Contact.e_islandFlag
            ce = ce.next
            continue

          if ce.contact.IsSensor() is true or ce.contact.IsEnabled() is false or ce.contact.IsTouching() is false
            ce = ce.next
            continue

          island.AddContact ce.contact
          ce.contact.m_flags |= b2Contact.e_islandFlag
          other = ce.other
          if other.m_flags & b2Body.e_islandFlag
            ce = ce.next
            continue

          stack[stackCount++] = other
          other.m_flags |= b2Body.e_islandFlag
          ce = ce.next
        jn = b.m_jointList

        while jn
          if jn.joint.m_islandFlag is true
            jn = jn.next
            continue

          other = jn.other
          if other.IsActive() is false
            jn = jn.next
            continue

          island.AddJoint jn.joint
          jn.joint.m_islandFlag = true
          if other.m_flags & b2Body.e_islandFlag
            jn = jn.next
            continue

          stack[stackCount++] = other
          other.m_flags |= b2Body.e_islandFlag
          jn = jn.next
      island.Solve step, @m_gravity, @m_allowSleep
      i = 0

      while i < island.m_bodyCount
        b = island.m_bodies[i]
        b.m_flags &= ~b2Body.e_islandFlag  if b.GetType() is b2Body.b2_staticBody
        ++i
      seed = seed.m_next
    i = 0
    while i < stack.length
      break  unless stack[i]
      stack[i] = null
      ++i
    b = @m_bodyList
    while b
      if b.IsAwake() is false or b.IsActive() is false
        b = b.m_next
        continue

      if b.GetType() is b2Body.b2_staticBody
        b = b.m_next
        continue

      b.SynchronizeFixtures()
      b = b.m_next
    @m_contactManager.FindNewContacts()
    return

  SolveTOI: (step) ->
    b = undefined
    fA = undefined
    fB = undefined
    bA = undefined
    bB = undefined
    cEdge = undefined
    j = undefined
    island = @m_island
    island.Initialize @m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, @m_contactManager.m_contactListener, @m_contactSolver
    queue = b2World.s_queue
    b = @m_bodyList
    while b
      b.m_flags &= ~b2Body.e_islandFlag
      b.m_sweep.t0 = 0.0
      b = b.m_next
    c = undefined
    c = @m_contactList
    while c
      c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag)
      c = c.m_next
    j = @m_jointList
    while j
      j.m_islandFlag = false
      j = j.m_next
    loop
      minContact = null
      minTOI = 1.0
      c = @m_contactList
      while c
        continue  if c.IsSensor() is true or c.IsEnabled() is false or c.IsContinuous() is false
        toi = 1.0
        if c.m_flags & b2Contact.e_toiFlag
          toi = c.m_toi
        else
          fA = c.m_fixtureA
          fB = c.m_fixtureB
          bA = fA.m_body
          bB = fB.m_body
          continue  if (bA.GetType() isnt b2Body.b2_dynamicBody or bA.IsAwake() is false) and (bB.GetType() isnt b2Body.b2_dynamicBody or bB.IsAwake() is false)
          t0 = bA.m_sweep.t0
          if bA.m_sweep.t0 < bB.m_sweep.t0
            t0 = bB.m_sweep.t0
            bA.m_sweep.Advance t0
          else if bB.m_sweep.t0 < bA.m_sweep.t0
            t0 = bA.m_sweep.t0
            bB.m_sweep.Advance t0
          toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep)
          b2Settings.b2Assert 0.0 <= toi and toi <= 1.0
          if toi > 0.0 and toi < 1.0
            toi = (1.0 - toi) * t0 + toi
            toi = 1  if toi > 1
          c.m_toi = toi
          c.m_flags |= b2Contact.e_toiFlag
        if Number.MIN_VALUE < toi and toi < minTOI
          minContact = c
          minTOI = toi
        c = c.m_next
      break  if not minContact? or 1.0 - 100.0 * Number.MIN_VALUE < minTOI
      fA = minContact.m_fixtureA
      fB = minContact.m_fixtureB
      bA = fA.m_body
      bB = fB.m_body
      b2World.s_backupA.Set bA.m_sweep
      b2World.s_backupB.Set bB.m_sweep
      bA.Advance minTOI
      bB.Advance minTOI
      minContact.Update @m_contactManager.m_contactListener
      minContact.m_flags &= ~b2Contact.e_toiFlag
      if minContact.IsSensor() is true or minContact.IsEnabled() is false
        bA.m_sweep.Set b2World.s_backupA
        bB.m_sweep.Set b2World.s_backupB
        bA.SynchronizeTransform()
        bB.SynchronizeTransform()
        continue
      continue  if minContact.IsTouching() is false
      seed = bA
      seed = bB  unless seed.GetType() is b2Body.b2_dynamicBody
      island.Clear()
      queueStart = 0
      queueSize = 0
      queue[queueStart + queueSize++] = seed
      seed.m_flags |= b2Body.e_islandFlag
      while queueSize > 0
        b = queue[queueStart++]
        --queueSize
        island.AddBody b
        b.SetAwake true  if b.IsAwake() is false
        continue  unless b.GetType() is b2Body.b2_dynamicBody
        cEdge = b.m_contactList
        while cEdge
          break  if island.m_contactCount is island.m_contactCapacity
          continue  if cEdge.contact.m_flags & b2Contact.e_islandFlag
          continue  if cEdge.contact.IsSensor() is true or cEdge.contact.IsEnabled() is false or cEdge.contact.IsTouching() is false
          island.AddContact cEdge.contact
          cEdge.contact.m_flags |= b2Contact.e_islandFlag
          other = cEdge.other
          continue  if other.m_flags & b2Body.e_islandFlag
          unless other.GetType() is b2Body.b2_staticBody
            other.Advance minTOI
            other.SetAwake true
          queue[queueStart + queueSize] = other
          ++queueSize
          other.m_flags |= b2Body.e_islandFlag
          cEdge = cEdge.next
        jEdge = b.m_jointList

        while jEdge
          continue  if island.m_jointCount is island.m_jointCapacity
          continue  if jEdge.joint.m_islandFlag is true
          other = jEdge.other
          continue  if other.IsActive() is false
          island.AddJoint jEdge.joint
          jEdge.joint.m_islandFlag = true
          continue  if other.m_flags & b2Body.e_islandFlag
          unless other.GetType() is b2Body.b2_staticBody
            other.Advance minTOI
            other.SetAwake true
          queue[queueStart + queueSize] = other
          ++queueSize
          other.m_flags |= b2Body.e_islandFlag
          jEdge = jEdge.next
      subStep = b2World.s_timestep
      subStep.warmStarting = false
      subStep.dt = (1.0 - minTOI) * step.dt
      subStep.inv_dt = 1.0 / subStep.dt
      subStep.dtRatio = 0.0
      subStep.velocityIterations = step.velocityIterations
      subStep.positionIterations = step.positionIterations
      island.SolveTOI subStep
      i = 0
      i = 0
      while i < island.m_bodyCount
        b = island.m_bodies[i]
        b.m_flags &= ~b2Body.e_islandFlag
        continue  if b.IsAwake() is false
        continue  unless b.GetType() is b2Body.b2_dynamicBody
        b.SynchronizeFixtures()
        cEdge = b.m_contactList
        while cEdge
          cEdge.contact.m_flags &= ~b2Contact.e_toiFlag
          cEdge = cEdge.next
        ++i
      i = 0
      while i < island.m_contactCount
        c = island.m_contacts[i]
#        c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag)
        c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag)
        ++i
      i = 0
      while i < island.m_jointCount
        j = island.m_joints[i]
        j.m_islandFlag = false
        ++i
      @m_contactManager.FindNewContacts()
    return

  DrawJoint: (joint) ->
    b1 = joint.GetBodyA()
    b2 = joint.GetBodyB()
    xf1 = b1.m_xf
    xf2 = b2.m_xf
    x1 = xf1.position
    x2 = xf2.position
    p1 = joint.GetAnchorA()
    p2 = joint.GetAnchorB()
    color = b2World.s_jointColor
    switch joint.m_type
      when b2Joint.e_distanceJoint
        @m_debugDraw.DrawSegment p1, p2, color
      when b2Joint.e_pulleyJoint
        pulley = ((if joint instanceof b2PulleyJoint then joint else null))
        s1 = pulley.GetGroundAnchorA()
        s2 = pulley.GetGroundAnchorB()
        @m_debugDraw.DrawSegment s1, p1, color
        @m_debugDraw.DrawSegment s2, p2, color
        @m_debugDraw.DrawSegment s1, s2, color
      when b2Joint.e_mouseJoint
        @m_debugDraw.DrawSegment p1, p2, color
      else
        @m_debugDraw.DrawSegment x1, p1, color  unless b1 is @m_groundBody
        @m_debugDraw.DrawSegment p1, p2, color
        @m_debugDraw.DrawSegment x2, p2, color  unless b2 is @m_groundBody
    return

  DrawShape: (shape, xf, color) ->
    switch shape.m_type
      when b2Shape.e_circleShape
        circle = ((if shape instanceof b2CircleShape then shape else null))
        center = b2Math.MulX(xf, circle.m_p)
        radius = circle.m_radius
        axis = xf.R.col1
        @m_debugDraw.DrawSolidCircle center, radius, axis, color
      when b2Shape.e_polygonShape
        i = 0
        poly = ((if shape instanceof b2PolygonShape then shape else null))
        vertexCount = parseInt(poly.GetVertexCount())
        localVertices = poly.GetVertices()
        vertices = new Array(vertexCount)
        i = 0
        while i < vertexCount
          vertices[i] = b2Math.MulX(xf, localVertices[i])
          ++i
        @m_debugDraw.DrawSolidPolygon vertices, vertexCount, color
      when b2Shape.e_edgeShape
        edge = ((if shape instanceof b2EdgeShape then shape else null))
        @m_debugDraw.DrawSegment b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color

