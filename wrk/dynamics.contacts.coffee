(->
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact
  b2Contact = Box2D.Dynamics.Contacts.b2Contact
  b2ContactConstraint = Box2D.Dynamics.Contacts.b2ContactConstraint
  b2ContactConstraintPoint = Box2D.Dynamics.Contacts.b2ContactConstraintPoint
  b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge
  b2ContactFactory = Box2D.Dynamics.Contacts.b2ContactFactory
  b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister
  b2ContactResult = Box2D.Dynamics.Contacts.b2ContactResult
  b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver
  b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact
  b2NullContact = Box2D.Dynamics.Contacts.b2NullContact
  b2PolyAndCircleContact = Box2D.Dynamics.Contacts.b2PolyAndCircleContact
  b2PolyAndEdgeContact = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact
  b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact
  b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold
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
  b2AABB = Box2D.Collision.b2AABB
  b2Bound = Box2D.Collision.b2Bound
  b2BoundValues = Box2D.Collision.b2BoundValues
  b2Collision = Box2D.Collision.b2Collision
  b2ContactID = Box2D.Collision.b2ContactID
  b2ContactPoint = Box2D.Collision.b2ContactPoint
  b2Distance = Box2D.Collision.b2Distance
  b2DistanceInput = Box2D.Collision.b2DistanceInput
  b2DistanceOutput = Box2D.Collision.b2DistanceOutput
  b2DistanceProxy = Box2D.Collision.b2DistanceProxy
  b2DynamicTree = Box2D.Collision.b2DynamicTree
  b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase
  b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode
  b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair
  b2Manifold = Box2D.Collision.b2Manifold
  b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint
  b2Point = Box2D.Collision.b2Point
  b2RayCastInput = Box2D.Collision.b2RayCastInput
  b2RayCastOutput = Box2D.Collision.b2RayCastOutput
  b2Segment = Box2D.Collision.b2Segment
  b2SeparationFunction = Box2D.Collision.b2SeparationFunction
  b2Simplex = Box2D.Collision.b2Simplex
  b2SimplexCache = Box2D.Collision.b2SimplexCache
  b2SimplexVertex = Box2D.Collision.b2SimplexVertex
  b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact
  b2TOIInput = Box2D.Collision.b2TOIInput
  b2WorldManifold = Box2D.Collision.b2WorldManifold
  ClipVertex = Box2D.Collision.ClipVertex
  Features = Box2D.Collision.Features
  IBroadPhase = Box2D.Collision.IBroadPhase
  Box2D.inherit b2CircleContact, Box2D.Dynamics.Contacts.b2Contact
  b2CircleContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2CircleContact.b2CircleContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2CircleContact.Create = (allocator) ->
    new b2CircleContact()

  b2CircleContact.Destroy = (contact, allocator) ->

  b2CircleContact::Reset = (fixtureA, fixtureB) ->
    @__super.Reset.call this, fixtureA, fixtureB
    return

  b2CircleContact::Evaluate = ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    b2Collision.CollideCircles @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2CircleShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

  b2Contact.b2Contact = ->
    @m_nodeA = new b2ContactEdge()
    @m_nodeB = new b2ContactEdge()
    @m_manifold = new b2Manifold()
    @m_oldManifold = new b2Manifold()
    return

  b2Contact::GetManifold = ->
    @m_manifold

  b2Contact::GetWorldManifold = (worldManifold) ->
    bodyA = @m_fixtureA.GetBody()
    bodyB = @m_fixtureB.GetBody()
    shapeA = @m_fixtureA.GetShape()
    shapeB = @m_fixtureB.GetShape()
    worldManifold.Initialize @m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius
    return

  b2Contact::IsTouching = ->
    (@m_flags & b2Contact.e_touchingFlag) is b2Contact.e_touchingFlag

  b2Contact::IsContinuous = ->
    (@m_flags & b2Contact.e_continuousFlag) is b2Contact.e_continuousFlag

  b2Contact::SetSensor = (sensor) ->
    if sensor
      @m_flags |= b2Contact.e_sensorFlag
    else
      @m_flags &= ~b2Contact.e_sensorFlag
    return

  b2Contact::IsSensor = ->
    (@m_flags & b2Contact.e_sensorFlag) is b2Contact.e_sensorFlag

  b2Contact::SetEnabled = (flag) ->
    if flag
      @m_flags |= b2Contact.e_enabledFlag
    else
      @m_flags &= ~b2Contact.e_enabledFlag
    return

  b2Contact::IsEnabled = ->
    (@m_flags & b2Contact.e_enabledFlag) is b2Contact.e_enabledFlag

  b2Contact::GetNext = ->
    @m_next

  b2Contact::GetFixtureA = ->
    @m_fixtureA

  b2Contact::GetFixtureB = ->
    @m_fixtureB

  b2Contact::FlagForFiltering = ->
    @m_flags |= b2Contact.e_filterFlag
    return

  b2Contact::b2Contact = ->

  b2Contact::Reset = (fixtureA, fixtureB) ->
    fixtureA = null  if fixtureA is `undefined`
    fixtureB = null  if fixtureB is `undefined`
    @m_flags = b2Contact.e_enabledFlag
    if not fixtureA or not fixtureB
      @m_fixtureA = null
      @m_fixtureB = null
      return
    @m_flags |= b2Contact.e_sensorFlag  if fixtureA.IsSensor() or fixtureB.IsSensor()
    bodyA = fixtureA.GetBody()
    bodyB = fixtureB.GetBody()
    @m_flags |= b2Contact.e_continuousFlag  if bodyA.GetType() isnt b2Body.b2_dynamicBody or bodyA.IsBullet() or bodyB.GetType() isnt b2Body.b2_dynamicBody or bodyB.IsBullet()
    @m_fixtureA = fixtureA
    @m_fixtureB = fixtureB
    @m_manifold.m_pointCount = 0
    @m_prev = null
    @m_next = null
    @m_nodeA.contact = null
    @m_nodeA.prev = null
    @m_nodeA.next = null
    @m_nodeA.other = null
    @m_nodeB.contact = null
    @m_nodeB.prev = null
    @m_nodeB.next = null
    @m_nodeB.other = null
    return

  b2Contact::Update = (listener) ->
    tManifold = @m_oldManifold
    @m_oldManifold = @m_manifold
    @m_manifold = tManifold
    @m_flags |= b2Contact.e_enabledFlag
    touching = false
    wasTouching = (@m_flags & b2Contact.e_touchingFlag) is b2Contact.e_touchingFlag
    bodyA = @m_fixtureA.m_body
    bodyB = @m_fixtureB.m_body
    aabbOverlap = @m_fixtureA.m_aabb.TestOverlap(@m_fixtureB.m_aabb)
    if @m_flags & b2Contact.e_sensorFlag
      if aabbOverlap
        shapeA = @m_fixtureA.GetShape()
        shapeB = @m_fixtureB.GetShape()
        xfA = bodyA.GetTransform()
        xfB = bodyB.GetTransform()
        touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB)
      @m_manifold.m_pointCount = 0
    else
      if bodyA.GetType() isnt b2Body.b2_dynamicBody or bodyA.IsBullet() or bodyB.GetType() isnt b2Body.b2_dynamicBody or bodyB.IsBullet()
        @m_flags |= b2Contact.e_continuousFlag
      else
        @m_flags &= ~b2Contact.e_continuousFlag
      if aabbOverlap
        @Evaluate()
        touching = @m_manifold.m_pointCount > 0
        i = 0

        while i < @m_manifold.m_pointCount
          mp2 = @m_manifold.m_points[i]
          mp2.m_normalImpulse = 0.0
          mp2.m_tangentImpulse = 0.0
          id2 = mp2.m_id
          j = 0

          while j < @m_oldManifold.m_pointCount
            mp1 = @m_oldManifold.m_points[j]
            if mp1.m_id.key is id2.key
              mp2.m_normalImpulse = mp1.m_normalImpulse
              mp2.m_tangentImpulse = mp1.m_tangentImpulse
              break
            ++j
          ++i
      else
        @m_manifold.m_pointCount = 0
      unless touching is wasTouching
        bodyA.SetAwake true
        bodyB.SetAwake true
    if touching
      @m_flags |= b2Contact.e_touchingFlag
    else
      @m_flags &= ~b2Contact.e_touchingFlag
    listener.BeginContact this  if wasTouching is false and touching is true
    listener.EndContact this  if wasTouching is true and touching is false
    listener.PreSolve this, @m_oldManifold  if (@m_flags & b2Contact.e_sensorFlag) is 0
    return

  b2Contact::Evaluate = ->

  b2Contact::ComputeTOI = (sweepA, sweepB) ->
    b2Contact.s_input.proxyA.Set @m_fixtureA.GetShape()
    b2Contact.s_input.proxyB.Set @m_fixtureB.GetShape()
    b2Contact.s_input.sweepA = sweepA
    b2Contact.s_input.sweepB = sweepB
    b2Contact.s_input.tolerance = b2Settings.b2_linearSlop
    b2TimeOfImpact.TimeOfImpact b2Contact.s_input

  Box2D.postDefs.push ->
    Box2D.Dynamics.Contacts.b2Contact.e_sensorFlag = 0x0001
    Box2D.Dynamics.Contacts.b2Contact.e_continuousFlag = 0x0002
    Box2D.Dynamics.Contacts.b2Contact.e_islandFlag = 0x0004
    Box2D.Dynamics.Contacts.b2Contact.e_toiFlag = 0x0008
    Box2D.Dynamics.Contacts.b2Contact.e_touchingFlag = 0x0010
    Box2D.Dynamics.Contacts.b2Contact.e_enabledFlag = 0x0020
    Box2D.Dynamics.Contacts.b2Contact.e_filterFlag = 0x0040
    Box2D.Dynamics.Contacts.b2Contact.s_input = new b2TOIInput()
    return

  b2ContactConstraint.b2ContactConstraint = ->
    @localPlaneNormal = new b2Vec2()
    @localPoint = new b2Vec2()
    @normal = new b2Vec2()
    @normalMass = new b2Mat22()
    @K = new b2Mat22()
    return

  b2ContactConstraint::b2ContactConstraint = ->
    @points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @points[i] = new b2ContactConstraintPoint()
      i++
    return

  b2ContactConstraintPoint.b2ContactConstraintPoint = ->
    @localPoint = new b2Vec2()
    @rA = new b2Vec2()
    @rB = new b2Vec2()
    return

  b2ContactEdge.b2ContactEdge = ->

  b2ContactFactory.b2ContactFactory = ->

  b2ContactFactory::b2ContactFactory = (allocator) ->
    @m_allocator = allocator
    @InitializeRegisters()
    return

  b2ContactFactory::AddType = (createFcn, destroyFcn, type1, type2) ->
    type1 = 0  if type1 is `undefined`
    type2 = 0  if type2 is `undefined`
    @m_registers[type1][type2].createFcn = createFcn
    @m_registers[type1][type2].destroyFcn = destroyFcn
    @m_registers[type1][type2].primary = true
    unless type1 is type2
      @m_registers[type2][type1].createFcn = createFcn
      @m_registers[type2][type1].destroyFcn = destroyFcn
      @m_registers[type2][type1].primary = false
    return

  b2ContactFactory::InitializeRegisters = ->
    @m_registers = new Vector(b2Shape.e_shapeTypeCount)
    i = 0

    while i < b2Shape.e_shapeTypeCount
      @m_registers[i] = new Vector(b2Shape.e_shapeTypeCount)
      j = 0

      while j < b2Shape.e_shapeTypeCount
        @m_registers[i][j] = new b2ContactRegister()
        j++
      i++
    @AddType b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape
    @AddType b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape
    @AddType b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape
    @AddType b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape
    @AddType b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape
    return

  b2ContactFactory::Create = (fixtureA, fixtureB) ->
    type1 = parseInt(fixtureA.GetType())
    type2 = parseInt(fixtureB.GetType())
    reg = @m_registers[type1][type2]
    c = undefined
    if reg.pool
      c = reg.pool
      reg.pool = c.m_next
      reg.poolCount--
      c.Reset fixtureA, fixtureB
      return c
    createFcn = reg.createFcn
    if createFcn?
      if reg.primary
        c = createFcn(@m_allocator)
        c.Reset fixtureA, fixtureB
        c
      else
        c = createFcn(@m_allocator)
        c.Reset fixtureB, fixtureA
        c
    else
      null

  b2ContactFactory::Destroy = (contact) ->
    if contact.m_manifold.m_pointCount > 0
      contact.m_fixtureA.m_body.SetAwake true
      contact.m_fixtureB.m_body.SetAwake true
    type1 = parseInt(contact.m_fixtureA.GetType())
    type2 = parseInt(contact.m_fixtureB.GetType())
    reg = @m_registers[type1][type2]
    if true
      reg.poolCount++
      contact.m_next = reg.pool
      reg.pool = contact
    destroyFcn = reg.destroyFcn
    destroyFcn contact, @m_allocator
    return

  b2ContactRegister.b2ContactRegister = ->

  b2ContactResult.b2ContactResult = ->
    @position = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return

  b2ContactSolver.b2ContactSolver = ->
    @m_step = new b2TimeStep()
    @m_constraints = new Vector()
    return

  b2ContactSolver::b2ContactSolver = ->

  b2ContactSolver::Initialize = (step, contacts, contactCount, allocator) ->
    contactCount = 0  if contactCount is `undefined`
    contact = undefined
    @m_step.Set step
    @m_allocator = allocator
    i = 0
    tVec = undefined
    tMat = undefined
    @m_constraintCount = contactCount
    @m_constraints[@m_constraints.length] = new b2ContactConstraint()  while @m_constraints.length < @m_constraintCount
    i = 0
    while i < contactCount
      contact = contacts[i]
      fixtureA = contact.m_fixtureA
      fixtureB = contact.m_fixtureB
      shapeA = fixtureA.m_shape
      shapeB = fixtureB.m_shape
      radiusA = shapeA.m_radius
      radiusB = shapeB.m_radius
      bodyA = fixtureA.m_body
      bodyB = fixtureB.m_body
      manifold = contact.GetManifold()
      friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction())
      restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution())
      vAX = bodyA.m_linearVelocity.x
      vAY = bodyA.m_linearVelocity.y
      vBX = bodyB.m_linearVelocity.x
      vBY = bodyB.m_linearVelocity.y
      wA = bodyA.m_angularVelocity
      wB = bodyB.m_angularVelocity
      b2Settings.b2Assert manifold.m_pointCount > 0
      b2ContactSolver.s_worldManifold.Initialize manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB
      normalX = b2ContactSolver.s_worldManifold.m_normal.x
      normalY = b2ContactSolver.s_worldManifold.m_normal.y
      cc = @m_constraints[i]
      cc.bodyA = bodyA
      cc.bodyB = bodyB
      cc.manifold = manifold
      cc.normal.x = normalX
      cc.normal.y = normalY
      cc.pointCount = manifold.m_pointCount
      cc.friction = friction
      cc.restitution = restitution
      cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x
      cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y
      cc.localPoint.x = manifold.m_localPoint.x
      cc.localPoint.y = manifold.m_localPoint.y
      cc.radius = radiusA + radiusB
      cc.type = manifold.m_type
      k = 0

      while k < cc.pointCount
        cp = manifold.m_points[k]
        ccp = cc.points[k]
        ccp.normalImpulse = cp.m_normalImpulse
        ccp.tangentImpulse = cp.m_tangentImpulse
        ccp.localPoint.SetV cp.m_localPoint
        rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x
        rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y
        rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x
        rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y
        rnA = rAX * normalY - rAY * normalX
        rnB = rBX * normalY - rBY * normalX
        rnA *= rnA
        rnB *= rnB
        kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB
        ccp.normalMass = 1.0 / kNormal
        kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass
        kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB
        ccp.equalizedMass = 1.0 / kEqualized
        tangentX = normalY
        tangentY = (-normalX)
        rtA = rAX * tangentY - rAY * tangentX
        rtB = rBX * tangentY - rBY * tangentX
        rtA *= rtA
        rtB *= rtB
        kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB
        ccp.tangentMass = 1.0 / kTangent
        ccp.velocityBias = 0.0
        tX = vBX + (-wB * rBY) - vAX - (-wA * rAY)
        tY = vBY + (wB * rBX) - vAY - (wA * rAX)
        vRel = cc.normal.x * tX + cc.normal.y * tY
        ccp.velocityBias += (-cc.restitution * vRel)  if vRel < (-b2Settings.b2_velocityThreshold)
        ++k
      if cc.pointCount is 2
        ccp1 = cc.points[0]
        ccp2 = cc.points[1]
        invMassA = bodyA.m_invMass
        invIA = bodyA.m_invI
        invMassB = bodyB.m_invMass
        invIB = bodyB.m_invI
        rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX
        rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX
        rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX
        rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX
        k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B
        k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B
        k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B
        k_maxConditionNumber = 100.0
        if k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)
          cc.K.col1.Set k11, k12
          cc.K.col2.Set k12, k22
          cc.K.GetInverse cc.normalMass
        else
          cc.pointCount = 1
      ++i
    return

  b2ContactSolver::InitVelocityConstraints = (step) ->
    tVec = undefined
    tVec2 = undefined
    tMat = undefined
    i = 0

    while i < @m_constraintCount
      c = @m_constraints[i]
      bodyA = c.bodyA
      bodyB = c.bodyB
      invMassA = bodyA.m_invMass
      invIA = bodyA.m_invI
      invMassB = bodyB.m_invMass
      invIB = bodyB.m_invI
      normalX = c.normal.x
      normalY = c.normal.y
      tangentX = normalY
      tangentY = (-normalX)
      tX = 0
      j = 0
      tCount = 0
      if step.warmStarting
        tCount = c.pointCount
        j = 0
        while j < tCount
          ccp = c.points[j]
          ccp.normalImpulse *= step.dtRatio
          ccp.tangentImpulse *= step.dtRatio
          PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX
          PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY
          bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX)
          bodyA.m_linearVelocity.x -= invMassA * PX
          bodyA.m_linearVelocity.y -= invMassA * PY
          bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX)
          bodyB.m_linearVelocity.x += invMassB * PX
          bodyB.m_linearVelocity.y += invMassB * PY
          ++j
      else
        tCount = c.pointCount
        j = 0
        while j < tCount
          ccp2 = c.points[j]
          ccp2.normalImpulse = 0.0
          ccp2.tangentImpulse = 0.0
          ++j
      ++i
    return

  b2ContactSolver::SolveVelocityConstraints = ->
    j = 0
    ccp = undefined
    rAX = 0
    rAY = 0
    rBX = 0
    rBY = 0
    dvX = 0
    dvY = 0
    vn = 0
    vt = 0
    lambda = 0
    maxFriction = 0
    newImpulse = 0
    PX = 0
    PY = 0
    dX = 0
    dY = 0
    P1X = 0
    P1Y = 0
    P2X = 0
    P2Y = 0
    tMat = undefined
    tVec = undefined
    i = 0

    while i < @m_constraintCount
      c = @m_constraints[i]
      bodyA = c.bodyA
      bodyB = c.bodyB
      wA = bodyA.m_angularVelocity
      wB = bodyB.m_angularVelocity
      vA = bodyA.m_linearVelocity
      vB = bodyB.m_linearVelocity
      invMassA = bodyA.m_invMass
      invIA = bodyA.m_invI
      invMassB = bodyB.m_invMass
      invIB = bodyB.m_invI
      normalX = c.normal.x
      normalY = c.normal.y
      tangentX = normalY
      tangentY = (-normalX)
      friction = c.friction
      tX = 0
      j = 0
      while j < c.pointCount
        ccp = c.points[j]
        dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y
        dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x
        vt = dvX * tangentX + dvY * tangentY
        lambda = ccp.tangentMass * (-vt)
        maxFriction = friction * ccp.normalImpulse
        newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, (-maxFriction), maxFriction)
        lambda = newImpulse - ccp.tangentImpulse
        PX = lambda * tangentX
        PY = lambda * tangentY
        vA.x -= invMassA * PX
        vA.y -= invMassA * PY
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX)
        vB.x += invMassB * PX
        vB.y += invMassB * PY
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX)
        ccp.tangentImpulse = newImpulse
        j++
      tCount = parseInt(c.pointCount)
      if c.pointCount is 1
        ccp = c.points[0]
        dvX = vB.x + (-wB * ccp.rB.y) - vA.x - (-wA * ccp.rA.y)
        dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x)
        vn = dvX * normalX + dvY * normalY
        lambda = (-ccp.normalMass * (vn - ccp.velocityBias))
        newImpulse = ccp.normalImpulse + lambda
        newImpulse = (if newImpulse > 0 then newImpulse else 0.0)
        lambda = newImpulse - ccp.normalImpulse
        PX = lambda * normalX
        PY = lambda * normalY
        vA.x -= invMassA * PX
        vA.y -= invMassA * PY
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX)
        vB.x += invMassB * PX
        vB.y += invMassB * PY
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX)
        ccp.normalImpulse = newImpulse
      else
        cp1 = c.points[0]
        cp2 = c.points[1]
        aX = cp1.normalImpulse
        aY = cp2.normalImpulse
        dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y
        dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x
        dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y
        dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x
        vn1 = dv1X * normalX + dv1Y * normalY
        vn2 = dv2X * normalX + dv2Y * normalY
        bX = vn1 - cp1.velocityBias
        bY = vn2 - cp2.velocityBias
        tMat = c.K
        bX -= tMat.col1.x * aX + tMat.col2.x * aY
        bY -= tMat.col1.y * aX + tMat.col2.y * aY
        k_errorTol = 0.001
        loop
          tMat = c.normalMass
          xX = (-(tMat.col1.x * bX + tMat.col2.x * bY))
          xY = (-(tMat.col1.y * bX + tMat.col2.y * bY))
          if xX >= 0.0 and xY >= 0.0
            dX = xX - aX
            dY = xY - aY
            P1X = dX * normalX
            P1Y = dX * normalY
            P2X = dY * normalX
            P2Y = dY * normalY
            vA.x -= invMassA * (P1X + P2X)
            vA.y -= invMassA * (P1Y + P2Y)
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X)
            vB.x += invMassB * (P1X + P2X)
            vB.y += invMassB * (P1Y + P2Y)
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X)
            cp1.normalImpulse = xX
            cp2.normalImpulse = xY
            break
          xX = (-cp1.normalMass * bX)
          xY = 0.0
          vn1 = 0.0
          vn2 = c.K.col1.y * xX + bY
          if xX >= 0.0 and vn2 >= 0.0
            dX = xX - aX
            dY = xY - aY
            P1X = dX * normalX
            P1Y = dX * normalY
            P2X = dY * normalX
            P2Y = dY * normalY
            vA.x -= invMassA * (P1X + P2X)
            vA.y -= invMassA * (P1Y + P2Y)
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X)
            vB.x += invMassB * (P1X + P2X)
            vB.y += invMassB * (P1Y + P2Y)
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X)
            cp1.normalImpulse = xX
            cp2.normalImpulse = xY
            break
          xX = 0.0
          xY = (-cp2.normalMass * bY)
          vn1 = c.K.col2.x * xY + bX
          vn2 = 0.0
          if xY >= 0.0 and vn1 >= 0.0
            dX = xX - aX
            dY = xY - aY
            P1X = dX * normalX
            P1Y = dX * normalY
            P2X = dY * normalX
            P2Y = dY * normalY
            vA.x -= invMassA * (P1X + P2X)
            vA.y -= invMassA * (P1Y + P2Y)
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X)
            vB.x += invMassB * (P1X + P2X)
            vB.y += invMassB * (P1Y + P2Y)
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X)
            cp1.normalImpulse = xX
            cp2.normalImpulse = xY
            break
          xX = 0.0
          xY = 0.0
          vn1 = bX
          vn2 = bY
          if vn1 >= 0.0 and vn2 >= 0.0
            dX = xX - aX
            dY = xY - aY
            P1X = dX * normalX
            P1Y = dX * normalY
            P2X = dY * normalX
            P2Y = dY * normalY
            vA.x -= invMassA * (P1X + P2X)
            vA.y -= invMassA * (P1Y + P2Y)
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X)
            vB.x += invMassB * (P1X + P2X)
            vB.y += invMassB * (P1Y + P2Y)
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X)
            cp1.normalImpulse = xX
            cp2.normalImpulse = xY
            break
          break
      bodyA.m_angularVelocity = wA
      bodyB.m_angularVelocity = wB
      ++i
    return

  b2ContactSolver::FinalizeVelocityConstraints = ->
    i = 0

    while i < @m_constraintCount
      c = @m_constraints[i]
      m = c.manifold
      j = 0

      while j < c.pointCount
        point1 = m.m_points[j]
        point2 = c.points[j]
        point1.m_normalImpulse = point2.normalImpulse
        point1.m_tangentImpulse = point2.tangentImpulse
        ++j
      ++i
    return

  b2ContactSolver::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    minSeparation = 0.0
    i = 0

    while i < @m_constraintCount
      c = @m_constraints[i]
      bodyA = c.bodyA
      bodyB = c.bodyB
      invMassA = bodyA.m_mass * bodyA.m_invMass
      invIA = bodyA.m_mass * bodyA.m_invI
      invMassB = bodyB.m_mass * bodyB.m_invMass
      invIB = bodyB.m_mass * bodyB.m_invI
      b2ContactSolver.s_psm.Initialize c
      normal = b2ContactSolver.s_psm.m_normal
      j = 0

      while j < c.pointCount
        ccp = c.points[j]
        point = b2ContactSolver.s_psm.m_points[j]
        separation = b2ContactSolver.s_psm.m_separations[j]
        rAX = point.x - bodyA.m_sweep.c.x
        rAY = point.y - bodyA.m_sweep.c.y
        rBX = point.x - bodyB.m_sweep.c.x
        rBY = point.y - bodyB.m_sweep.c.y
        minSeparation = (if minSeparation < separation then minSeparation else separation)
        C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), (-b2Settings.b2_maxLinearCorrection), 0.0)
        impulse = (-ccp.equalizedMass * C)
        PX = impulse * normal.x
        PY = impulse * normal.y
        bodyA.m_sweep.c.x -= invMassA * PX
        bodyA.m_sweep.c.y -= invMassA * PY
        bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX)
        bodyA.SynchronizeTransform()
        bodyB.m_sweep.c.x += invMassB * PX
        bodyB.m_sweep.c.y += invMassB * PY
        bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX)
        bodyB.SynchronizeTransform()
        j++
      i++
    minSeparation > (-1.5 * b2Settings.b2_linearSlop)

  Box2D.postDefs.push ->
    Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold = new b2WorldManifold()
    Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new b2PositionSolverManifold()
    return

  Box2D.inherit b2EdgeAndCircleContact, Box2D.Dynamics.Contacts.b2Contact
  b2EdgeAndCircleContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2EdgeAndCircleContact.b2EdgeAndCircleContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2EdgeAndCircleContact.Create = (allocator) ->
    new b2EdgeAndCircleContact()

  b2EdgeAndCircleContact.Destroy = (contact, allocator) ->

  b2EdgeAndCircleContact::Reset = (fixtureA, fixtureB) ->
    @__super.Reset.call this, fixtureA, fixtureB
    return

  b2EdgeAndCircleContact::Evaluate = ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    @b2CollideEdgeAndCircle @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2EdgeShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

  b2EdgeAndCircleContact::b2CollideEdgeAndCircle = (manifold, edge, xf1, circle, xf2) ->

  Box2D.inherit b2NullContact, Box2D.Dynamics.Contacts.b2Contact
  b2NullContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2NullContact.b2NullContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2NullContact::b2NullContact = ->
    @__super.b2Contact.call this
    return

  b2NullContact::Evaluate = ->

  Box2D.inherit b2PolyAndCircleContact, Box2D.Dynamics.Contacts.b2Contact
  b2PolyAndCircleContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2PolyAndCircleContact.b2PolyAndCircleContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2PolyAndCircleContact.Create = (allocator) ->
    new b2PolyAndCircleContact()

  b2PolyAndCircleContact.Destroy = (contact, allocator) ->

  b2PolyAndCircleContact::Reset = (fixtureA, fixtureB) ->
    @__super.Reset.call this, fixtureA, fixtureB
    b2Settings.b2Assert fixtureA.GetType() is b2Shape.e_polygonShape
    b2Settings.b2Assert fixtureB.GetType() is b2Shape.e_circleShape
    return

  b2PolyAndCircleContact::Evaluate = ->
    bA = @m_fixtureA.m_body
    bB = @m_fixtureB.m_body
    b2Collision.CollidePolygonAndCircle @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2CircleShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

  Box2D.inherit b2PolyAndEdgeContact, Box2D.Dynamics.Contacts.b2Contact
  b2PolyAndEdgeContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2PolyAndEdgeContact.b2PolyAndEdgeContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2PolyAndEdgeContact.Create = (allocator) ->
    new b2PolyAndEdgeContact()

  b2PolyAndEdgeContact.Destroy = (contact, allocator) ->

  b2PolyAndEdgeContact::Reset = (fixtureA, fixtureB) ->
    @__super.Reset.call this, fixtureA, fixtureB
    b2Settings.b2Assert fixtureA.GetType() is b2Shape.e_polygonShape
    b2Settings.b2Assert fixtureB.GetType() is b2Shape.e_edgeShape
    return

  b2PolyAndEdgeContact::Evaluate = ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    @b2CollidePolyAndEdge @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2EdgeShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

  b2PolyAndEdgeContact::b2CollidePolyAndEdge = (manifold, polygon, xf1, edge, xf2) ->

  Box2D.inherit b2PolygonContact, Box2D.Dynamics.Contacts.b2Contact
  b2PolygonContact::__super = Box2D.Dynamics.Contacts.b2Contact::
  b2PolygonContact.b2PolygonContact = ->
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply this, arguments
    return

  b2PolygonContact.Create = (allocator) ->
    new b2PolygonContact()

  b2PolygonContact.Destroy = (contact, allocator) ->

  b2PolygonContact::Reset = (fixtureA, fixtureB) ->
    @__super.Reset.call this, fixtureA, fixtureB
    return

  b2PolygonContact::Evaluate = ->
    bA = @m_fixtureA.GetBody()
    bB = @m_fixtureB.GetBody()
    b2Collision.CollidePolygons @m_manifold, ((if @m_fixtureA.GetShape() instanceof b2PolygonShape then @m_fixtureA.GetShape() else null)), bA.m_xf, ((if @m_fixtureB.GetShape() instanceof b2PolygonShape then @m_fixtureB.GetShape() else null)), bB.m_xf
    return

  b2PositionSolverManifold.b2PositionSolverManifold = ->

  b2PositionSolverManifold::b2PositionSolverManifold = ->
    @m_normal = new b2Vec2()
    @m_separations = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints)
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2Vec2()
      i++
    return

  b2PositionSolverManifold::Initialize = (cc) ->
    b2Settings.b2Assert cc.pointCount > 0
    i = 0
    clipPointX = 0
    clipPointY = 0
    tMat = undefined
    tVec = undefined
    planePointX = 0
    planePointY = 0
    switch cc.type
      when b2Manifold.e_circles
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPoint
        pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyB.m_xf.R
        tVec = cc.points[0].localPoint
        pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        dX = pointBX - pointAX
        dY = pointBY - pointAY
        d2 = dX * dX + dY * dY
        if d2 > Number.MIN_VALUE * Number.MIN_VALUE
          d = Math.sqrt(d2)
          @m_normal.x = dX / d
          @m_normal.y = dY / d
        else
          @m_normal.x = 1.0
          @m_normal.y = 0.0
        @m_points[0].x = 0.5 * (pointAX + pointBX)
        @m_points[0].y = 0.5 * (pointAY + pointBY)
        @m_separations[0] = dX * @m_normal.x + dY * @m_normal.y - cc.radius
      when b2Manifold.e_faceA
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPlaneNormal
        @m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        @m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = cc.bodyA.m_xf.R
        tVec = cc.localPoint
        planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyB.m_xf.R
        i = 0
        while i < cc.pointCount
          tVec = cc.points[i].localPoint
          clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
          clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
          @m_separations[i] = (clipPointX - planePointX) * @m_normal.x + (clipPointY - planePointY) * @m_normal.y - cc.radius
          @m_points[i].x = clipPointX
          @m_points[i].y = clipPointY
          ++i
      when b2Manifold.e_faceB
        tMat = cc.bodyB.m_xf.R
        tVec = cc.localPlaneNormal
        @m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        @m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = cc.bodyB.m_xf.R
        tVec = cc.localPoint
        planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tMat = cc.bodyA.m_xf.R
        i = 0
        while i < cc.pointCount
          tVec = cc.points[i].localPoint
          clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
          clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
          @m_separations[i] = (clipPointX - planePointX) * @m_normal.x + (clipPointY - planePointY) * @m_normal.y - cc.radius
          @m_points[i].Set clipPointX, clipPointY
          ++i
        @m_normal.x *= (-1)
        @m_normal.y *= (-1)

  Box2D.postDefs.push ->
    Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA = new b2Vec2()
    Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB = new b2Vec2()
    return

  return
)()
