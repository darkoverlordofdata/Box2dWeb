class Box2D.Dynamics.b2Body


  @s_xf1                = new b2Transform()
  @e_islandFlag         = 0x0001
  @e_awakeFlag          = 0x0002
  @e_allowSleepFlag     = 0x0004
  @e_bulletFlag         = 0x0008
  @e_fixedRotationFlag  = 0x0010
  @e_activeFlag         = 0x0020
  @b2_staticBody        = 0
  @b2_kinematicBody     = 1
  @b2_dynamicBody       = 2

  m_xf                  : null
  m_sweep               : null
  m_linearVelocity      : null
  m_force               : null
  m_flags               : 0
  m_world               : null
  m_jointList           : null
  m_controllerList      : null
  m_contactList         : null
  m_controllerCount     : null
  m_prev                : null
  m_next                : null
  m_linearVelocity      : null
  m_angularVelocity     : null
  m_linearDamping       : null
  m_angularDamping      : null
  m_force               : null
  m_torque              : 0
  m_sleepTime           : 0
  m_type                : 0
  m_mass                : 0
  m_invMass             : 0
  m_I                   : 0
  m_invI                : 0
  m_inertiaScale        : 0
  m_userData            : null
  m_fixtureList         : null
  m_fixtureCount        : 0

  constructor: (bd, world) ->
    @m_xf = new b2Transform()
    @m_sweep = new b2Sweep()
    @m_linearVelocity = new b2Vec2()
    @m_force = new b2Vec2()
    @m_flags = 0
    @m_flags |= b2Body.e_bulletFlag  if bd.bullet
    @m_flags |= b2Body.e_fixedRotationFlag  if bd.fixedRotation
    @m_flags |= b2Body.e_allowSleepFlag  if bd.allowSleep
    @m_flags |= b2Body.e_awakeFlag  if bd.awake
    @m_flags |= b2Body.e_activeFlag  if bd.active
    @m_world = world
    @m_xf.position.SetV bd.position
    @m_xf.R.Set bd.angle
    @m_sweep.localCenter.SetZero()
    @m_sweep.t0 = 1.0
    @m_sweep.a0 = @m_sweep.a = bd.angle
    tMat = @m_xf.R
    tVec = @m_sweep.localCenter
    @m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    @m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    @m_sweep.c.x += @m_xf.position.x
    @m_sweep.c.y += @m_xf.position.y
    @m_sweep.c0.SetV @m_sweep.c
    @m_jointList = null
    @m_controllerList = null
    @m_contactList = null
    @m_controllerCount = 0
    @m_prev = null
    @m_next = null
    @m_linearVelocity.SetV bd.linearVelocity
    @m_angularVelocity = bd.angularVelocity
    @m_linearDamping = bd.linearDamping
    @m_angularDamping = bd.angularDamping
    @m_force.Set 0.0, 0.0
    @m_torque = 0.0
    @m_sleepTime = 0.0
    @m_type = bd.type
    if @m_type is b2Body.b2_dynamicBody
      @m_mass = 1.0
      @m_invMass = 1.0
    else
      @m_mass = 0.0
      @m_invMass = 0.0
    @m_I = 0.0
    @m_invI = 0.0
    @m_inertiaScale = bd.inertiaScale
    @m_userData = bd.userData
    @m_fixtureList = null
    @m_fixtureCount = 0
    return



  ConnectEdges: (s1, s2, angle1) ->
    angle1 = 0  if angle1 is undefined
    angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x)
    coreOffset = Math.tan((angle2 - angle1) * 0.5)
    core = b2Math.MulFV(coreOffset, s2.GetDirectionVector())
    core = b2Math.SubtractVV(core, s2.GetNormalVector())
    core = b2Math.MulFV(b2Settings.b2_toiSlop, core)
    core = b2Math.AddVV(core, s2.GetVertex1())
    cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector())
    cornerDir.Normalize()
    convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0
    s1.SetNextEdge s2, core, cornerDir, convex
    s2.SetPrevEdge s1, core, cornerDir, convex
    return angle2

  CreateFixture: (def) ->
    return null  if @m_world.IsLocked() is true
    fixture = new b2Fixture()
    fixture.Create this, @m_xf, def
    if @m_flags & b2Body.e_activeFlag
      broadPhase = @m_world.m_contactManager.m_broadPhase
      fixture.CreateProxy broadPhase, @m_xf
    fixture.m_next = @m_fixtureList
    @m_fixtureList = fixture
    ++@m_fixtureCount
    fixture.m_body = this
    @ResetMassData()  if fixture.m_density > 0.0
    @m_world.m_flags |= 0x0001 #b2World.e_newFixture

    return fixture

  CreateFixture2: (shape, density) ->
    density = 0.0  if density is undefined
    def = new b2FixtureDef()
    def.shape = shape
    def.density = density
    return @CreateFixture def

  DestroyFixture: (fixture) ->
    return  if @m_world.IsLocked() is true
    node = @m_fixtureList
    ppF = null
    found = false
    while node?
      if node is fixture
        if ppF
          ppF.m_next = fixture.m_next
        else
          @m_fixtureList = fixture.m_next
        found = true
        break
      ppF = node
      node = node.m_next
    edge = @m_contactList
    while edge
      c = edge.contact
      edge = edge.next
      fixtureA = c.GetFixtureA()
      fixtureB = c.GetFixtureB()
      @m_world.m_contactManager.Destroy c  if fixture is fixtureA or fixture is fixtureB
    if @m_flags & b2Body.e_activeFlag
      broadPhase = @m_world.m_contactManager.m_broadPhase
      fixture.DestroyProxy broadPhase
    else

    fixture.Destroy()
    fixture.m_body = null
    fixture.m_next = null
    --@m_fixtureCount
    @ResetMassData()
    return

  SetPositionAndAngle: (position, angle) ->
    angle = 0  if angle is undefined
    f = undefined
    return  if @m_world.IsLocked() is true
    @m_xf.R.Set angle
    @m_xf.position.SetV position
    tMat = @m_xf.R
    tVec = @m_sweep.localCenter
    @m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    @m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    @m_sweep.c.x += @m_xf.position.x
    @m_sweep.c.y += @m_xf.position.y
    @m_sweep.c0.SetV @m_sweep.c
    @m_sweep.a0 = @m_sweep.a = angle
    broadPhase = @m_world.m_contactManager.m_broadPhase
    f = @m_fixtureList
    while f
      f.Synchronize broadPhase, @m_xf, @m_xf
      f = f.m_next
    @m_world.m_contactManager.FindNewContacts()
    return

  SetTransform: (xf) ->
    @SetPositionAndAngle xf.position, xf.GetAngle()
    return

  GetTransform: ->
    return @m_xf

  GetPosition: ->
    return @m_xf.position

  SetPosition: (position) ->
    @SetPositionAndAngle position, @GetAngle()
    return

  GetAngle: ->
    return @m_sweep.a

  SetAngle: (angle) ->
    angle = 0  if angle is undefined
    @SetPositionAndAngle @GetPosition(), angle
    return

  GetWorldCenter: ->
    return @m_sweep.c

  GetLocalCenter: ->
    return @m_sweep.localCenter

  SetLinearVelocity: (v) ->
    return  if @m_type is b2Body.b2_staticBody
    @m_linearVelocity.SetV v
    return

  GetLinearVelocity: ->
    return @m_linearVelocity

  SetAngularVelocity: (omega) ->
    omega = 0  if omega is undefined
    return  if @m_type is b2Body.b2_staticBody
    @m_angularVelocity = omega
    return

  GetAngularVelocity: ->
    @m_angularVelocity

  GetDefinition: ->
    bd = new b2BodyDef()
    bd.type = @GetType()
    bd.allowSleep = (@m_flags & b2Body.e_allowSleepFlag) is b2Body.e_allowSleepFlag
    bd.angle = @GetAngle()
    bd.angularDamping = @m_angularDamping
    bd.angularVelocity = @m_angularVelocity
    bd.fixedRotation = (@m_flags & b2Body.e_fixedRotationFlag) is b2Body.e_fixedRotationFlag
    bd.bullet = (@m_flags & b2Body.e_bulletFlag) is b2Body.e_bulletFlag
    bd.awake = (@m_flags & b2Body.e_awakeFlag) is b2Body.e_awakeFlag
    bd.linearDamping = @m_linearDamping
    bd.linearVelocity.SetV @GetLinearVelocity()
    bd.position = @GetPosition()
    bd.userData = @GetUserData()
    return bd

  ApplyForce: (force, point) ->
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_force.x += force.x
    @m_force.y += force.y
    @m_torque += ((point.x - @m_sweep.c.x) * force.y - (point.y - @m_sweep.c.y) * force.x)
    return

  ApplyTorque: (torque) ->
    torque = 0  if torque is undefined
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_torque += torque
    return

  ApplyImpulse: (impulse, point) ->
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_linearVelocity.x += @m_invMass * impulse.x
    @m_linearVelocity.y += @m_invMass * impulse.y
    @m_angularVelocity += @m_invI * ((point.x - @m_sweep.c.x) * impulse.y - (point.y - @m_sweep.c.y) * impulse.x)
    return

  Split: (callback) ->
    linearVelocity = @GetLinearVelocity().Copy()
    angularVelocity = @GetAngularVelocity()
    center = @GetWorldCenter()
    body1 = this
    body2 = @m_world.CreateBody(@GetDefinition())
    prev = undefined
    f = body1.m_fixtureList

    while f
      if callback(f)
        next = f.m_next
        if prev
          prev.m_next = next
        else
          body1.m_fixtureList = next
        body1.m_fixtureCount--
        f.m_next = body2.m_fixtureList
        body2.m_fixtureList = f
        body2.m_fixtureCount++
        f.m_body = body2
        f = next
      else
        prev = f
        f = f.m_next
    body1.ResetMassData()
    body2.ResetMassData()
    center1 = body1.GetWorldCenter()
    center2 = body2.GetWorldCenter()
    velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)))
    velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)))
    body1.SetLinearVelocity velocity1
    body2.SetLinearVelocity velocity2
    body1.SetAngularVelocity angularVelocity
    body2.SetAngularVelocity angularVelocity
    body1.SynchronizeFixtures()
    body2.SynchronizeFixtures()
    return body2

  Merge: (other) ->
    f = undefined
    f = other.m_fixtureList
    while f
      next = f.m_next
      other.m_fixtureCount--
      f.m_next = @m_fixtureList
      @m_fixtureList = f
      @m_fixtureCount++
      f.m_body = body2
      f = next
    body1.m_fixtureCount = 0
    body1 = this
    body2 = other
    center1 = body1.GetWorldCenter()
    center2 = body2.GetWorldCenter()
    velocity1 = body1.GetLinearVelocity().Copy()
    velocity2 = body2.GetLinearVelocity().Copy()
    angular1 = body1.GetAngularVelocity()
    angular = body2.GetAngularVelocity()
    body1.ResetMassData()
    @SynchronizeFixtures()
    return

  GetMass: ->
    return @m_mass

  GetInertia: ->
    return @m_I

  GetMassData: (data) ->
    data.mass = @m_mass
    data.I = @m_I
    data.center.SetV @m_sweep.localCenter
    return

  SetMassData: (massData) ->
    b2Settings.b2Assert @m_world.IsLocked() is false
    return  if @m_world.IsLocked() is true
    return  unless @m_type is b2Body.b2_dynamicBody
    @m_invMass = 0.0
    @m_I = 0.0
    @m_invI = 0.0
    @m_mass = massData.mass
    @m_mass = 1.0  if @m_mass <= 0.0
    @m_invMass = 1.0 / @m_mass
    if massData.I > 0.0 and (@m_flags & b2Body.e_fixedRotationFlag) is 0
      @m_I = massData.I - @m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y)
      @m_invI = 1.0 / @m_I
    oldCenter = @m_sweep.c.Copy()
    @m_sweep.localCenter.SetV massData.center
    @m_sweep.c0.SetV b2Math.MulX(@m_xf, @m_sweep.localCenter)
    @m_sweep.c.SetV @m_sweep.c0
    @m_linearVelocity.x += @m_angularVelocity * (-(@m_sweep.c.y - oldCenter.y))
    @m_linearVelocity.y += @m_angularVelocity * (+(@m_sweep.c.x - oldCenter.x))
    return

  ResetMassData: ->
    @m_mass = 0.0
    @m_invMass = 0.0
    @m_I = 0.0
    @m_invI = 0.0
    @m_sweep.localCenter.SetZero()
    return  if @m_type is b2Body.b2_staticBody or @m_type is b2Body.b2_kinematicBody
    center = b2Vec2.Make(0, 0)
    f = @m_fixtureList

    while f
      continue  if f.m_density is 0.0
      massData = f.GetMassData()
      @m_mass += massData.mass
      center.x += massData.center.x * massData.mass
      center.y += massData.center.y * massData.mass
      @m_I += massData.I
      f = f.m_next
    if @m_mass > 0.0
      @m_invMass = 1.0 / @m_mass
      center.x *= @m_invMass
      center.y *= @m_invMass
    else
      @m_mass = 1.0
      @m_invMass = 1.0
    if @m_I > 0.0 and (@m_flags & b2Body.e_fixedRotationFlag) is 0
      @m_I -= @m_mass * (center.x * center.x + center.y * center.y)
      @m_I *= @m_inertiaScale
      b2Settings.b2Assert @m_I > 0
      @m_invI = 1.0 / @m_I
    else
      @m_I = 0.0
      @m_invI = 0.0
    oldCenter = @m_sweep.c.Copy()
    @m_sweep.localCenter.SetV center
    @m_sweep.c0.SetV b2Math.MulX(@m_xf, @m_sweep.localCenter)
    @m_sweep.c.SetV @m_sweep.c0
    @m_linearVelocity.x += @m_angularVelocity * (-(@m_sweep.c.y - oldCenter.y))
    @m_linearVelocity.y += @m_angularVelocity * (+(@m_sweep.c.x - oldCenter.x))
    return

  GetWorldPoint: (localPoint) ->
    A = @m_xf.R
    u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y)
    u.x += @m_xf.position.x
    u.y += @m_xf.position.y
    return u

  GetWorldVector: (localVector) ->
    return b2Math.MulMV @m_xf.R, localVector

  GetLocalPoint: (worldPoint) ->
    return b2Math.MulXT @m_xf, worldPoint

  GetLocalVector: (worldVector) ->
    return b2Math.MulTMV @m_xf.R, worldVector

  GetLinearVelocityFromWorldPoint: (worldPoint) ->
    return new b2Vec2(@m_linearVelocity.x - @m_angularVelocity * (worldPoint.y - @m_sweep.c.y), @m_linearVelocity.y + @m_angularVelocity * (worldPoint.x - @m_sweep.c.x))

  GetLinearVelocityFromLocalPoint: (localPoint) ->
    A = @m_xf.R
    worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y)
    worldPoint.x += @m_xf.position.x
    worldPoint.y += @m_xf.position.y
    return new b2Vec2(@m_linearVelocity.x - @m_angularVelocity * (worldPoint.y - @m_sweep.c.y), @m_linearVelocity.y + @m_angularVelocity * (worldPoint.x - @m_sweep.c.x))

  GetLinearDamping: ->
    return @m_linearDamping

  SetLinearDamping: (linearDamping) ->
    linearDamping = 0  if linearDamping is undefined
    @m_linearDamping = linearDamping
    return

  GetAngularDamping: ->
    return @m_angularDamping

  SetAngularDamping: (angularDamping) ->
    angularDamping = 0  if angularDamping is undefined
    @m_angularDamping = angularDamping
    return

  SetType: (type) ->
    type = 0  if type is undefined
    return  if @m_type is type
    @m_type = type
    @ResetMassData()
    if @m_type is b2Body.b2_staticBody
      @m_linearVelocity.SetZero()
      @m_angularVelocity = 0.0
    @SetAwake true
    @m_force.SetZero()
    @m_torque = 0.0
    ce = @m_contactList

    while ce
      ce.contact.FlagForFiltering()
      ce = ce.next
    return

  GetType: ->
    return @m_type

  SetBullet: (flag) ->
    if flag
      @m_flags |= b2Body.e_bulletFlag
    else
      @m_flags &= ~b2Body.e_bulletFlag
    return

  IsBullet: ->
    return (@m_flags & b2Body.e_bulletFlag) is b2Body.e_bulletFlag

  SetSleepingAllowed: (flag) ->
    if flag
      @m_flags |= b2Body.e_allowSleepFlag
    else
      @m_flags &= ~b2Body.e_allowSleepFlag
      @SetAwake true
    return

  SetAwake: (flag) ->
    if flag
      @m_flags |= b2Body.e_awakeFlag
      @m_sleepTime = 0.0
    else
      @m_flags &= ~b2Body.e_awakeFlag
      @m_sleepTime = 0.0
      @m_linearVelocity.SetZero()
      @m_angularVelocity = 0.0
      @m_force.SetZero()
      @m_torque = 0.0
    return

  IsAwake: ->
    return (@m_flags & b2Body.e_awakeFlag) is b2Body.e_awakeFlag

  SetFixedRotation: (fixed) ->
    if fixed
      @m_flags |= b2Body.e_fixedRotationFlag
    else
      @m_flags &= ~b2Body.e_fixedRotationFlag
    @ResetMassData()
    return

  IsFixedRotation: ->
    return (@m_flags & b2Body.e_fixedRotationFlag) is b2Body.e_fixedRotationFlag

  SetActive: (flag) ->
    return  if flag is @IsActive()
    broadPhase = undefined
    f = undefined
    if flag
      @m_flags |= b2Body.e_activeFlag
      broadPhase = @m_world.m_contactManager.m_broadPhase
      f = @m_fixtureList
      while f
        f.CreateProxy broadPhase, @m_xf
        f = f.m_next
    else
      @m_flags &= ~b2Body.e_activeFlag
      broadPhase = @m_world.m_contactManager.m_broadPhase
      f = @m_fixtureList
      while f
        f.DestroyProxy broadPhase
        f = f.m_next
      ce = @m_contactList
      while ce
        ce0 = ce
        ce = ce.next
        @m_world.m_contactManager.Destroy ce0.contact
      @m_contactList = null
    return

  IsActive: ->
    return (@m_flags & b2Body.e_activeFlag) is b2Body.e_activeFlag

  IsSleepingAllowed: ->
    return (@m_flags & b2Body.e_allowSleepFlag) is b2Body.e_allowSleepFlag

  GetFixtureList: ->
    return @m_fixtureList

  GetJointList: ->
    return @m_jointList

  GetControllerList: ->
    return @m_controllerList

  GetContactList: ->
    return @m_contactList

  GetNext: ->
    return @m_next

  GetUserData: ->
    return @m_userData

  SetUserData: (data) ->
    @m_userData = data
    return

  GetWorld: ->
    return @m_world

  SynchronizeFixtures: ->
    xf1 = b2Body.s_xf1
    xf1.R.Set @m_sweep.a0
    tMat = xf1.R
    tVec = @m_sweep.localCenter
    xf1.position.x = @m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    xf1.position.y = @m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    f = undefined
    broadPhase = @m_world.m_contactManager.m_broadPhase
    f = @m_fixtureList
    while f
      f.Synchronize broadPhase, xf1, @m_xf
      f = f.m_next
    return

  SynchronizeTransform: ->
    @m_xf.R.Set @m_sweep.a
    tMat = @m_xf.R
    tVec = @m_sweep.localCenter
    @m_xf.position.x = @m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    @m_xf.position.y = @m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    return

  ShouldCollide: (other) ->
    return false  if @m_type isnt b2Body.b2_dynamicBody and other.m_type isnt b2Body.b2_dynamicBody
    jn = @m_jointList

    while jn
      return false  if jn.joint.m_collideConnected is false  if jn.other is other
      jn = jn.next
    return true

  Advance: (t) ->
    t = 0  if t is undefined
    @m_sweep.Advance t
    @m_sweep.c.SetV @m_sweep.c0
    @m_sweep.a = @m_sweep.a0
    @SynchronizeTransform()
    return

