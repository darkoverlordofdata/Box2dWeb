Box2D = require('../../index')

class Box2D.Dynamics.Contacts.b2Contact

  m_nodeA         : null
  m_nodeB         : null
  m_manifold      : null
  m_oldManifold   : null
  m_fixtureA      : null
  m_fixtureB      : null
  m_touching      : false


  constructor: ->
    @m_nodeA = new b2ContactEdge()
    @m_nodeB = new b2ContactEdge()
    @m_manifold = new b2Manifold()
    @m_oldManifold = new b2Manifold()
    return

  GetManifold: ->
    @m_manifold

  GetWorldManifold: (worldManifold) ->
    bodyA = @m_fixtureA.GetBody()
    bodyB = @m_fixtureB.GetBody()
    shapeA = @m_fixtureA.GetShape()
    shapeB = @m_fixtureB.GetShape()
    worldManifold.Initialize @m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius
    return

  IsTouching: ->
    (@m_flags & b2Contact.e_touchingFlag) is b2Contact.e_touchingFlag

  IsContinuous: ->
    (@m_flags & b2Contact.e_continuousFlag) is b2Contact.e_continuousFlag

  SetSensor: (sensor) ->
    if sensor
      @m_flags |= b2Contact.e_sensorFlag
    else
      @m_flags &= ~b2Contact.e_sensorFlag
    return

  IsSensor: ->
    (@m_flags & b2Contact.e_sensorFlag) is b2Contact.e_sensorFlag

  SetEnabled: (flag) ->
    if flag
      @m_flags |= b2Contact.e_enabledFlag
    else
      @m_flags &= ~b2Contact.e_enabledFlag
    return

  IsEnabled: ->
    (@m_flags & b2Contact.e_enabledFlag) is b2Contact.e_enabledFlag

  GetNext: ->
    @m_next

  GetFixtureA: ->
    @m_fixtureA

  GetFixtureB: ->
    @m_fixtureB

  FlagForFiltering: ->
    @m_flags |= b2Contact.e_filterFlag
    return

  b2Contact::b2Contact = ->

  Reset: (fixtureA, fixtureB) ->
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

  Update: (listener) ->
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

  Evaluate: ->

  ComputeTOI: (sweepA, sweepB) ->
    b2Contact.s_input.proxyA.Set @m_fixtureA.GetShape()
    b2Contact.s_input.proxyB.Set @m_fixtureB.GetShape()
    b2Contact.s_input.sweepA = sweepA
    b2Contact.s_input.sweepB = sweepB
    b2Contact.s_input.tolerance = b2Settings.b2_linearSlop
    b2TimeOfImpact.TimeOfImpact b2Contact.s_input

  @e_sensorFlag = 0x0001
  @e_continuousFlag = 0x0002
  @e_islandFlag = 0x0004
  @e_toiFlag = 0x0008
  @e_touchingFlag = 0x0010
  @e_enabledFlag = 0x0020
  @e_filterFlag = 0x0040
  @s_input = new b2TOIInput()

