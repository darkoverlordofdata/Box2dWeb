Box2D = require('../index')

b2ContactFilter = Box2D.Dynamics.Box2D.Dynamics.b2ContactFilter
b2ContactListener = Box2D.Dynamics.Box2D.Dynamics.b2ContactListener


class Box2D.Dynamics.b2ContactManager

  constructor: ->
    @m_world = null
    @m_contactCount = 0
    @m_contactFilter = b2ContactFilter.b2_defaultFilter
    @m_contactListener = b2ContactListener.b2_defaultListener
    @m_contactFactory = new b2ContactFactory(@m_allocator)
    @m_broadPhase = new b2DynamicTreeBroadPhase()
    return

  AddPair: (proxyUserDataA, proxyUserDataB) ->
    fixtureA = ((if proxyUserDataA instanceof b2Fixture then proxyUserDataA else null))
    fixtureB = ((if proxyUserDataB instanceof b2Fixture then proxyUserDataB else null))
    bodyA = fixtureA.GetBody()
    bodyB = fixtureB.GetBody()
    return  if bodyA is bodyB
    edge = bodyB.GetContactList()
    while edge
      if edge.other is bodyA
        fA = edge.contact.GetFixtureA()
        fB = edge.contact.GetFixtureB()
        return  if fA is fixtureA and fB is fixtureB
        return  if fA is fixtureB and fB is fixtureA
      edge = edge.next
    return  if bodyB.ShouldCollide(bodyA) is false
    return  if @m_contactFilter.ShouldCollide(fixtureA, fixtureB) is false
    c = @m_contactFactory.Create(fixtureA, fixtureB)
    fixtureA = c.GetFixtureA()
    fixtureB = c.GetFixtureB()
    bodyA = fixtureA.m_body
    bodyB = fixtureB.m_body
    c.m_prev = null
    c.m_next = @m_world.m_contactList
    @m_world.m_contactList.m_prev = c  if @m_world.m_contactList?
    @m_world.m_contactList = c
    c.m_nodeA.contact = c
    c.m_nodeA.other = bodyB
    c.m_nodeA.prev = null
    c.m_nodeA.next = bodyA.m_contactList
    bodyA.m_contactList.prev = c.m_nodeA  if bodyA.m_contactList?
    bodyA.m_contactList = c.m_nodeA
    c.m_nodeB.contact = c
    c.m_nodeB.other = bodyA
    c.m_nodeB.prev = null
    c.m_nodeB.next = bodyB.m_contactList
    bodyB.m_contactList.prev = c.m_nodeB  if bodyB.m_contactList?
    bodyB.m_contactList = c.m_nodeB
    ++@m_world.m_contactCount
    return

  FindNewContacts: ->
    @m_broadPhase.UpdatePairs Box2D.generateCallback(this, @AddPair)
    return

  Destroy: (c) ->
    fixtureA = c.GetFixtureA()
    fixtureB = c.GetFixtureB()
    bodyA = fixtureA.GetBody()
    bodyB = fixtureB.GetBody()
    @m_contactListener.EndContact c  if c.IsTouching()
    c.m_prev.m_next = c.m_next  if c.m_prev
    c.m_next.m_prev = c.m_prev  if c.m_next
    @m_world.m_contactList = c.m_next  if c is @m_world.m_contactList
    c.m_nodeA.prev.next = c.m_nodeA.next  if c.m_nodeA.prev
    c.m_nodeA.next.prev = c.m_nodeA.prev  if c.m_nodeA.next
    bodyA.m_contactList = c.m_nodeA.next  if c.m_nodeA is bodyA.m_contactList
    c.m_nodeB.prev.next = c.m_nodeB.next  if c.m_nodeB.prev
    c.m_nodeB.next.prev = c.m_nodeB.prev  if c.m_nodeB.next
    bodyB.m_contactList = c.m_nodeB.next  if c.m_nodeB is bodyB.m_contactList
    @m_contactFactory.Destroy c
    --@m_contactCount
    return

  Collide: ->
    c = @m_world.m_contactList
    while c
      fixtureA = c.GetFixtureA()
      fixtureB = c.GetFixtureB()
      bodyA = fixtureA.GetBody()
      bodyB = fixtureB.GetBody()
      if bodyA.IsAwake() is false and bodyB.IsAwake() is false
        c = c.GetNext()
        continue
      if c.m_flags & b2Contact.e_filterFlag
        if bodyB.ShouldCollide(bodyA) is false
          cNuke = c
          c = cNuke.GetNext()
          @Destroy cNuke
          continue
        if @m_contactFilter.ShouldCollide(fixtureA, fixtureB) is false
          cNuke = c
          c = cNuke.GetNext()
          @Destroy cNuke
          continue
        c.m_flags &= ~b2Contact.e_filterFlag
      proxyA = fixtureA.m_proxy
      proxyB = fixtureB.m_proxy
      overlap = @m_broadPhase.TestOverlap(proxyA, proxyB)
      if overlap is false
        cNuke = c
        c = cNuke.GetNext()
        @Destroy cNuke
        continue
      c.Update @m_contactListener
      c = c.GetNext()
    return

  @b2ContactManager.s_evalCP = new b2ContactPoint()


