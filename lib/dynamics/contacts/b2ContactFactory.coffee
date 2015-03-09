Box2D = require('../../index')

#Array = Box2D.Array
b2Shape                   = Box2D.Collision.Shapes.b2Shape
b2ContactRegister         = Box2D.Dynamics.Contacts.b2ContactRegister
b2CircleContact           = Box2D.Dynamics.Contacts.b2CircleContact
b2PolyAndCircleContact    = Box2D.Dynamics.Contacts.b2PolyAndCircleContact
b2PolygonContact          = Box2D.Dynamics.Contacts.b2PolygonContact
b2EdgeAndCircleContact    = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact
b2PolyAndEdgeContact      = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact

class Box2D.Dynamics.Contacts.b2ContactFactory

  m_allocator: null
  m_registers: null

  constructor: (allocator) ->
    @m_allocator = allocator
    @InitializeRegisters()
    return

  AddType: (createFcn, destroyFcn, type1, type2) ->
    type1 = 0  if type1 is undefined
    type2 = 0  if type2 is undefined
    @m_registers[type1][type2].createFcn = createFcn
    @m_registers[type1][type2].destroyFcn = destroyFcn
    @m_registers[type1][type2].primary = true
    unless type1 is type2
      @m_registers[type2][type1].createFcn = createFcn
      @m_registers[type2][type1].destroyFcn = destroyFcn
      @m_registers[type2][type1].primary = false
    return

  InitializeRegisters: ->
    @m_registers = new Array(b2Shape.e_shapeTypeCount)
    i = 0

    while i < b2Shape.e_shapeTypeCount
      @m_registers[i] = new Array(b2Shape.e_shapeTypeCount)
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

  Create: (fixtureA, fixtureB) ->
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

  Destroy: (contact) ->
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



