Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2
b2Body      = Box2D.Dynamics.b2Body
b2Joint     = Box2D.Dynamics.Joints.b2Joint
b2Contact   = Box2D.Dynamics.b2Contact

class Box2D.Dynamics.b2World

  m_bodyList            : null
  m_jointList           : null
  m_fixturesList        : null
  m_contactListener     : null
  m_jointsList          : null
  m_worldID             : 0

  constructor: (gravity, doSleep) ->
    @m_bodyList = []
    @m_jointList = []
    @m_fixturesList = []
    @m_contactListener = null
    @m_jointsList = []
    @m_worldID = window.ext.IDTK_SRV_BOX2D.makeCall("createWorld", gravity.x, gravity.y, doSleep)
    return

  SetContactListener: (listener) ->
    @m_contactListener = listener
    return

  SetContactFilter: (filter) ->
    callbackFunc = (a, b) ->
      fa = world.m_fixturesList[a]
      fb = world.m_fixturesList[b]
      filter.ShouldCollide fa, fb

    window.ext.IDTK_SRV_BOX2D.makeCall "setContactFilter", @m_worldID, callbackFunc
    return

  CreateBody: (def) ->
    b = new b2Body(def, this)
    @m_bodyList[b.m_bodyID] = b
    return b

  DestroyBody: (b) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "deleteBody", @m_worldID, b.m_bodyID
    delete @m_bodyList[b.m_bodyID]
    return

    i = 0

    while i < b.m_fixtures.length
      delete @m_fixturesList[b.m_fixtures[i].m_fixtureID]
      ++i
    return

  CreateJoint: (def) ->
    return  if def.bodyA.m_bodyID is def.bodyB.m_bodyID
    bodyA = def.bodyA
    bodyB = def.bodyB
    def.bodyA = bodyA.m_bodyID
    def.bodyB = bodyB.m_bodyID
    jointFunc = "createDistanceJoint"
    jointFunc = "createRevoluteJoint"  if def.type is b2Joint.e_revoluteJoint
    joint = new b2Joint(def)
    joint.m_jointID = window.ext.IDTK_SRV_BOX2D.makeCall(jointFunc, @m_worldID, def)
    def.bodyA = bodyA
    def.bodyB = bodyB
    @m_jointsList.push joint
    return joint

  DestroyJoint: (joint) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "destroyJoint", @m_worldID, joint.m_jointID
    return

  GetJointList: ->
    return null  if @m_jointsList.length is 0

    # Build the linked-list impersonation inside of the array
    i = 0

    while i < @m_jointsList.length - 1
      @m_jointsList[i].next = @m_jointsList[i + 1]
      ++i
    @m_jointsList[@m_jointsList.length - 1].next = null
    return @m_jointsList[0]

  SetContinuousPhysics: (continuous) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "setContinuous", @m_worldID, continuous
    return

  SetGravity: (gravity) ->
    window.ext.IDTK_SRV_BOX2D.makeCall "setGravity", @m_worldID, gravity.x, gravity.y
    return

  Step: (dt, velocityIterations, positionIterations) ->
    i = undefined
    transforms = window.ext.IDTK_SRV_BOX2D.makeCall("step", @m_worldID, dt, velocityIterations, positionIterations)
    count = transforms[0] # Array returns [ <number of elements> , elem1.bodyID , elem1.posX , elem1.posY , elem1.angle, elem2.bodyID , ....]
    i = 1
    while i <= count * 4
      body = @m_bodyList[transforms[i + 0]]
      # end of the transforms array
      break  if body is null
      body.m_xf.position.Set transforms[i + 1], transforms[i + 2]
      body.m_xf.R.Set transforms[i + 3]
      i += 4

    # Handle object contacts
    if @m_contactListener isnt null
      contacts = window.ext.IDTK_SRV_BOX2D.makeCall("getLastContacts", @m_worldID)
      count = contacts[0]
      i = 1
      while i <= count * 3
        f1 = contacts[i + 0]
        f2 = contacts[i + 1]
        touching = contacts[i + 2]
        fix1 = @m_fixturesList[f1]
        fix2 = @m_fixturesList[f2]
        if (typeof (fix1) is "undefined") or (typeof (fix2) is "undefined")
          console.log "One of the fixtures in a contact DOESN'T EXIST!!"
          continue
        @m_contactListener.BeginContact new b2Contact(fix1, fix2, touching)
        i += 3
    return

  ClearForces: ->
    window.ext.IDTK_SRV_BOX2D.makeCall "clearForces", @m_worldID
    return

  SetDebugDraw: -> #d

  DrawDebugData: ->

