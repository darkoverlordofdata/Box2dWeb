class Box2D.Dynamics.Controllers.b2Controller

  m_bodyList    : null
  m_bodyCount   : 0
  m_next        : null
  m_world       : null

  Step: (step) ->

  Draw: (debugDraw) ->

  AddBody: (body) ->
    edge = new b2ControllerEdge()
    edge.controller = this
    edge.body = body
    edge.nextBody = @m_bodyList
    edge.prevBody = null
    @m_bodyList = edge
    edge.nextBody.prevBody = edge  if edge.nextBody
    @m_bodyCount++
    edge.nextController = body.m_controllerList
    edge.prevController = null
    body.m_controllerList = edge
    edge.nextController.prevController = edge  if edge.nextController
    body.m_controllerCount++
    return

  RemoveBody: (body) ->
    edge = body.m_controllerList
    edge = edge.nextController  while edge and edge.controller isnt this
    edge.prevBody.nextBody = edge.nextBody  if edge.prevBody
    edge.nextBody.prevBody = edge.prevBody  if edge.nextBody
    edge.nextController.prevController = edge.prevController  if edge.nextController
    edge.prevController.nextController = edge.nextController  if edge.prevController
    @m_bodyList = edge.nextBody  if @m_bodyList is edge
    body.m_controllerList = edge.nextController  if body.m_controllerList is edge
    body.m_controllerCount--
    @m_bodyCount--
    return

  Clear: ->
    @RemoveBody @m_bodyList.body  while @m_bodyList
    return

  GetNext: ->
    @m_next

  GetWorld: ->
    @m_world

  GetBodyList: ->
    @m_bodyList



