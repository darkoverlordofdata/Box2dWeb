Box2D = require('../../index')

class Box2D.Dynamics.Joints.b2Joint

  constructor: (def) ->
    @bodyA = def.bodyA
    @bodyB = def.bodyB
    @userData = def.userData
    @type = def.type
    @next = null
    return

  GetBodyA: ->
    @bodyA

  GetBodyB: ->
    @bodyB

  GetUserData: ->
    @userData

  GetType: ->
    @type

  GetNext: ->
    @next

  b2Joint.e_distanceJoint = 0
  b2Joint.e_revoluteJoint = 1

