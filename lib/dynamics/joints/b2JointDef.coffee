Box2D = require('../../index')

class Box2D.Dynamics.Joints.b2JointDef

  constructor: ->
    @type = b2Joint.e_unknownJoint
    @userData = null
    @bodyA = null
    @bodyB = null
    @collideConnected = false
    return


