#+--------------------------------------------------------------------+
#| index.coffee
#+--------------------------------------------------------------------+
#| Copyright DarkOverlordOfData (c) 2015
#+--------------------------------------------------------------------+
#|
#| This file is a part of box2d.coffee
#|
#| box2d.coffee is free software; you can copy, modify, and distribute
#| it under the terms of the MIT License
#|
#+--------------------------------------------------------------------+
#
# Box2D
#
###
###
'use strict'
module.exports =
class Box2D

  @parseUInt: (v) -> Math.abs parseInt(v)

  @Vector: (length=0) ->
    tmp = new Array(length)
    i = 0

    while i < length
      tmp[i] = 0
      ++i
    return tmp

  @equals: (o1, o2) ->
    return false  if o1 is null
    return true  if (o2 instanceof Function) and (o1 instanceof o2)
    return true  if (o1.constructor.__implements?[o2.name])
    false


  @generateCallback: (context, cb) ->
    return ->
      cb.apply context, arguments
      return


class Box2D.Common

class Box2D.Common.Math
require './common/math/b2Vec2'
require './common/math/b2Vec3'
require './common/math/b2Mat22'
require './common/math/b2Mat33'
require './common/math/b2Sweep'
require './common/math/b2Transform'
require './common/math/b2Math'

require './common/b2Color'
require './common/b2Settings'

class Box2D.Collision
class Box2D.Collision.Shapes
require './collision/shapes/b2Shape'
require './collision/shapes/b2CircleShape'
require './collision/shapes/b2PolygonShape'
require './collision/shapes/b2EdgeChainDef'
require './collision/shapes/b2EdgeShape'
require './collision/shapes/b2MassData'

require './collision/Features'
require './collision/b2ContactID'
require './collision/ClipVertex'
require './collision/b2AABB'
require './collision/b2SimplexCache'
require './collision/b2Bound'
require './collision/b2BoundValues'
require './collision/b2ManifoldPoint'
require './collision/b2Manifold'
require './collision/b2Collision'
require './collision/b2ContactPoint'
require './collision/b2SimplexVertex'
require './collision/b2Simplex'
require './collision/b2Distance'
require './collision/b2DistanceInput'
require './collision/b2DistanceOutput'
require './collision/b2DistanceProxy'
require './collision/b2DynamicTreeNode'
require './collision/b2DynamicTree'
require './collision/b2DynamicTreePair'
require './collision/b2DynamicTreeBroadPhase'
require './collision/b2Point'
require './collision/b2RayCastInput'
require './collision/b2RayCastOutput'
require './collision/b2Segment'
require './collision/b2SeparationFunction'
require './collision/b2TimeOfImpact'
require './collision/b2TOIInput'
require './collision/b2WorldManifold'


class Box2D.Dynamics
require './dynamics/b2FilterData'
require './dynamics/b2TimeStep'

class Box2D.Dynamics.Contacts
require './dynamics/contacts/b2ContactRegister'
require './dynamics/contacts/b2ContactResult'
require './dynamics/contacts/b2ContactEdge'
require './dynamics/contacts/b2ContactConstraintPoint'
require './dynamics/contacts/b2ContactConstraint'
require './dynamics/contacts/b2Contact'
require './dynamics/contacts/b2CircleContact'
require './dynamics/contacts/b2PositionSolverManifold'
require './dynamics/contacts/b2ContactSolver'
require './dynamics/contacts/b2EdgeAndCircleContact'
require './dynamics/contacts/b2NullContact'
require './dynamics/contacts/b2PolyAndCircleContact'
require './dynamics/contacts/b2PolyAndEdgeContact'
require './dynamics/contacts/b2PolygonContact'
require './dynamics/contacts/b2ContactFactory'

class Box2D.Dynamics.Controllers
require './dynamics/controllers/b2ControllerEdge'
require './dynamics/controllers/b2Controller'
require './dynamics/controllers/b2BuoyancyController'
require './dynamics/controllers/b2ConstantAccelController'
require './dynamics/controllers/b2ConstantForceController'
require './dynamics/controllers/b2GravityController'
require './dynamics/controllers/b2TensorDampingController'

class Box2D.Dynamics.Joints
require './dynamics/joints/b2Joint'
require './dynamics/joints/b2JointDef'
require './dynamics/joints/b2JointEdge'
require './dynamics/joints/b2Jacobian'
require './dynamics/joints/b2DistanceJoint'
require './dynamics/joints/b2DistanceJointDef'
require './dynamics/joints/b2FrictionJoint'
require './dynamics/joints/b2FrictionJointDef'
require './dynamics/joints/b2GearJoint'
require './dynamics/joints/b2GearJointDef'
require './dynamics/joints/b2LineJoint'
require './dynamics/joints/b2LineJointDef'
require './dynamics/joints/b2MouseJoint'
require './dynamics/joints/b2MouseJointDef'
require './dynamics/joints/b2PrismaticJoint'
require './dynamics/joints/b2PrismaticJointDef'
require './dynamics/joints/b2PulleyJoint'
require './dynamics/joints/b2PulleyJointDef'
require './dynamics/joints/b2RevoluteJoint'
require './dynamics/joints/b2RevoluteJointDef'
require './dynamics/joints/b2WeldJoint'
require './dynamics/joints/b2WeldJointDef'

require './dynamics/b2ContactListener'
require './dynamics/b2ContactFilter'
require './dynamics/b2Fixture'
require './dynamics/b2ContactManager'
require './dynamics/b2Body'
require './dynamics/b2DebugDraw'
require './dynamics/b2BodyDef'
require './dynamics/b2ContactFilter'
require './dynamics/b2ContactImpulse'
require './dynamics/b2FixtureDef'
require './dynamics/b2Island'
require './dynamics/b2World'
require './dynamics/b2DestructionListener'

