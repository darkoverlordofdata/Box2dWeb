#
#* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
#*
#* This software is provided 'as-is', without any express or implied
#* warranty.  In no event will the authors be held liable for any damages
#* arising from the use of this software.
#* Permission is granted to anyone to use this software for any purpose,
#* including commercial applications, and to alter it and redistribute it
#* freely, subject to the following restrictions:
#* 1. The origin of this software must not be misrepresented; you must not
#* claim that you wrote the original software. If you use this software
#* in a product, an acknowledgment in the product documentation would be
#* appreciated but is not required.
#* 2. Altered source versions must be plainly marked as such, and must not be
#* misrepresented as being the original software.
#* 3. This notice may not be removed or altered from any source distribution.
#
Box2D = {}
((a2j, undefined_) ->
  
  ###*
  if(!(Object.prototype.defineProperty instanceof Function)
  
  Corrected by deleteing
  10 chars.
  how many to document it?
  
  - darkoverlordofdata
  ###
  emptyFn = ->
  if (Object.defineProperty not instanceof Function) and Object::__defineGetter__ instanceof Function and Object::__defineSetter__ instanceof Function
    Object.defineProperty = (obj, p, cfg) ->
      obj.__defineGetter__ p, cfg.get  if cfg.get instanceof Function
      obj.__defineSetter__ p, cfg.set  if cfg.set instanceof Function
      return
  a2j.inherit = (cls, base) ->
    tmpCtr = cls
    emptyFn:: = base::
    cls:: = new emptyFn
    cls::constructor = tmpCtr
    return

  a2j.generateCallback = generateCallback = (context, cb) ->
    ->
      cb.apply context, arguments
      return

  a2j.NVector = NVector = (length) ->
    length = 0  if length is `undefined`
    tmp = new Array(length or 0)
    i = 0

    while i < length
      tmp[i] = 0
      ++i
    tmp

  a2j.is = equal = (o1, o2) ->
    return false  if o1 is null
    return true  if (o2 instanceof Function) and (o1 instanceof o2)
    return true  if (o1.constructor.__implements?) and (o1.constructor.__implements[o2])
    false

  a2j.parseUInt = (v) ->
    Math.abs parseInt(v)

  return
) Box2D

##TODO remove assignments from global namespace
Vector = Array
Vector_a2j_Number = Box2D.NVector

#package structure
Box2D = {}  if typeof (Box2D) is "undefined"
Box2D.Collision = {}  if typeof (Box2D.Collision) is "undefined"
Box2D.Collision.Shapes = {}  if typeof (Box2D.Collision.Shapes) is "undefined"
Box2D.Common = {}  if typeof (Box2D.Common) is "undefined"
Box2D.Common.Math = {}  if typeof (Box2D.Common.Math) is "undefined"
Box2D.Dynamics = {}  if typeof (Box2D.Dynamics) is "undefined"
Box2D.Dynamics.Contacts = {}  if typeof (Box2D.Dynamics.Contacts) is "undefined"
Box2D.Dynamics.Controllers = {}  if typeof (Box2D.Dynamics.Controllers) is "undefined"
Box2D.Dynamics.Joints = {}  if typeof (Box2D.Dynamics.Joints) is "undefined"

#pre-definitions
(->
  b2AABB = ->
    b2AABB.b2AABB.apply this, arguments
    return
  b2Bound = ->
    b2Bound.b2Bound.apply this, arguments
    return
  b2BoundValues = ->
    b2BoundValues.b2BoundValues.apply this, arguments
    @b2BoundValues.apply this, arguments  if @constructor is b2BoundValues
    return
  b2Collision = ->
    b2Collision.b2Collision.apply this, arguments
    return
  b2ContactID = ->
    b2ContactID.b2ContactID.apply this, arguments
    @b2ContactID.apply this, arguments  if @constructor is b2ContactID
    return
  b2ContactPoint = ->
    b2ContactPoint.b2ContactPoint.apply this, arguments
    return
  b2Distance = ->
    b2Distance.b2Distance.apply this, arguments
    return
  b2DistanceInput = ->
    b2DistanceInput.b2DistanceInput.apply this, arguments
    return
  b2DistanceOutput = ->
    b2DistanceOutput.b2DistanceOutput.apply this, arguments
    return
  b2DistanceProxy = ->
    b2DistanceProxy.b2DistanceProxy.apply this, arguments
    return
  b2DynamicTree = ->
    b2DynamicTree.b2DynamicTree.apply this, arguments
    @b2DynamicTree.apply this, arguments  if @constructor is b2DynamicTree
    return
  b2DynamicTreeBroadPhase = ->
    b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase.apply this, arguments
    return
  b2DynamicTreeNode = ->
    b2DynamicTreeNode.b2DynamicTreeNode.apply this, arguments
    return
  b2DynamicTreePair = ->
    b2DynamicTreePair.b2DynamicTreePair.apply this, arguments
    return
  b2Manifold = ->
    b2Manifold.b2Manifold.apply this, arguments
    @b2Manifold.apply this, arguments  if @constructor is b2Manifold
    return
  b2ManifoldPoint = ->
    b2ManifoldPoint.b2ManifoldPoint.apply this, arguments
    @b2ManifoldPoint.apply this, arguments  if @constructor is b2ManifoldPoint
    return
  b2Point = ->
    b2Point.b2Point.apply this, arguments
    return
  b2RayCastInput = ->
    b2RayCastInput.b2RayCastInput.apply this, arguments
    @b2RayCastInput.apply this, arguments  if @constructor is b2RayCastInput
    return
  b2RayCastOutput = ->
    b2RayCastOutput.b2RayCastOutput.apply this, arguments
    return
  b2Segment = ->
    b2Segment.b2Segment.apply this, arguments
    return
  b2SeparationFunction = ->
    b2SeparationFunction.b2SeparationFunction.apply this, arguments
    return
  b2Simplex = ->
    b2Simplex.b2Simplex.apply this, arguments
    @b2Simplex.apply this, arguments  if @constructor is b2Simplex
    return
  b2SimplexCache = ->
    b2SimplexCache.b2SimplexCache.apply this, arguments
    return
  b2SimplexVertex = ->
    b2SimplexVertex.b2SimplexVertex.apply this, arguments
    return
  b2TimeOfImpact = ->
    b2TimeOfImpact.b2TimeOfImpact.apply this, arguments
    return
  b2TOIInput = ->
    b2TOIInput.b2TOIInput.apply this, arguments
    return
  b2WorldManifold = ->
    b2WorldManifold.b2WorldManifold.apply this, arguments
    @b2WorldManifold.apply this, arguments  if @constructor is b2WorldManifold
    return
  ClipVertex = ->
    ClipVertex.ClipVertex.apply this, arguments
    return
  Features = ->
    Features.Features.apply this, arguments
    return
  b2CircleShape = ->
    b2CircleShape.b2CircleShape.apply this, arguments
    @b2CircleShape.apply this, arguments  if @constructor is b2CircleShape
    return
  b2EdgeChainDef = ->
    b2EdgeChainDef.b2EdgeChainDef.apply this, arguments
    @b2EdgeChainDef.apply this, arguments  if @constructor is b2EdgeChainDef
    return
  b2EdgeShape = ->
    b2EdgeShape.b2EdgeShape.apply this, arguments
    @b2EdgeShape.apply this, arguments  if @constructor is b2EdgeShape
    return
  b2MassData = ->
    b2MassData.b2MassData.apply this, arguments
    return
  b2PolygonShape = ->
    b2PolygonShape.b2PolygonShape.apply this, arguments
    @b2PolygonShape.apply this, arguments  if @constructor is b2PolygonShape
    return
  b2Shape = ->
    b2Shape.b2Shape.apply this, arguments
    @b2Shape.apply this, arguments  if @constructor is b2Shape
    return
  b2Color = ->
    b2Color.b2Color.apply this, arguments
    @b2Color.apply this, arguments  if @constructor is b2Color
    return
  b2Settings = ->
    b2Settings.b2Settings.apply this, arguments
    return
  b2Mat22 = ->
    b2Mat22.b2Mat22.apply this, arguments
    @b2Mat22.apply this, arguments  if @constructor is b2Mat22
    return
  b2Mat33 = ->
    b2Mat33.b2Mat33.apply this, arguments
    @b2Mat33.apply this, arguments  if @constructor is b2Mat33
    return
  b2Math = ->
    b2Math.b2Math.apply this, arguments
    return
  b2Sweep = ->
    b2Sweep.b2Sweep.apply this, arguments
    return
  b2Transform = ->
    b2Transform.b2Transform.apply this, arguments
    @b2Transform.apply this, arguments  if @constructor is b2Transform
    return
  b2Vec2 = ->
    b2Vec2.b2Vec2.apply this, arguments
    @b2Vec2.apply this, arguments  if @constructor is b2Vec2
    return
  b2Vec3 = ->
    b2Vec3.b2Vec3.apply this, arguments
    @b2Vec3.apply this, arguments  if @constructor is b2Vec3
    return
  b2Body = ->
    b2Body.b2Body.apply this, arguments
    @b2Body.apply this, arguments  if @constructor is b2Body
    return
  b2BodyDef = ->
    b2BodyDef.b2BodyDef.apply this, arguments
    @b2BodyDef.apply this, arguments  if @constructor is b2BodyDef
    return
  b2ContactFilter = ->
    b2ContactFilter.b2ContactFilter.apply this, arguments
    return
  b2ContactImpulse = ->
    b2ContactImpulse.b2ContactImpulse.apply this, arguments
    return
  b2ContactListener = ->
    b2ContactListener.b2ContactListener.apply this, arguments
    return
  b2ContactManager = ->
    b2ContactManager.b2ContactManager.apply this, arguments
    @b2ContactManager.apply this, arguments  if @constructor is b2ContactManager
    return
  b2DebugDraw = ->
    b2DebugDraw.b2DebugDraw.apply this, arguments
    @b2DebugDraw.apply this, arguments  if @constructor is b2DebugDraw
    return
  b2DestructionListener = ->
    b2DestructionListener.b2DestructionListener.apply this, arguments
    return
  b2FilterData = ->
    b2FilterData.b2FilterData.apply this, arguments
    return
  b2Fixture = ->
    b2Fixture.b2Fixture.apply this, arguments
    @b2Fixture.apply this, arguments  if @constructor is b2Fixture
    return
  b2FixtureDef = ->
    b2FixtureDef.b2FixtureDef.apply this, arguments
    @b2FixtureDef.apply this, arguments  if @constructor is b2FixtureDef
    return
  b2Island = ->
    b2Island.b2Island.apply this, arguments
    @b2Island.apply this, arguments  if @constructor is b2Island
    return
  b2TimeStep = ->
    b2TimeStep.b2TimeStep.apply this, arguments
    return
  b2World = ->
    b2World.b2World.apply this, arguments
    @b2World.apply this, arguments  if @constructor is b2World
    return
  b2CircleContact = ->
    b2CircleContact.b2CircleContact.apply this, arguments
    return
  b2Contact = ->
    b2Contact.b2Contact.apply this, arguments
    @b2Contact.apply this, arguments  if @constructor is b2Contact
    return
  b2ContactConstraint = ->
    b2ContactConstraint.b2ContactConstraint.apply this, arguments
    @b2ContactConstraint.apply this, arguments  if @constructor is b2ContactConstraint
    return
  b2ContactConstraintPoint = ->
    b2ContactConstraintPoint.b2ContactConstraintPoint.apply this, arguments
    return
  b2ContactEdge = ->
    b2ContactEdge.b2ContactEdge.apply this, arguments
    return
  b2ContactFactory = ->
    b2ContactFactory.b2ContactFactory.apply this, arguments
    @b2ContactFactory.apply this, arguments  if @constructor is b2ContactFactory
    return
  b2ContactRegister = ->
    b2ContactRegister.b2ContactRegister.apply this, arguments
    return
  b2ContactResult = ->
    b2ContactResult.b2ContactResult.apply this, arguments
    return
  b2ContactSolver = ->
    b2ContactSolver.b2ContactSolver.apply this, arguments
    @b2ContactSolver.apply this, arguments  if @constructor is b2ContactSolver
    return
  b2EdgeAndCircleContact = ->
    b2EdgeAndCircleContact.b2EdgeAndCircleContact.apply this, arguments
    return
  b2NullContact = ->
    b2NullContact.b2NullContact.apply this, arguments
    @b2NullContact.apply this, arguments  if @constructor is b2NullContact
    return
  b2PolyAndCircleContact = ->
    b2PolyAndCircleContact.b2PolyAndCircleContact.apply this, arguments
    return
  b2PolyAndEdgeContact = ->
    b2PolyAndEdgeContact.b2PolyAndEdgeContact.apply this, arguments
    return
  b2PolygonContact = ->
    b2PolygonContact.b2PolygonContact.apply this, arguments
    return
  b2PositionSolverManifold = ->
    b2PositionSolverManifold.b2PositionSolverManifold.apply this, arguments
    @b2PositionSolverManifold.apply this, arguments  if @constructor is b2PositionSolverManifold
    return
  b2BuoyancyController = ->
    b2BuoyancyController.b2BuoyancyController.apply this, arguments
    return
  b2ConstantAccelController = ->
    b2ConstantAccelController.b2ConstantAccelController.apply this, arguments
    return
  b2ConstantForceController = ->
    b2ConstantForceController.b2ConstantForceController.apply this, arguments
    return
  b2Controller = ->
    b2Controller.b2Controller.apply this, arguments
    return
  b2ControllerEdge = ->
    b2ControllerEdge.b2ControllerEdge.apply this, arguments
    return
  b2GravityController = ->
    b2GravityController.b2GravityController.apply this, arguments
    return
  b2TensorDampingController = ->
    b2TensorDampingController.b2TensorDampingController.apply this, arguments
    return
  b2DistanceJoint = ->
    b2DistanceJoint.b2DistanceJoint.apply this, arguments
    @b2DistanceJoint.apply this, arguments  if @constructor is b2DistanceJoint
    return
  b2DistanceJointDef = ->
    b2DistanceJointDef.b2DistanceJointDef.apply this, arguments
    @b2DistanceJointDef.apply this, arguments  if @constructor is b2DistanceJointDef
    return
  b2FrictionJoint = ->
    b2FrictionJoint.b2FrictionJoint.apply this, arguments
    @b2FrictionJoint.apply this, arguments  if @constructor is b2FrictionJoint
    return
  b2FrictionJointDef = ->
    b2FrictionJointDef.b2FrictionJointDef.apply this, arguments
    @b2FrictionJointDef.apply this, arguments  if @constructor is b2FrictionJointDef
    return
  b2GearJoint = ->
    b2GearJoint.b2GearJoint.apply this, arguments
    @b2GearJoint.apply this, arguments  if @constructor is b2GearJoint
    return
  b2GearJointDef = ->
    b2GearJointDef.b2GearJointDef.apply this, arguments
    @b2GearJointDef.apply this, arguments  if @constructor is b2GearJointDef
    return
  b2Jacobian = ->
    b2Jacobian.b2Jacobian.apply this, arguments
    return
  b2Joint = ->
    b2Joint.b2Joint.apply this, arguments
    @b2Joint.apply this, arguments  if @constructor is b2Joint
    return
  b2JointDef = ->
    b2JointDef.b2JointDef.apply this, arguments
    @b2JointDef.apply this, arguments  if @constructor is b2JointDef
    return
  b2JointEdge = ->
    b2JointEdge.b2JointEdge.apply this, arguments
    return
  b2LineJoint = ->
    b2LineJoint.b2LineJoint.apply this, arguments
    @b2LineJoint.apply this, arguments  if @constructor is b2LineJoint
    return
  b2LineJointDef = ->
    b2LineJointDef.b2LineJointDef.apply this, arguments
    @b2LineJointDef.apply this, arguments  if @constructor is b2LineJointDef
    return
  b2MouseJoint = ->
    b2MouseJoint.b2MouseJoint.apply this, arguments
    @b2MouseJoint.apply this, arguments  if @constructor is b2MouseJoint
    return
  b2MouseJointDef = ->
    b2MouseJointDef.b2MouseJointDef.apply this, arguments
    @b2MouseJointDef.apply this, arguments  if @constructor is b2MouseJointDef
    return
  b2PrismaticJoint = ->
    b2PrismaticJoint.b2PrismaticJoint.apply this, arguments
    @b2PrismaticJoint.apply this, arguments  if @constructor is b2PrismaticJoint
    return
  b2PrismaticJointDef = ->
    b2PrismaticJointDef.b2PrismaticJointDef.apply this, arguments
    @b2PrismaticJointDef.apply this, arguments  if @constructor is b2PrismaticJointDef
    return
  b2PulleyJoint = ->
    b2PulleyJoint.b2PulleyJoint.apply this, arguments
    @b2PulleyJoint.apply this, arguments  if @constructor is b2PulleyJoint
    return
  b2PulleyJointDef = ->
    b2PulleyJointDef.b2PulleyJointDef.apply this, arguments
    @b2PulleyJointDef.apply this, arguments  if @constructor is b2PulleyJointDef
    return
  b2RevoluteJoint = ->
    b2RevoluteJoint.b2RevoluteJoint.apply this, arguments
    @b2RevoluteJoint.apply this, arguments  if @constructor is b2RevoluteJoint
    return
  b2RevoluteJointDef = ->
    b2RevoluteJointDef.b2RevoluteJointDef.apply this, arguments
    @b2RevoluteJointDef.apply this, arguments  if @constructor is b2RevoluteJointDef
    return
  b2WeldJoint = ->
    b2WeldJoint.b2WeldJoint.apply this, arguments
    @b2WeldJoint.apply this, arguments  if @constructor is b2WeldJoint
    return
  b2WeldJointDef = ->
    b2WeldJointDef.b2WeldJointDef.apply this, arguments
    @b2WeldJointDef.apply this, arguments  if @constructor is b2WeldJointDef
    return
  Box2D.Collision.IBroadPhase = "Box2D.Collision.IBroadPhase"
  Box2D.Collision.b2AABB = b2AABB
  Box2D.Collision.b2Bound = b2Bound
  Box2D.Collision.b2BoundValues = b2BoundValues
  Box2D.Collision.b2Collision = b2Collision
  Box2D.Collision.b2ContactID = b2ContactID
  Box2D.Collision.b2ContactPoint = b2ContactPoint
  Box2D.Collision.b2Distance = b2Distance
  Box2D.Collision.b2DistanceInput = b2DistanceInput
  Box2D.Collision.b2DistanceOutput = b2DistanceOutput
  Box2D.Collision.b2DistanceProxy = b2DistanceProxy
  Box2D.Collision.b2DynamicTree = b2DynamicTree
  Box2D.Collision.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase
  Box2D.Collision.b2DynamicTreeNode = b2DynamicTreeNode
  Box2D.Collision.b2DynamicTreePair = b2DynamicTreePair
  Box2D.Collision.b2Manifold = b2Manifold
  Box2D.Collision.b2ManifoldPoint = b2ManifoldPoint
  Box2D.Collision.b2Point = b2Point
  Box2D.Collision.b2RayCastInput = b2RayCastInput
  Box2D.Collision.b2RayCastOutput = b2RayCastOutput
  Box2D.Collision.b2Segment = b2Segment
  Box2D.Collision.b2SeparationFunction = b2SeparationFunction
  Box2D.Collision.b2Simplex = b2Simplex
  Box2D.Collision.b2SimplexCache = b2SimplexCache
  Box2D.Collision.b2SimplexVertex = b2SimplexVertex
  Box2D.Collision.b2TimeOfImpact = b2TimeOfImpact
  Box2D.Collision.b2TOIInput = b2TOIInput
  Box2D.Collision.b2WorldManifold = b2WorldManifold
  Box2D.Collision.ClipVertex = ClipVertex
  Box2D.Collision.Features = Features
  Box2D.Collision.Shapes.b2CircleShape = b2CircleShape
  Box2D.Collision.Shapes.b2EdgeChainDef = b2EdgeChainDef
  Box2D.Collision.Shapes.b2EdgeShape = b2EdgeShape
  Box2D.Collision.Shapes.b2MassData = b2MassData
  Box2D.Collision.Shapes.b2PolygonShape = b2PolygonShape
  Box2D.Collision.Shapes.b2Shape = b2Shape
  Box2D.Common.b2internal = "Box2D.Common.b2internal"
  Box2D.Common.b2Color = b2Color
  Box2D.Common.b2Settings = b2Settings
  Box2D.Common.Math.b2Mat22 = b2Mat22
  Box2D.Common.Math.b2Mat33 = b2Mat33
  Box2D.Common.Math.b2Math = b2Math
  Box2D.Common.Math.b2Sweep = b2Sweep
  Box2D.Common.Math.b2Transform = b2Transform
  Box2D.Common.Math.b2Vec2 = b2Vec2
  Box2D.Common.Math.b2Vec3 = b2Vec3
  Box2D.Dynamics.b2Body = b2Body
  Box2D.Dynamics.b2BodyDef = b2BodyDef
  Box2D.Dynamics.b2ContactFilter = b2ContactFilter
  Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse
  Box2D.Dynamics.b2ContactListener = b2ContactListener
  Box2D.Dynamics.b2ContactManager = b2ContactManager
  Box2D.Dynamics.b2DebugDraw = b2DebugDraw
  Box2D.Dynamics.b2DestructionListener = b2DestructionListener
  Box2D.Dynamics.b2FilterData = b2FilterData
  Box2D.Dynamics.b2Fixture = b2Fixture
  Box2D.Dynamics.b2FixtureDef = b2FixtureDef
  Box2D.Dynamics.b2Island = b2Island
  Box2D.Dynamics.b2TimeStep = b2TimeStep
  Box2D.Dynamics.b2World = b2World
  Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact
  Box2D.Dynamics.Contacts.b2Contact = b2Contact
  Box2D.Dynamics.Contacts.b2ContactConstraint = b2ContactConstraint
  Box2D.Dynamics.Contacts.b2ContactConstraintPoint = b2ContactConstraintPoint
  Box2D.Dynamics.Contacts.b2ContactEdge = b2ContactEdge
  Box2D.Dynamics.Contacts.b2ContactFactory = b2ContactFactory
  Box2D.Dynamics.Contacts.b2ContactRegister = b2ContactRegister
  Box2D.Dynamics.Contacts.b2ContactResult = b2ContactResult
  Box2D.Dynamics.Contacts.b2ContactSolver = b2ContactSolver
  Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact
  Box2D.Dynamics.Contacts.b2NullContact = b2NullContact
  Box2D.Dynamics.Contacts.b2PolyAndCircleContact = b2PolyAndCircleContact
  Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = b2PolyAndEdgeContact
  Box2D.Dynamics.Contacts.b2PolygonContact = b2PolygonContact
  Box2D.Dynamics.Contacts.b2PositionSolverManifold = b2PositionSolverManifold
  Box2D.Dynamics.Controllers.b2BuoyancyController = b2BuoyancyController
  Box2D.Dynamics.Controllers.b2ConstantAccelController = b2ConstantAccelController
  Box2D.Dynamics.Controllers.b2ConstantForceController = b2ConstantForceController
  Box2D.Dynamics.Controllers.b2Controller = b2Controller
  Box2D.Dynamics.Controllers.b2ControllerEdge = b2ControllerEdge
  Box2D.Dynamics.Controllers.b2GravityController = b2GravityController
  Box2D.Dynamics.Controllers.b2TensorDampingController = b2TensorDampingController
  Box2D.Dynamics.Joints.b2DistanceJoint = b2DistanceJoint
  Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef
  Box2D.Dynamics.Joints.b2FrictionJoint = b2FrictionJoint
  Box2D.Dynamics.Joints.b2FrictionJointDef = b2FrictionJointDef
  Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint
  Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef
  Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian
  Box2D.Dynamics.Joints.b2Joint = b2Joint
  Box2D.Dynamics.Joints.b2JointDef = b2JointDef
  Box2D.Dynamics.Joints.b2JointEdge = b2JointEdge
  Box2D.Dynamics.Joints.b2LineJoint = b2LineJoint
  Box2D.Dynamics.Joints.b2LineJointDef = b2LineJointDef
  Box2D.Dynamics.Joints.b2MouseJoint = b2MouseJoint
  Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef
  Box2D.Dynamics.Joints.b2PrismaticJoint = b2PrismaticJoint
  Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef
  Box2D.Dynamics.Joints.b2PulleyJoint = b2PulleyJoint
  Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef
  Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint
  Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef
  Box2D.Dynamics.Joints.b2WeldJoint = b2WeldJoint
  Box2D.Dynamics.Joints.b2WeldJointDef = b2WeldJointDef
  return
)() #definitions
Box2D.postDefs = []
(->
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
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
  b2AABB.b2AABB = ->
    @lowerBound = new b2Vec2()
    @upperBound = new b2Vec2()
    return

  b2AABB::IsValid = ->
    dX = @upperBound.x - @lowerBound.x
    dY = @upperBound.y - @lowerBound.y
    valid = dX >= 0.0 and dY >= 0.0
    valid = valid and @lowerBound.IsValid() and @upperBound.IsValid()
    valid

  b2AABB::GetCenter = ->
    new b2Vec2((@lowerBound.x + @upperBound.x) / 2, (@lowerBound.y + @upperBound.y) / 2)

  b2AABB::GetExtents = ->
    new b2Vec2((@upperBound.x - @lowerBound.x) / 2, (@upperBound.y - @lowerBound.y) / 2)

  b2AABB::Contains = (aabb) ->
    result = true
    result = result and @lowerBound.x <= aabb.lowerBound.x
    result = result and @lowerBound.y <= aabb.lowerBound.y
    result = result and aabb.upperBound.x <= @upperBound.x
    result = result and aabb.upperBound.y <= @upperBound.y
    result

  b2AABB::RayCast = (output, input) ->
    tmin = (-Number.MAX_VALUE)
    tmax = Number.MAX_VALUE
    pX = input.p1.x
    pY = input.p1.y
    dX = input.p2.x - input.p1.x
    dY = input.p2.y - input.p1.y
    absDX = Math.abs(dX)
    absDY = Math.abs(dY)
    normal = output.normal
    inv_d = 0
    t1 = 0
    t2 = 0
    t3 = 0
    s = 0
    if absDX < Number.MIN_VALUE
      return false  if pX < @lowerBound.x or @upperBound.x < pX
    else
      inv_d = 1.0 / dX
      t1 = (@lowerBound.x - pX) * inv_d
      t2 = (@upperBound.x - pX) * inv_d
      s = (-1.0)
      if t1 > t2
        t3 = t1
        t1 = t2
        t2 = t3
        s = 1.0
      if t1 > tmin
        normal.x = s
        normal.y = 0
        tmin = t1
      tmax = Math.min(tmax, t2)
      return false  if tmin > tmax
    if absDY < Number.MIN_VALUE
      return false  if pY < @lowerBound.y or @upperBound.y < pY
    else
      inv_d = 1.0 / dY
      t1 = (@lowerBound.y - pY) * inv_d
      t2 = (@upperBound.y - pY) * inv_d
      s = (-1.0)
      if t1 > t2
        t3 = t1
        t1 = t2
        t2 = t3
        s = 1.0
      if t1 > tmin
        normal.y = s
        normal.x = 0
        tmin = t1
      tmax = Math.min(tmax, t2)
      return false  if tmin > tmax
    output.fraction = tmin
    true

  b2AABB::TestOverlap = (other) ->
    d1X = other.lowerBound.x - @upperBound.x
    d1Y = other.lowerBound.y - @upperBound.y
    d2X = @lowerBound.x - other.upperBound.x
    d2Y = @lowerBound.y - other.upperBound.y
    return false  if d1X > 0.0 or d1Y > 0.0
    return false  if d2X > 0.0 or d2Y > 0.0
    true

  b2AABB.Combine = (aabb1, aabb2) ->
    aabb = new b2AABB()
    aabb.Combine aabb1, aabb2
    aabb

  b2AABB::Combine = (aabb1, aabb2) ->
    @lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x)
    @lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y)
    @upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x)
    @upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y)
    return

  b2Bound.b2Bound = ->

  b2Bound::IsLower = ->
    (@value & 1) is 0

  b2Bound::IsUpper = ->
    (@value & 1) is 1

  b2Bound::Swap = (b) ->
    tempValue = @value
    tempProxy = @proxy
    tempStabbingCount = @stabbingCount
    @value = b.value
    @proxy = b.proxy
    @stabbingCount = b.stabbingCount
    b.value = tempValue
    b.proxy = tempProxy
    b.stabbingCount = tempStabbingCount
    return

  b2BoundValues.b2BoundValues = ->

  b2BoundValues::b2BoundValues = ->
    @lowerValues = new Vector_a2j_Number()
    @lowerValues[0] = 0.0
    @lowerValues[1] = 0.0
    @upperValues = new Vector_a2j_Number()
    @upperValues[0] = 0.0
    @upperValues[1] = 0.0
    return

  b2Collision.b2Collision = ->

  b2Collision.ClipSegmentToLine = (vOut, vIn, normal, offset) ->
    offset = 0  if offset is `undefined`
    cv = undefined
    numOut = 0
    cv = vIn[0]
    vIn0 = cv.v
    cv = vIn[1]
    vIn1 = cv.v
    distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset
    distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset
    vOut[numOut++].Set vIn[0]  if distance0 <= 0.0
    vOut[numOut++].Set vIn[1]  if distance1 <= 0.0
    if distance0 * distance1 < 0.0
      interp = distance0 / (distance0 - distance1)
      cv = vOut[numOut]
      tVec = cv.v
      tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x)
      tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y)
      cv = vOut[numOut]
      cv2 = undefined
      if distance0 > 0.0
        cv2 = vIn[0]
        cv.id = cv2.id
      else
        cv2 = vIn[1]
        cv.id = cv2.id
      ++numOut
    numOut

  b2Collision.EdgeSeparation = (poly1, xf1, edge1, poly2, xf2) ->
    edge1 = 0  if edge1 is `undefined`
    count1 = parseInt(poly1.m_vertexCount)
    vertices1 = poly1.m_vertices
    normals1 = poly1.m_normals
    count2 = parseInt(poly2.m_vertexCount)
    vertices2 = poly2.m_vertices
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = normals1[edge1]
    normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY)
    normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY)
    index = 0
    minDot = Number.MAX_VALUE
    i = 0

    while i < count2
      tVec = vertices2[i]
      dot = tVec.x * normal1X + tVec.y * normal1Y
      if dot < minDot
        minDot = dot
        index = i
      ++i
    tVec = vertices1[edge1]
    tMat = xf1.R
    v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tVec = vertices2[index]
    tMat = xf2.R
    v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    v2X -= v1X
    v2Y -= v1Y
    separation = v2X * normal1WorldX + v2Y * normal1WorldY
    separation

  b2Collision.FindMaxSeparation = (edgeIndex, poly1, xf1, poly2, xf2) ->
    count1 = parseInt(poly1.m_vertexCount)
    normals1 = poly1.m_normals
    tVec = undefined
    tMat = undefined
    tMat = xf2.R
    tVec = poly2.m_centroid
    dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf1.R
    tVec = poly1.m_centroid
    dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y)
    dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y)
    edge = 0
    maxDot = (-Number.MAX_VALUE)
    i = 0

    while i < count1
      tVec = normals1[i]
      dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y)
      if dot > maxDot
        maxDot = dot
        edge = i
      ++i
    s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2)
    prevEdge = parseInt((if edge - 1 >= 0 then edge - 1 else count1 - 1))
    sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2)
    nextEdge = parseInt((if edge + 1 < count1 then edge + 1 else 0))
    sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2)
    bestEdge = 0
    bestSeparation = 0
    increment = 0
    if sPrev > s and sPrev > sNext
      increment = (-1)
      bestEdge = prevEdge
      bestSeparation = sPrev
    else if sNext > s
      increment = 1
      bestEdge = nextEdge
      bestSeparation = sNext
    else
      edgeIndex[0] = edge
      return s
    loop
      if increment is (-1)
        edge = (if bestEdge - 1 >= 0 then bestEdge - 1 else count1 - 1)
      else
        edge = (if bestEdge + 1 < count1 then bestEdge + 1 else 0)
      s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2)
      if s > bestSeparation
        bestEdge = edge
        bestSeparation = s
      else
        break
    edgeIndex[0] = bestEdge
    bestSeparation

  b2Collision.FindIncidentEdge = (c, poly1, xf1, edge1, poly2, xf2) ->
    edge1 = 0  if edge1 is `undefined`
    count1 = parseInt(poly1.m_vertexCount)
    normals1 = poly1.m_normals
    count2 = parseInt(poly2.m_vertexCount)
    vertices2 = poly2.m_vertices
    normals2 = poly2.m_normals
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = normals1[edge1]
    normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y)
    normal1Y = (tMat.col2.x * normal1X + tMat.col2.y * normal1Y)
    normal1X = tX
    index = 0
    minDot = Number.MAX_VALUE
    i = 0

    while i < count2
      tVec = normals2[i]
      dot = (normal1X * tVec.x + normal1Y * tVec.y)
      if dot < minDot
        minDot = dot
        index = i
      ++i
    tClip = undefined
    i1 = parseInt(index)
    i2 = parseInt((if i1 + 1 < count2 then i1 + 1 else 0))
    tClip = c[0]
    tVec = vertices2[i1]
    tMat = xf2.R
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tClip.id.features.referenceEdge = edge1
    tClip.id.features.incidentEdge = i1
    tClip.id.features.incidentVertex = 0
    tClip = c[1]
    tVec = vertices2[i2]
    tMat = xf2.R
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tClip.id.features.referenceEdge = edge1
    tClip.id.features.incidentEdge = i2
    tClip.id.features.incidentVertex = 1
    return

  b2Collision.MakeClipPointVector = ->
    r = new Vector(2)
    r[0] = new ClipVertex()
    r[1] = new ClipVertex()
    r

  b2Collision.CollidePolygons = (manifold, polyA, xfA, polyB, xfB) ->
    cv = undefined
    manifold.m_pointCount = 0
    totalRadius = polyA.m_radius + polyB.m_radius
    edgeA = 0
    b2Collision.s_edgeAO[0] = edgeA
    separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB)
    edgeA = b2Collision.s_edgeAO[0]
    return  if separationA > totalRadius
    edgeB = 0
    b2Collision.s_edgeBO[0] = edgeB
    separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA)
    edgeB = b2Collision.s_edgeBO[0]
    return  if separationB > totalRadius
    poly1 = undefined
    poly2 = undefined
    xf1 = undefined
    xf2 = undefined
    edge1 = 0
    flip = 0
    k_relativeTol = 0.98
    k_absoluteTol = 0.001
    tMat = undefined
    if separationB > k_relativeTol * separationA + k_absoluteTol
      poly1 = polyB
      poly2 = polyA
      xf1 = xfB
      xf2 = xfA
      edge1 = edgeB
      manifold.m_type = b2Manifold.e_faceB
      flip = 1
    else
      poly1 = polyA
      poly2 = polyB
      xf1 = xfA
      xf2 = xfB
      edge1 = edgeA
      manifold.m_type = b2Manifold.e_faceA
      flip = 0
    incidentEdge = b2Collision.s_incidentEdge
    b2Collision.FindIncidentEdge incidentEdge, poly1, xf1, edge1, poly2, xf2
    count1 = parseInt(poly1.m_vertexCount)
    vertices1 = poly1.m_vertices
    local_v11 = vertices1[edge1]
    local_v12 = undefined
    if edge1 + 1 < count1
      local_v12 = vertices1[parseInt(edge1 + 1)]
    else
      local_v12 = vertices1[0]
    localTangent = b2Collision.s_localTangent
    localTangent.Set local_v12.x - local_v11.x, local_v12.y - local_v11.y
    localTangent.Normalize()
    localNormal = b2Collision.s_localNormal
    localNormal.x = localTangent.y
    localNormal.y = (-localTangent.x)
    planePoint = b2Collision.s_planePoint
    planePoint.Set 0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y)
    tangent = b2Collision.s_tangent
    tMat = xf1.R
    tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y)
    tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y)
    tangent2 = b2Collision.s_tangent2
    tangent2.x = (-tangent.x)
    tangent2.y = (-tangent.y)
    normal = b2Collision.s_normal
    normal.x = tangent.y
    normal.y = (-tangent.x)
    v11 = b2Collision.s_v11
    v12 = b2Collision.s_v12
    v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y)
    v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y)
    v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y)
    v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y)
    frontOffset = normal.x * v11.x + normal.y * v11.y
    sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius
    sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius
    clipPoints1 = b2Collision.s_clipPoints1
    clipPoints2 = b2Collision.s_clipPoints2
    np = 0
    np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1)
    return  if np < 2
    np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2)
    return  if np < 2
    manifold.m_localPlaneNormal.SetV localNormal
    manifold.m_localPoint.SetV planePoint
    pointCount = 0
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      cv = clipPoints2[i]
      separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset
      if separation <= totalRadius
        cp = manifold.m_points[pointCount]
        tMat = xf2.R
        tX = cv.v.x - xf2.position.x
        tY = cv.v.y - xf2.position.y
        cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y)
        cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y)
        cp.m_id.Set cv.id
        cp.m_id.features.flip = flip
        ++pointCount
      ++i
    manifold.m_pointCount = pointCount
    return

  b2Collision.CollideCircles = (manifold, circle1, xf1, circle2, xf2) ->
    manifold.m_pointCount = 0
    tMat = undefined
    tVec = undefined
    tMat = xf1.R
    tVec = circle1.m_p
    p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    tMat = xf2.R
    tVec = circle2.m_p
    p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dX = p2X - p1X
    dY = p2Y - p1Y
    distSqr = dX * dX + dY * dY
    radius = circle1.m_radius + circle2.m_radius
    return  if distSqr > radius * radius
    manifold.m_type = b2Manifold.e_circles
    manifold.m_localPoint.SetV circle1.m_p
    manifold.m_localPlaneNormal.SetZero()
    manifold.m_pointCount = 1
    manifold.m_points[0].m_localPoint.SetV circle2.m_p
    manifold.m_points[0].m_id.key = 0
    return

  b2Collision.CollidePolygonAndCircle = (manifold, polygon, xf1, circle, xf2) ->
    manifold.m_pointCount = 0
    tPoint = undefined
    dX = 0
    dY = 0
    positionX = 0
    positionY = 0
    tVec = undefined
    tMat = undefined
    tMat = xf2.R
    tVec = circle.m_p
    cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    dX = cX - xf1.position.x
    dY = cY - xf1.position.y
    tMat = xf1.R
    cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y)
    cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y)
    dist = 0
    normalIndex = 0
    separation = (-Number.MAX_VALUE)
    radius = polygon.m_radius + circle.m_radius
    vertexCount = parseInt(polygon.m_vertexCount)
    vertices = polygon.m_vertices
    normals = polygon.m_normals
    i = 0

    while i < vertexCount
      tVec = vertices[i]
      dX = cLocalX - tVec.x
      dY = cLocalY - tVec.y
      tVec = normals[i]
      s = tVec.x * dX + tVec.y * dY
      return  if s > radius
      if s > separation
        separation = s
        normalIndex = i
      ++i
    vertIndex1 = parseInt(normalIndex)
    vertIndex2 = parseInt((if vertIndex1 + 1 < vertexCount then vertIndex1 + 1 else 0))
    v1 = vertices[vertIndex1]
    v2 = vertices[vertIndex2]
    if separation < Number.MIN_VALUE
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.SetV normals[normalIndex]
      manifold.m_localPoint.x = 0.5 * (v1.x + v2.x)
      manifold.m_localPoint.y = 0.5 * (v1.y + v2.y)
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
      return
    u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y)
    u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y)
    if u1 <= 0.0
      return  if (cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = cLocalX - v1.x
      manifold.m_localPlaneNormal.y = cLocalY - v1.y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.SetV v1
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    else if u2 <= 0
      return  if (cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = cLocalX - v2.x
      manifold.m_localPlaneNormal.y = cLocalY - v2.y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.SetV v2
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    else
      faceCenterX = 0.5 * (v1.x + v2.x)
      faceCenterY = 0.5 * (v1.y + v2.y)
      separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y
      return  if separation > radius
      manifold.m_pointCount = 1
      manifold.m_type = b2Manifold.e_faceA
      manifold.m_localPlaneNormal.x = normals[vertIndex1].x
      manifold.m_localPlaneNormal.y = normals[vertIndex1].y
      manifold.m_localPlaneNormal.Normalize()
      manifold.m_localPoint.Set faceCenterX, faceCenterY
      manifold.m_points[0].m_localPoint.SetV circle.m_p
      manifold.m_points[0].m_id.key = 0
    return

  b2Collision.TestOverlap = (a, b) ->
    t1 = b.lowerBound
    t2 = a.upperBound
    d1X = t1.x - t2.x
    d1Y = t1.y - t2.y
    t1 = a.lowerBound
    t2 = b.upperBound
    d2X = t1.x - t2.x
    d2Y = t1.y - t2.y
    return false  if d1X > 0.0 or d1Y > 0.0
    return false  if d2X > 0.0 or d2Y > 0.0
    true

  Box2D.postDefs.push ->
    Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector()
    Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1)
    Box2D.Collision.b2Collision.s_edgeBO = new Vector_a2j_Number(1)
    Box2D.Collision.b2Collision.s_localTangent = new b2Vec2()
    Box2D.Collision.b2Collision.s_localNormal = new b2Vec2()
    Box2D.Collision.b2Collision.s_planePoint = new b2Vec2()
    Box2D.Collision.b2Collision.s_normal = new b2Vec2()
    Box2D.Collision.b2Collision.s_tangent = new b2Vec2()
    Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2()
    Box2D.Collision.b2Collision.s_v11 = new b2Vec2()
    Box2D.Collision.b2Collision.s_v12 = new b2Vec2()
    Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2()
    Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff
    return

  b2ContactID.b2ContactID = ->
    @features = new Features()
    return

  b2ContactID::b2ContactID = ->
    @features._m_id = this
    return

  b2ContactID::Set = (id) ->
    @key = id._key
    return

  b2ContactID::Copy = ->
    id = new b2ContactID()
    id.key = @key
    id

  Object.defineProperty b2ContactID::, "key",
    enumerable: false
    configurable: true
    get: ->
      @_key

  Object.defineProperty b2ContactID::, "key",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_key = value
      @features._referenceEdge = @_key & 0x000000ff
      @features._incidentEdge = ((@_key & 0x0000ff00) >> 8) & 0x000000ff
      @features._incidentVertex = ((@_key & 0x00ff0000) >> 16) & 0x000000ff
      @features._flip = ((@_key & 0xff000000) >> 24) & 0x000000ff
      return

  b2ContactPoint.b2ContactPoint = ->
    @position = new b2Vec2()
    @velocity = new b2Vec2()
    @normal = new b2Vec2()
    @id = new b2ContactID()
    return

  b2Distance.b2Distance = ->

  b2Distance.Distance = (output, cache, input) ->
    ++b2Distance.b2_gjkCalls
    proxyA = input.proxyA
    proxyB = input.proxyB
    transformA = input.transformA
    transformB = input.transformB
    simplex = b2Distance.s_simplex
    simplex.ReadCache cache, proxyA, transformA, proxyB, transformB
    vertices = simplex.m_vertices
    k_maxIters = 20
    saveA = b2Distance.s_saveA
    saveB = b2Distance.s_saveB
    saveCount = 0
    closestPoint = simplex.GetClosestPoint()
    distanceSqr1 = closestPoint.LengthSquared()
    distanceSqr2 = distanceSqr1
    i = 0
    p = undefined
    iter = 0
    while iter < k_maxIters
      saveCount = simplex.m_count
      i = 0
      while i < saveCount
        saveA[i] = vertices[i].indexA
        saveB[i] = vertices[i].indexB
        i++
      switch simplex.m_count
        when 1, 2
          simplex.Solve2()
        when 3
          simplex.Solve3()
        else
          b2Settings.b2Assert false
      break  if simplex.m_count is 3
      p = simplex.GetClosestPoint()
      distanceSqr2 = p.LengthSquared()
      
      ###*
      todo: this looks like a bug
      ###
      
      #if (distanceSqr2 > distanceSqr1) {}
      distanceSqr1 = distanceSqr2
      d = simplex.GetSearchDirection()
      break  if d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE
      vertex = vertices[simplex.m_count]
      vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()))
      vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA))
      vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d))
      vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB))
      vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA)
      ++iter
      ++b2Distance.b2_gjkIters
      duplicate = false
      i = 0
      while i < saveCount
        if vertex.indexA is saveA[i] and vertex.indexB is saveB[i]
          duplicate = true
          break
        i++
      break  if duplicate
      ++simplex.m_count
    b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter)
    simplex.GetWitnessPoints output.pointA, output.pointB
    output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length()
    output.iterations = iter
    simplex.WriteCache cache
    if input.useRadii
      rA = proxyA.m_radius
      rB = proxyB.m_radius
      if output.distance > rA + rB and output.distance > Number.MIN_VALUE
        output.distance -= rA + rB
        normal = b2Math.SubtractVV(output.pointB, output.pointA)
        normal.Normalize()
        output.pointA.x += rA * normal.x
        output.pointA.y += rA * normal.y
        output.pointB.x -= rB * normal.x
        output.pointB.y -= rB * normal.y
      else
        p = new b2Vec2()
        p.x = .5 * (output.pointA.x + output.pointB.x)
        p.y = .5 * (output.pointA.y + output.pointB.y)
        output.pointA.x = output.pointB.x = p.x
        output.pointA.y = output.pointB.y = p.y
        output.distance = 0.0
    return

  Box2D.postDefs.push ->
    Box2D.Collision.b2Distance.s_simplex = new b2Simplex()
    Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3)
    Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3)
    return

  b2DistanceInput.b2DistanceInput = ->

  b2DistanceOutput.b2DistanceOutput = ->
    @pointA = new b2Vec2()
    @pointB = new b2Vec2()
    return

  b2DistanceProxy.b2DistanceProxy = ->

  b2DistanceProxy::Set = (shape) ->
    switch shape.GetType()
      when b2Shape.e_circleShape
        circle = ((if shape instanceof b2CircleShape then shape else null))
        @m_vertices = new Vector(1, true)
        @m_vertices[0] = circle.m_p
        @m_count = 1
        @m_radius = circle.m_radius
      when b2Shape.e_polygonShape
        polygon = ((if shape instanceof b2PolygonShape then shape else null))
        @m_vertices = polygon.m_vertices
        @m_count = polygon.m_vertexCount
        @m_radius = polygon.m_radius
      else
        b2Settings.b2Assert false
    return

  b2DistanceProxy::GetSupport = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    bestIndex

  b2DistanceProxy::GetSupportVertex = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_count
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    @m_vertices[bestIndex]

  b2DistanceProxy::GetVertexCount = ->
    @m_count

  b2DistanceProxy::GetVertex = (index) ->
    index = 0  if index is `undefined`
    b2Settings.b2Assert 0 <= index and index < @m_count
    @m_vertices[index]

  b2DynamicTree.b2DynamicTree = ->

  b2DynamicTree::b2DynamicTree = ->
    @m_root = null
    @m_freeList = null
    @m_path = 0
    @m_insertionCount = 0
    return

  b2DynamicTree::CreateProxy = (aabb, userData) ->
    node = @AllocateNode()
    extendX = b2Settings.b2_aabbExtension
    extendY = b2Settings.b2_aabbExtension
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    node.aabb.upperBound.x = aabb.upperBound.x + extendX
    node.aabb.upperBound.y = aabb.upperBound.y + extendY
    node.userData = userData
    @InsertLeaf node
    node

  b2DynamicTree::DestroyProxy = (proxy) ->
    @RemoveLeaf proxy
    @FreeNode proxy
    return

  b2DynamicTree::MoveProxy = (proxy, aabb, displacement) ->
    b2Settings.b2Assert proxy.IsLeaf()
    return false  if proxy.aabb.Contains(aabb)
    @RemoveLeaf proxy
    extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.x > 0 then displacement.x else (-displacement.x)))
    extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.y > 0 then displacement.y else (-displacement.y)))
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY
    @InsertLeaf proxy
    true

  b2DynamicTree::Rebalance = (iterations) ->
    iterations = 0  if iterations is `undefined`
    return  unless @m_root?
    i = 0

    while i < iterations
      node = @m_root
      bit = 0
      while node.IsLeaf() is false
        node = (if (@m_path >> bit) & 1 then node.child2 else node.child1)
        bit = (bit + 1) & 31
      ++@m_path
      @RemoveLeaf node
      @InsertLeaf node
      i++
    return

  b2DynamicTree::GetFatAABB = (proxy) ->
    proxy.aabb

  b2DynamicTree::GetUserData = (proxy) ->
    proxy.userData

  b2DynamicTree::Query = (callback, aabb) ->
    return  unless @m_root?
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      if node.aabb.TestOverlap(aabb)
        if node.IsLeaf()
          proceed = callback(node)
          return  unless proceed
        else
          stack[count++] = node.child1
          stack[count++] = node.child2
    return

  b2DynamicTree::RayCast = (callback, input) ->
    return  unless @m_root?
    p1 = input.p1
    p2 = input.p2
    r = b2Math.SubtractVV(p1, p2)
    r.Normalize()
    v = b2Math.CrossFV(1.0, r)
    abs_v = b2Math.AbsV(v)
    maxFraction = input.maxFraction
    segmentAABB = new b2AABB()
    tX = 0
    tY = 0
    tX = p1.x + maxFraction * (p2.x - p1.x)
    tY = p1.y + maxFraction * (p2.y - p1.y)
    segmentAABB.lowerBound.x = Math.min(p1.x, tX)
    segmentAABB.lowerBound.y = Math.min(p1.y, tY)
    segmentAABB.upperBound.x = Math.max(p1.x, tX)
    segmentAABB.upperBound.y = Math.max(p1.y, tY)
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      continue  if node.aabb.TestOverlap(segmentAABB) is false
      c = node.aabb.GetCenter()
      h = node.aabb.GetExtents()
      separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y
      continue  if separation > 0.0
      if node.IsLeaf()
        subInput = new b2RayCastInput()
        subInput.p1 = input.p1
        subInput.p2 = input.p2
        subInput.maxFraction = input.maxFraction
        maxFraction = callback(subInput, node)
        return  if maxFraction is 0.0
        if maxFraction > 0.0
          tX = p1.x + maxFraction * (p2.x - p1.x)
          tY = p1.y + maxFraction * (p2.y - p1.y)
          segmentAABB.lowerBound.x = Math.min(p1.x, tX)
          segmentAABB.lowerBound.y = Math.min(p1.y, tY)
          segmentAABB.upperBound.x = Math.max(p1.x, tX)
          segmentAABB.upperBound.y = Math.max(p1.y, tY)
      else
        stack[count++] = node.child1
        stack[count++] = node.child2
    return

  b2DynamicTree::AllocateNode = ->
    if @m_freeList
      node = @m_freeList
      @m_freeList = node.parent
      node.parent = null
      node.child1 = null
      node.child2 = null
      return node
    new b2DynamicTreeNode()

  b2DynamicTree::FreeNode = (node) ->
    node.parent = @m_freeList
    @m_freeList = node
    return

  b2DynamicTree::InsertLeaf = (leaf) ->
    ++@m_insertionCount
    unless @m_root?
      @m_root = leaf
      @m_root.parent = null
      return
    center = leaf.aabb.GetCenter()
    sibling = @m_root
    if sibling.IsLeaf() is false
      loop
        child1 = sibling.child1
        child2 = sibling.child2
        norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y)
        norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y)
        if norm1 < norm2
          sibling = child1
        else
          sibling = child2
        break unless sibling.IsLeaf() is false
    node1 = sibling.parent
    node2 = @AllocateNode()
    node2.parent = node1
    node2.userData = null
    node2.aabb.Combine leaf.aabb, sibling.aabb
    if node1
      if sibling.parent.child1 is sibling
        node1.child1 = node2
      else
        node1.child2 = node2
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      loop
        break  if node1.aabb.Contains(node2.aabb)
        node1.aabb.Combine node1.child1.aabb, node1.child2.aabb
        node2 = node1
        node1 = node1.parent
        break unless node1
    else
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      @m_root = node2
    return

  b2DynamicTree::RemoveLeaf = (leaf) ->
    if leaf is @m_root
      @m_root = null
      return
    node2 = leaf.parent
    node1 = node2.parent
    sibling = undefined
    if node2.child1 is leaf
      sibling = node2.child2
    else
      sibling = node2.child1
    if node1
      if node1.child1 is node2
        node1.child1 = sibling
      else
        node1.child2 = sibling
      sibling.parent = node1
      @FreeNode node2
      while node1
        oldAABB = node1.aabb
        node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb)
        break  if oldAABB.Contains(node1.aabb)
        node1 = node1.parent
    else
      @m_root = sibling
      sibling.parent = null
      @FreeNode node2
    return

  b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase = ->
    @m_tree = new b2DynamicTree()
    @m_moveBuffer = new Vector()
    @m_pairBuffer = new Vector()
    @m_pairCount = 0
    return

  b2DynamicTreeBroadPhase::CreateProxy = (aabb, userData) ->
    proxy = @m_tree.CreateProxy(aabb, userData)
    ++@m_proxyCount
    @BufferMove proxy
    proxy

  b2DynamicTreeBroadPhase::DestroyProxy = (proxy) ->
    @UnBufferMove proxy
    --@m_proxyCount
    @m_tree.DestroyProxy proxy
    return

  b2DynamicTreeBroadPhase::MoveProxy = (proxy, aabb, displacement) ->
    buffer = @m_tree.MoveProxy(proxy, aabb, displacement)
    @BufferMove proxy  if buffer
    return

  b2DynamicTreeBroadPhase::TestOverlap = (proxyA, proxyB) ->
    aabbA = @m_tree.GetFatAABB(proxyA)
    aabbB = @m_tree.GetFatAABB(proxyB)
    aabbA.TestOverlap aabbB

  b2DynamicTreeBroadPhase::GetUserData = (proxy) ->
    @m_tree.GetUserData proxy

  b2DynamicTreeBroadPhase::GetFatAABB = (proxy) ->
    @m_tree.GetFatAABB proxy

  b2DynamicTreeBroadPhase::GetProxyCount = ->
    @m_proxyCount

  b2DynamicTreeBroadPhase::UpdatePairs = (callback) ->
    __this = this
    __this.m_pairCount = 0
    i = 0
    queryProxy = undefined
    i = 0
    while i < __this.m_moveBuffer.length
      QueryCallback = (proxy) ->
        return true  if proxy is queryProxy
        __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair()  if __this.m_pairCount is __this.m_pairBuffer.length
        pair = __this.m_pairBuffer[__this.m_pairCount]
        pair.proxyA = (if proxy < queryProxy then proxy else queryProxy)
        pair.proxyB = (if proxy >= queryProxy then proxy else queryProxy)
        ++__this.m_pairCount
        true
      queryProxy = __this.m_moveBuffer[i]
      fatAABB = __this.m_tree.GetFatAABB(queryProxy)
      __this.m_tree.Query QueryCallback, fatAABB
      ++i
    __this.m_moveBuffer.length = 0
    i = 0

    while i < __this.m_pairCount
      primaryPair = __this.m_pairBuffer[i]
      userDataA = __this.m_tree.GetUserData(primaryPair.proxyA)
      userDataB = __this.m_tree.GetUserData(primaryPair.proxyB)
      callback userDataA, userDataB
      ++i
      while i < __this.m_pairCount
        pair = __this.m_pairBuffer[i]
        break  if pair.proxyA isnt primaryPair.proxyA or pair.proxyB isnt primaryPair.proxyB
        ++i
    return

  b2DynamicTreeBroadPhase::Query = (callback, aabb) ->
    @m_tree.Query callback, aabb
    return

  b2DynamicTreeBroadPhase::RayCast = (callback, input) ->
    @m_tree.RayCast callback, input
    return

  b2DynamicTreeBroadPhase::Validate = ->

  b2DynamicTreeBroadPhase::Rebalance = (iterations) ->
    iterations = 0  if iterations is `undefined`
    @m_tree.Rebalance iterations
    return

  b2DynamicTreeBroadPhase::BufferMove = (proxy) ->
    @m_moveBuffer[@m_moveBuffer.length] = proxy
    return

  b2DynamicTreeBroadPhase::UnBufferMove = (proxy) ->
    i = parseInt(@m_moveBuffer.indexOf(proxy))
    @m_moveBuffer.splice i, 1
    return

  b2DynamicTreeBroadPhase::ComparePairs = (pair1, pair2) ->
    0

  b2DynamicTreeBroadPhase.__implements = {}
  b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true
  b2DynamicTreeNode.b2DynamicTreeNode = ->
    @aabb = new b2AABB()
    return

  b2DynamicTreeNode::IsLeaf = ->
    not @child1?

  b2DynamicTreePair.b2DynamicTreePair = ->

  b2Manifold.b2Manifold = ->
    @m_pointCount = 0
    return

  b2Manifold::b2Manifold = ->
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2ManifoldPoint()
      i++
    @m_localPlaneNormal = new b2Vec2()
    @m_localPoint = new b2Vec2()
    return

  b2Manifold::Reset = ->
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Reset()
      i++
    @m_localPlaneNormal.SetZero()
    @m_localPoint.SetZero()
    @m_type = 0
    @m_pointCount = 0
    return

  b2Manifold::Set = (m) ->
    @m_pointCount = m.m_pointCount
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      ((if @m_points[i] instanceof b2ManifoldPoint then @m_points[i] else null)).Set m.m_points[i]
      i++
    @m_localPlaneNormal.SetV m.m_localPlaneNormal
    @m_localPoint.SetV m.m_localPoint
    @m_type = m.m_type
    return

  b2Manifold::Copy = ->
    copy = new b2Manifold()
    copy.Set this
    copy

  Box2D.postDefs.push ->
    Box2D.Collision.b2Manifold.e_circles = 0x0001
    Box2D.Collision.b2Manifold.e_faceA = 0x0002
    Box2D.Collision.b2Manifold.e_faceB = 0x0004
    return

  b2ManifoldPoint.b2ManifoldPoint = ->
    @m_localPoint = new b2Vec2()
    @m_id = new b2ContactID()
    return

  b2ManifoldPoint::b2ManifoldPoint = ->
    @Reset()
    return

  b2ManifoldPoint::Reset = ->
    @m_localPoint.SetZero()
    @m_normalImpulse = 0.0
    @m_tangentImpulse = 0.0
    @m_id.key = 0
    return

  b2ManifoldPoint::Set = (m) ->
    @m_localPoint.SetV m.m_localPoint
    @m_normalImpulse = m.m_normalImpulse
    @m_tangentImpulse = m.m_tangentImpulse
    @m_id.Set m.m_id
    return

  b2Point.b2Point = ->
    @p = new b2Vec2()
    return

  b2Point::Support = (xf, vX, vY) ->
    vX = 0  if vX is `undefined`
    vY = 0  if vY is `undefined`
    @p

  b2Point::GetFirstVertex = (xf) ->
    @p

  b2RayCastInput.b2RayCastInput = ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    return

  b2RayCastInput::b2RayCastInput = (p1, p2, maxFraction) ->
    p1 = null  if p1 is `undefined`
    p2 = null  if p2 is `undefined`
    maxFraction = 1  if maxFraction is `undefined`
    @p1.SetV p1  if p1
    @p2.SetV p2  if p2
    @maxFraction = maxFraction
    return

  b2RayCastOutput.b2RayCastOutput = ->
    @normal = new b2Vec2()
    return

  b2Segment.b2Segment = ->
    @p1 = new b2Vec2()
    @p2 = new b2Vec2()
    return

  b2Segment::TestSegment = (lambda, normal, segment, maxLambda) ->
    maxLambda = 0  if maxLambda is `undefined`
    s = segment.p1
    rX = segment.p2.x - s.x
    rY = segment.p2.y - s.y
    dX = @p2.x - @p1.x
    dY = @p2.y - @p1.y
    nX = dY
    nY = (-dX)
    k_slop = 100.0 * Number.MIN_VALUE
    denom = (-(rX * nX + rY * nY))
    if denom > k_slop
      bX = s.x - @p1.x
      bY = s.y - @p1.y
      a = (bX * nX + bY * nY)
      if 0.0 <= a and a <= maxLambda * denom
        mu2 = (-rX * bY) + rY * bX
        if (-k_slop * denom) <= mu2 and mu2 <= denom * (1.0 + k_slop)
          a /= denom
          nLen = Math.sqrt(nX * nX + nY * nY)
          nX /= nLen
          nY /= nLen
          lambda[0] = a
          normal.Set nX, nY
          return true
    false

  b2Segment::Extend = (aabb) ->
    @ExtendForward aabb
    @ExtendBackward aabb
    return

  b2Segment::ExtendForward = (aabb) ->
    dX = @p2.x - @p1.x
    dY = @p2.y - @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p1.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p1.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p1.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p1.y) / dY else Number.POSITIVE_INFINITY)))
    @p2.x = @p1.x + dX * lambda
    @p2.y = @p1.y + dY * lambda
    return

  b2Segment::ExtendBackward = (aabb) ->
    dX = (-@p2.x) + @p1.x
    dY = (-@p2.y) + @p1.y
    lambda = Math.min((if dX > 0 then (aabb.upperBound.x - @p2.x) / dX else (if dX < 0 then (aabb.lowerBound.x - @p2.x) / dX else Number.POSITIVE_INFINITY)), (if dY > 0 then (aabb.upperBound.y - @p2.y) / dY else (if dY < 0 then (aabb.lowerBound.y - @p2.y) / dY else Number.POSITIVE_INFINITY)))
    @p1.x = @p2.x + dX * lambda
    @p1.y = @p2.y + dY * lambda
    return

  b2SeparationFunction.b2SeparationFunction = ->
    @m_localPoint = new b2Vec2()
    @m_axis = new b2Vec2()
    return

  b2SeparationFunction::Initialize = (cache, proxyA, transformA, proxyB, transformB) ->
    @m_proxyA = proxyA
    @m_proxyB = proxyB
    count = parseInt(cache.count)
    b2Settings.b2Assert 0 < count and count < 3
    localPointA = undefined
    localPointA1 = undefined
    localPointA2 = undefined
    localPointB = undefined
    localPointB1 = undefined
    localPointB2 = undefined
    pointAX = 0
    pointAY = 0
    pointBX = 0
    pointBY = 0
    normalX = 0
    normalY = 0
    tMat = undefined
    tVec = undefined
    s = 0
    sgn = 0
    if count is 1
      @m_type = b2SeparationFunction.e_points
      localPointA = @m_proxyA.GetVertex(cache.indexA[0])
      localPointB = @m_proxyB.GetVertex(cache.indexB[0])
      tVec = localPointA
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointB
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      @m_axis.x = pointBX - pointAX
      @m_axis.y = pointBY - pointAY
      @m_axis.Normalize()
    else if cache.indexB[0] is cache.indexB[1]
      @m_type = b2SeparationFunction.e_faceA
      localPointA1 = @m_proxyA.GetVertex(cache.indexA[0])
      localPointA2 = @m_proxyA.GetVertex(cache.indexA[1])
      localPointB = @m_proxyB.GetVertex(cache.indexB[0])
      @m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x)
      @m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y)
      @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0)
      @m_axis.Normalize()
      tVec = @m_axis
      tMat = transformA.R
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tVec = @m_localPoint
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointB
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY
      @m_axis.NegativeSelf()  if s < 0.0
    else if cache.indexA[0] is cache.indexA[0]
      @m_type = b2SeparationFunction.e_faceB
      localPointB1 = @m_proxyB.GetVertex(cache.indexB[0])
      localPointB2 = @m_proxyB.GetVertex(cache.indexB[1])
      localPointA = @m_proxyA.GetVertex(cache.indexA[0])
      @m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x)
      @m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y)
      @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0)
      @m_axis.Normalize()
      tVec = @m_axis
      tMat = transformB.R
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tVec = @m_localPoint
      tMat = transformB.R
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      tVec = localPointA
      tMat = transformA.R
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY
      @m_axis.NegativeSelf()  if s < 0.0
    else
      localPointA1 = @m_proxyA.GetVertex(cache.indexA[0])
      localPointA2 = @m_proxyA.GetVertex(cache.indexA[1])
      localPointB1 = @m_proxyB.GetVertex(cache.indexB[0])
      localPointB2 = @m_proxyB.GetVertex(cache.indexB[1])
      pA = b2Math.MulX(transformA, localPointA)
      dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1))
      pB = b2Math.MulX(transformB, localPointB)
      dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1))
      a = dA.x * dA.x + dA.y * dA.y
      e = dB.x * dB.x + dB.y * dB.y
      r = b2Math.SubtractVV(dB, dA)
      c = dA.x * r.x + dA.y * r.y
      f = dB.x * r.x + dB.y * r.y
      b = dA.x * dB.x + dA.y * dB.y
      denom = a * e - b * b
      s = 0.0
      s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0)  unless denom is 0.0
      t = (b * s + f) / e
      if t < 0.0
        t = 0.0
        s = b2Math.Clamp((b - c) / a, 0.0, 1.0)
      localPointA = new b2Vec2()
      localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x)
      localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y)
      localPointB = new b2Vec2()
      localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x)
      localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y)
      if s is 0.0 or s is 1.0
        @m_type = b2SeparationFunction.e_faceB
        @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0)
        @m_axis.Normalize()
        @m_localPoint = localPointB
        tVec = @m_axis
        tMat = transformB.R
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tVec = @m_localPoint
        tMat = transformB.R
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tVec = localPointA
        tMat = transformA.R
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY
        @m_axis.NegativeSelf()  if s < 0.0
      else
        @m_type = b2SeparationFunction.e_faceA
        @m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0)
        @m_localPoint = localPointA
        tVec = @m_axis
        tMat = transformA.R
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tVec = @m_localPoint
        tMat = transformA.R
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        tVec = localPointB
        tMat = transformB.R
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
        sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY
        @m_axis.NegativeSelf()  if s < 0.0
    return

  b2SeparationFunction::Evaluate = (transformA, transformB) ->
    axisA = undefined
    axisB = undefined
    localPointA = undefined
    localPointB = undefined
    pointA = undefined
    pointB = undefined
    seperation = 0
    normal = undefined
    switch @m_type
      when b2SeparationFunction.e_points
        axisA = b2Math.MulTMV(transformA.R, @m_axis)
        axisB = b2Math.MulTMV(transformB.R, @m_axis.GetNegative())
        localPointA = @m_proxyA.GetSupportVertex(axisA)
        localPointB = @m_proxyB.GetSupportVertex(axisB)
        pointA = b2Math.MulX(transformA, localPointA)
        pointB = b2Math.MulX(transformB, localPointB)
        seperation = (pointB.x - pointA.x) * @m_axis.x + (pointB.y - pointA.y) * @m_axis.y
        seperation
      when b2SeparationFunction.e_faceA
        normal = b2Math.MulMV(transformA.R, @m_axis)
        pointA = b2Math.MulX(transformA, @m_localPoint)
        axisB = b2Math.MulTMV(transformB.R, normal.GetNegative())
        localPointB = @m_proxyB.GetSupportVertex(axisB)
        pointB = b2Math.MulX(transformB, localPointB)
        seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y
        seperation
      when b2SeparationFunction.e_faceB
        normal = b2Math.MulMV(transformB.R, @m_axis)
        pointB = b2Math.MulX(transformB, @m_localPoint)
        axisA = b2Math.MulTMV(transformA.R, normal.GetNegative())
        localPointA = @m_proxyA.GetSupportVertex(axisA)
        pointA = b2Math.MulX(transformA, localPointA)
        seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y
        seperation
      else
        b2Settings.b2Assert false
        0.0

  Box2D.postDefs.push ->
    Box2D.Collision.b2SeparationFunction.e_points = 0x01
    Box2D.Collision.b2SeparationFunction.e_faceA = 0x02
    Box2D.Collision.b2SeparationFunction.e_faceB = 0x04
    return

  b2Simplex.b2Simplex = ->
    @m_v1 = new b2SimplexVertex()
    @m_v2 = new b2SimplexVertex()
    @m_v3 = new b2SimplexVertex()
    @m_vertices = new Vector(3)
    return

  b2Simplex::b2Simplex = ->
    @m_vertices[0] = @m_v1
    @m_vertices[1] = @m_v2
    @m_vertices[2] = @m_v3
    return

  b2Simplex::ReadCache = (cache, proxyA, transformA, proxyB, transformB) ->
    b2Settings.b2Assert 0 <= cache.count and cache.count <= 3
    wALocal = undefined
    wBLocal = undefined
    @m_count = cache.count
    vertices = @m_vertices
    i = 0

    while i < @m_count
      v = vertices[i]
      v.indexA = cache.indexA[i]
      v.indexB = cache.indexB[i]
      wALocal = proxyA.GetVertex(v.indexA)
      wBLocal = proxyB.GetVertex(v.indexB)
      v.wA = b2Math.MulX(transformA, wALocal)
      v.wB = b2Math.MulX(transformB, wBLocal)
      v.w = b2Math.SubtractVV(v.wB, v.wA)
      v.a = 0
      i++
    if @m_count > 1
      metric1 = cache.metric
      metric2 = @GetMetric()
      @m_count = 0  if metric2 < .5 * metric1 or 2.0 * metric1 < metric2 or metric2 < Number.MIN_VALUE
    if @m_count is 0
      v = vertices[0]
      v.indexA = 0
      v.indexB = 0
      wALocal = proxyA.GetVertex(0)
      wBLocal = proxyB.GetVertex(0)
      v.wA = b2Math.MulX(transformA, wALocal)
      v.wB = b2Math.MulX(transformB, wBLocal)
      v.w = b2Math.SubtractVV(v.wB, v.wA)
      @m_count = 1
    return

  b2Simplex::WriteCache = (cache) ->
    cache.metric = @GetMetric()
    cache.count = Box2D.parseUInt(@m_count)
    vertices = @m_vertices
    i = 0

    while i < @m_count
      cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA)
      cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB)
      i++
    return

  b2Simplex::GetSearchDirection = ->
    switch @m_count
      when 1
        @m_v1.w.GetNegative()
      when 2
        e12 = b2Math.SubtractVV(@m_v2.w, @m_v1.w)
        sgn = b2Math.CrossVV(e12, @m_v1.w.GetNegative())
        if sgn > 0.0
          b2Math.CrossFV 1.0, e12
        else
          b2Math.CrossVF e12, 1.0
      else
        b2Settings.b2Assert false
        new b2Vec2()

  b2Simplex::GetClosestPoint = ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
        new b2Vec2()
      when 1
        @m_v1.w
      when 2
        new b2Vec2(@m_v1.a * @m_v1.w.x + @m_v2.a * @m_v2.w.x, @m_v1.a * @m_v1.w.y + @m_v2.a * @m_v2.w.y)
      else
        b2Settings.b2Assert false
        new b2Vec2()

  b2Simplex::GetWitnessPoints = (pA, pB) ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
      when 1
        pA.SetV @m_v1.wA
        pB.SetV @m_v1.wB
      when 2
        pA.x = @m_v1.a * @m_v1.wA.x + @m_v2.a * @m_v2.wA.x
        pA.y = @m_v1.a * @m_v1.wA.y + @m_v2.a * @m_v2.wA.y
        pB.x = @m_v1.a * @m_v1.wB.x + @m_v2.a * @m_v2.wB.x
        pB.y = @m_v1.a * @m_v1.wB.y + @m_v2.a * @m_v2.wB.y
      when 3
        pB.x = pA.x = @m_v1.a * @m_v1.wA.x + @m_v2.a * @m_v2.wA.x + @m_v3.a * @m_v3.wA.x
        pB.y = pA.y = @m_v1.a * @m_v1.wA.y + @m_v2.a * @m_v2.wA.y + @m_v3.a * @m_v3.wA.y
      else
        b2Settings.b2Assert false

  b2Simplex::GetMetric = ->
    switch @m_count
      when 0
        b2Settings.b2Assert false
        0.0
      when 1
        0.0
      when 2
        b2Math.SubtractVV(@m_v1.w, @m_v2.w).Length()
      when 3
        b2Math.CrossVV b2Math.SubtractVV(@m_v2.w, @m_v1.w), b2Math.SubtractVV(@m_v3.w, @m_v1.w)
      else
        b2Settings.b2Assert false
        0.0

  b2Simplex::Solve2 = ->
    w1 = @m_v1.w
    w2 = @m_v2.w
    e12 = b2Math.SubtractVV(w2, w1)
    d12_2 = (-(w1.x * e12.x + w1.y * e12.y))
    if d12_2 <= 0.0
      @m_v1.a = 1.0
      @m_count = 1
      return
    d12_1 = (w2.x * e12.x + w2.y * e12.y)
    if d12_1 <= 0.0
      @m_v2.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v2
      return
    inv_d12 = 1.0 / (d12_1 + d12_2)
    @m_v1.a = d12_1 * inv_d12
    @m_v2.a = d12_2 * inv_d12
    @m_count = 2
    return

  b2Simplex::Solve3 = ->
    w1 = @m_v1.w
    w2 = @m_v2.w
    w3 = @m_v3.w
    e12 = b2Math.SubtractVV(w2, w1)
    w1e12 = b2Math.Dot(w1, e12)
    w2e12 = b2Math.Dot(w2, e12)
    d12_1 = w2e12
    d12_2 = (-w1e12)
    e13 = b2Math.SubtractVV(w3, w1)
    w1e13 = b2Math.Dot(w1, e13)
    w3e13 = b2Math.Dot(w3, e13)
    d13_1 = w3e13
    d13_2 = (-w1e13)
    e23 = b2Math.SubtractVV(w3, w2)
    w2e23 = b2Math.Dot(w2, e23)
    w3e23 = b2Math.Dot(w3, e23)
    d23_1 = w3e23
    d23_2 = (-w2e23)
    n123 = b2Math.CrossVV(e12, e13)
    d123_1 = n123 * b2Math.CrossVV(w2, w3)
    d123_2 = n123 * b2Math.CrossVV(w3, w1)
    d123_3 = n123 * b2Math.CrossVV(w1, w2)
    if d12_2 <= 0.0 and d13_2 <= 0.0
      @m_v1.a = 1.0
      @m_count = 1
      return
    if d12_1 > 0.0 and d12_2 > 0.0 and d123_3 <= 0.0
      inv_d12 = 1.0 / (d12_1 + d12_2)
      @m_v1.a = d12_1 * inv_d12
      @m_v2.a = d12_2 * inv_d12
      @m_count = 2
      return
    if d13_1 > 0.0 and d13_2 > 0.0 and d123_2 <= 0.0
      inv_d13 = 1.0 / (d13_1 + d13_2)
      @m_v1.a = d13_1 * inv_d13
      @m_v3.a = d13_2 * inv_d13
      @m_count = 2
      @m_v2.Set @m_v3
      return
    if d12_1 <= 0.0 and d23_2 <= 0.0
      @m_v2.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v2
      return
    if d13_1 <= 0.0 and d23_1 <= 0.0
      @m_v3.a = 1.0
      @m_count = 1
      @m_v1.Set @m_v3
      return
    if d23_1 > 0.0 and d23_2 > 0.0 and d123_1 <= 0.0
      inv_d23 = 1.0 / (d23_1 + d23_2)
      @m_v2.a = d23_1 * inv_d23
      @m_v3.a = d23_2 * inv_d23
      @m_count = 2
      @m_v1.Set @m_v3
      return
    inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3)
    @m_v1.a = d123_1 * inv_d123
    @m_v2.a = d123_2 * inv_d123
    @m_v3.a = d123_3 * inv_d123
    @m_count = 3
    return

  b2SimplexCache.b2SimplexCache = ->
    @indexA = new Vector_a2j_Number(3)
    @indexB = new Vector_a2j_Number(3)
    return

  b2SimplexVertex.b2SimplexVertex = ->

  b2SimplexVertex::Set = (other) ->
    @wA.SetV other.wA
    @wB.SetV other.wB
    @w.SetV other.w
    @a = other.a
    @indexA = other.indexA
    @indexB = other.indexB
    return

  b2TimeOfImpact.b2TimeOfImpact = ->

  b2TimeOfImpact.TimeOfImpact = (input) ->
    ++b2TimeOfImpact.b2_toiCalls
    proxyA = input.proxyA
    proxyB = input.proxyB
    sweepA = input.sweepA
    sweepB = input.sweepB
    b2Settings.b2Assert sweepA.t0 is sweepB.t0
    b2Settings.b2Assert 1.0 - sweepA.t0 > Number.MIN_VALUE
    radius = proxyA.m_radius + proxyB.m_radius
    tolerance = input.tolerance
    alpha = 0.0
    k_maxIterations = 1000
    iter = 0
    target = 0.0
    b2TimeOfImpact.s_cache.count = 0
    b2TimeOfImpact.s_distanceInput.useRadii = false
    loop
      sweepA.GetTransform b2TimeOfImpact.s_xfA, alpha
      sweepB.GetTransform b2TimeOfImpact.s_xfB, alpha
      b2TimeOfImpact.s_distanceInput.proxyA = proxyA
      b2TimeOfImpact.s_distanceInput.proxyB = proxyB
      b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA
      b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB
      b2Distance.Distance b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput
      if b2TimeOfImpact.s_distanceOutput.distance <= 0.0
        alpha = 1.0
        break
      b2TimeOfImpact.s_fcn.Initialize b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB
      separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if separation <= 0.0
        alpha = 1.0
        break
      if iter is 0
        if separation > radius
          target = b2Math.Max(radius - tolerance, 0.75 * radius)
        else
          target = b2Math.Max(separation - tolerance, 0.02 * radius)
      if separation - target < 0.5 * tolerance
        if iter is 0
          alpha = 1.0
          break
        break
      newAlpha = alpha
      x1 = alpha
      x2 = 1.0
      f1 = separation
      sweepA.GetTransform b2TimeOfImpact.s_xfA, x2
      sweepB.GetTransform b2TimeOfImpact.s_xfB, x2
      f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if f2 >= target
        alpha = 1.0
        break
      rootIterCount = 0
      loop
        x = 0
        if rootIterCount & 1
          x = x1 + (target - f1) * (x2 - x1) / (f2 - f1)
        else
          x = 0.5 * (x1 + x2)
        sweepA.GetTransform b2TimeOfImpact.s_xfA, x
        sweepB.GetTransform b2TimeOfImpact.s_xfB, x
        f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
        if b2Math.Abs(f - target) < 0.025 * tolerance
          newAlpha = x
          break
        if f > target
          x1 = x
          f1 = f
        else
          x2 = x
          f2 = f
        ++rootIterCount
        ++b2TimeOfImpact.b2_toiRootIters
        break  if rootIterCount is 50
      b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount)
      break  if newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha
      alpha = newAlpha
      iter++
      ++b2TimeOfImpact.b2_toiIters
      break  if iter is k_maxIterations
    b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter)
    alpha

  Box2D.postDefs.push ->
    Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0
    Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache()
    Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput()
    Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform()
    Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform()
    Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction()
    Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput()
    return

  b2TOIInput.b2TOIInput = ->
    @proxyA = new b2DistanceProxy()
    @proxyB = new b2DistanceProxy()
    @sweepA = new b2Sweep()
    @sweepB = new b2Sweep()
    return

  b2WorldManifold.b2WorldManifold = ->
    @m_normal = new b2Vec2()
    return

  b2WorldManifold::b2WorldManifold = ->
    @m_points = new Vector(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @m_points[i] = new b2Vec2()
      i++
    return

  b2WorldManifold::Initialize = (manifold, xfA, radiusA, xfB, radiusB) ->
    radiusA = 0  if radiusA is `undefined`
    radiusB = 0  if radiusB is `undefined`
    return  if manifold.m_pointCount is 0
    i = 0
    tVec = undefined
    tMat = undefined
    normalX = 0
    normalY = 0
    planePointX = 0
    planePointY = 0
    clipPointX = 0
    clipPointY = 0
    switch manifold.m_type
      when b2Manifold.e_circles
        tMat = xfA.R
        tVec = manifold.m_localPoint
        pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfB.R
        tVec = manifold.m_points[0].m_localPoint
        pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        dX = pointBX - pointAX
        dY = pointBY - pointAY
        d2 = dX * dX + dY * dY
        if d2 > Number.MIN_VALUE * Number.MIN_VALUE
          d = Math.sqrt(d2)
          @m_normal.x = dX / d
          @m_normal.y = dY / d
        else
          @m_normal.x = 1
          @m_normal.y = 0
        cAX = pointAX + radiusA * @m_normal.x
        cAY = pointAY + radiusA * @m_normal.y
        cBX = pointBX - radiusB * @m_normal.x
        cBY = pointBY - radiusB * @m_normal.y
        @m_points[0].x = 0.5 * (cAX + cBX)
        @m_points[0].y = 0.5 * (cAY + cBY)
      when b2Manifold.e_faceA
        tMat = xfA.R
        tVec = manifold.m_localPlaneNormal
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfA.R
        tVec = manifold.m_localPoint
        planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        @m_normal.x = normalX
        @m_normal.y = normalY
        i = 0
        while i < manifold.m_pointCount
          tMat = xfB.R
          tVec = manifold.m_points[i].m_localPoint
          clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
          clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
          @m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX
          @m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY
          i++
      when b2Manifold.e_faceB
        tMat = xfB.R
        tVec = manifold.m_localPlaneNormal
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        tMat = xfB.R
        tVec = manifold.m_localPoint
        planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
        planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
        @m_normal.x = (-normalX)
        @m_normal.y = (-normalY)
        i = 0
        while i < manifold.m_pointCount
          tMat = xfA.R
          tVec = manifold.m_points[i].m_localPoint
          clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
          clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
          @m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX
          @m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY
          i++

  ClipVertex.ClipVertex = ->
    @v = new b2Vec2()
    @id = new b2ContactID()
    return

  ClipVertex::Set = (other) ->
    @v.SetV other.v
    @id.Set other.id
    return

  Features.Features = ->

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    get: ->
      @_referenceEdge

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_referenceEdge = value
      @_m_id._key = (@_m_id._key & 0xffffff00) | (@_referenceEdge & 0x000000ff)
      return

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    get: ->
      @_incidentEdge

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentEdge = value
      @_m_id._key = (@_m_id._key & 0xffff00ff) | ((@_incidentEdge << 8) & 0x0000ff00)
      return

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    get: ->
      @_incidentVertex

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentVertex = value
      @_m_id._key = (@_m_id._key & 0xff00ffff) | ((@_incidentVertex << 16) & 0x00ff0000)
      return

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    get: ->
      @_flip

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_flip = value
      @_m_id._key = (@_m_id._key & 0x00ffffff) | ((@_flip << 24) & 0xff000000)
      return

  return
)()
(->
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
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
  Box2D.inherit b2CircleShape, Box2D.Collision.Shapes.b2Shape
  b2CircleShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2CircleShape.b2CircleShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    @m_p = new b2Vec2()
    return

  b2CircleShape::Copy = ->
    s = new b2CircleShape()
    s.Set this
    s

  b2CircleShape::Set = (other) ->
    @__super.Set.call this, other
    if Box2D.equal(other, b2CircleShape)
      other2 = ((if other instanceof b2CircleShape then other else null))
      @m_p.SetV other2.m_p
    return

  b2CircleShape::TestPoint = (transform, p) ->
    tMat = transform.R
    dX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    dY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    dX = p.x - dX
    dY = p.y - dY
    (dX * dX + dY * dY) <= @m_radius * @m_radius

  b2CircleShape::RayCast = (output, input, transform) ->
    tMat = transform.R
    positionX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    positionY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    sX = input.p1.x - positionX
    sY = input.p1.y - positionY
    b = (sX * sX + sY * sY) - @m_radius * @m_radius
    rX = input.p2.x - input.p1.x
    rY = input.p2.y - input.p1.y
    c = (sX * rX + sY * rY)
    rr = (rX * rX + rY * rY)
    sigma = c * c - rr * b
    return false  if sigma < 0.0 or rr < Number.MIN_VALUE
    a = (-(c + Math.sqrt(sigma)))
    if 0.0 <= a and a <= input.maxFraction * rr
      a /= rr
      output.fraction = a
      output.normal.x = sX + a * rX
      output.normal.y = sY + a * rY
      output.normal.Normalize()
      return true
    false

  b2CircleShape::ComputeAABB = (aabb, transform) ->
    tMat = transform.R
    pX = transform.position.x + (tMat.col1.x * @m_p.x + tMat.col2.x * @m_p.y)
    pY = transform.position.y + (tMat.col1.y * @m_p.x + tMat.col2.y * @m_p.y)
    aabb.lowerBound.Set pX - @m_radius, pY - @m_radius
    aabb.upperBound.Set pX + @m_radius, pY + @m_radius
    return

  b2CircleShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = density * b2Settings.b2_pi * @m_radius * @m_radius
    massData.center.SetV @m_p
    massData.I = massData.mass * (0.5 * @m_radius * @m_radius + (@m_p.x * @m_p.x + @m_p.y * @m_p.y))
    return

  b2CircleShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    p = b2Math.MulX(xf, @m_p)
    l = (-(b2Math.Dot(normal, p) - offset))
    return 0  if l < (-@m_radius) + Number.MIN_VALUE
    if l > @m_radius
      c.SetV p
      return Math.PI * @m_radius * @m_radius
    r2 = @m_radius * @m_radius
    l2 = l * l
    area = r2 * (Math.asin(l / @m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2)
    com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area)
    c.x = p.x + normal.x * com
    c.y = p.y + normal.y * com
    area

  b2CircleShape::GetLocalPosition = ->
    @m_p

  b2CircleShape::SetLocalPosition = (position) ->
    @m_p.SetV position
    return

  b2CircleShape::GetRadius = ->
    @m_radius

  b2CircleShape::SetRadius = (radius) ->
    radius = 0  if radius is `undefined`
    @m_radius = radius
    return

  b2CircleShape::b2CircleShape = (radius) ->
    radius = 0  if radius is `undefined`
    @__super.b2Shape.call this
    @m_type = b2Shape.e_circleShape
    @m_radius = radius
    return

  b2EdgeChainDef.b2EdgeChainDef = ->

  b2EdgeChainDef::b2EdgeChainDef = ->
    @vertexCount = 0
    @isALoop = true
    @vertices = []
    return

  Box2D.inherit b2EdgeShape, Box2D.Collision.Shapes.b2Shape
  b2EdgeShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2EdgeShape.b2EdgeShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    @s_supportVec = new b2Vec2()
    @m_v1 = new b2Vec2()
    @m_v2 = new b2Vec2()
    @m_coreV1 = new b2Vec2()
    @m_coreV2 = new b2Vec2()
    @m_normal = new b2Vec2()
    @m_direction = new b2Vec2()
    @m_cornerDir1 = new b2Vec2()
    @m_cornerDir2 = new b2Vec2()
    return

  b2EdgeShape::TestPoint = (transform, p) ->
    false

  b2EdgeShape::RayCast = (output, input, transform) ->
    tMat = undefined
    rX = input.p2.x - input.p1.x
    rY = input.p2.y - input.p1.y
    tMat = transform.R
    v1X = transform.position.x + (tMat.col1.x * @m_v1.x + tMat.col2.x * @m_v1.y)
    v1Y = transform.position.y + (tMat.col1.y * @m_v1.x + tMat.col2.y * @m_v1.y)
    nX = transform.position.y + (tMat.col1.y * @m_v2.x + tMat.col2.y * @m_v2.y) - v1Y
    nY = (-(transform.position.x + (tMat.col1.x * @m_v2.x + tMat.col2.x * @m_v2.y) - v1X))
    k_slop = 100.0 * Number.MIN_VALUE
    denom = (-(rX * nX + rY * nY))
    if denom > k_slop
      bX = input.p1.x - v1X
      bY = input.p1.y - v1Y
      a = (bX * nX + bY * nY)
      if 0.0 <= a and a <= input.maxFraction * denom
        mu2 = (-rX * bY) + rY * bX
        if (-k_slop * denom) <= mu2 and mu2 <= denom * (1.0 + k_slop)
          a /= denom
          output.fraction = a
          nLen = Math.sqrt(nX * nX + nY * nY)
          output.normal.x = nX / nLen
          output.normal.y = nY / nLen
          return true
    false

  b2EdgeShape::ComputeAABB = (aabb, transform) ->
    tMat = transform.R
    v1X = transform.position.x + (tMat.col1.x * @m_v1.x + tMat.col2.x * @m_v1.y)
    v1Y = transform.position.y + (tMat.col1.y * @m_v1.x + tMat.col2.y * @m_v1.y)
    v2X = transform.position.x + (tMat.col1.x * @m_v2.x + tMat.col2.x * @m_v2.y)
    v2Y = transform.position.y + (tMat.col1.y * @m_v2.x + tMat.col2.y * @m_v2.y)
    if v1X < v2X
      aabb.lowerBound.x = v1X
      aabb.upperBound.x = v2X
    else
      aabb.lowerBound.x = v2X
      aabb.upperBound.x = v1X
    if v1Y < v2Y
      aabb.lowerBound.y = v1Y
      aabb.upperBound.y = v2Y
    else
      aabb.lowerBound.y = v2Y
      aabb.upperBound.y = v1Y
    return

  b2EdgeShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    massData.mass = 0
    massData.center.SetV @m_v1
    massData.I = 0
    return

  b2EdgeShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    v0 = new b2Vec2(normal.x * offset, normal.y * offset)
    v1 = b2Math.MulX(xf, @m_v1)
    v2 = b2Math.MulX(xf, @m_v2)
    d1 = b2Math.Dot(normal, v1) - offset
    d2 = b2Math.Dot(normal, v2) - offset
    if d1 > 0
      if d2 > 0
        return 0
      else
        v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x
        v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y
    else
      if d2 > 0
        v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x
        v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y
      else
    c.x = (v0.x + v1.x + v2.x) / 3
    c.y = (v0.y + v1.y + v2.y) / 3
    0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x))

  b2EdgeShape::GetLength = ->
    @m_length

  b2EdgeShape::GetVertex1 = ->
    @m_v1

  b2EdgeShape::GetVertex2 = ->
    @m_v2

  b2EdgeShape::GetCoreVertex1 = ->
    @m_coreV1

  b2EdgeShape::GetCoreVertex2 = ->
    @m_coreV2

  b2EdgeShape::GetNormalVector = ->
    @m_normal

  b2EdgeShape::GetDirectionVector = ->
    @m_direction

  b2EdgeShape::GetCorner1Vector = ->
    @m_cornerDir1

  b2EdgeShape::GetCorner2Vector = ->
    @m_cornerDir2

  b2EdgeShape::Corner1IsConvex = ->
    @m_cornerConvex1

  b2EdgeShape::Corner2IsConvex = ->
    @m_cornerConvex2

  b2EdgeShape::GetFirstVertex = (xf) ->
    tMat = xf.R
    new b2Vec2(xf.position.x + (tMat.col1.x * @m_coreV1.x + tMat.col2.x * @m_coreV1.y), xf.position.y + (tMat.col1.y * @m_coreV1.x + tMat.col2.y * @m_coreV1.y))

  b2EdgeShape::GetNextEdge = ->
    @m_nextEdge

  b2EdgeShape::GetPrevEdge = ->
    @m_prevEdge

  b2EdgeShape::Support = (xf, dX, dY) ->
    dX = 0  if dX is `undefined`
    dY = 0  if dY is `undefined`
    tMat = xf.R
    v1X = xf.position.x + (tMat.col1.x * @m_coreV1.x + tMat.col2.x * @m_coreV1.y)
    v1Y = xf.position.y + (tMat.col1.y * @m_coreV1.x + tMat.col2.y * @m_coreV1.y)
    v2X = xf.position.x + (tMat.col1.x * @m_coreV2.x + tMat.col2.x * @m_coreV2.y)
    v2Y = xf.position.y + (tMat.col1.y * @m_coreV2.x + tMat.col2.y * @m_coreV2.y)
    if (v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)
      @s_supportVec.x = v1X
      @s_supportVec.y = v1Y
    else
      @s_supportVec.x = v2X
      @s_supportVec.y = v2Y
    @s_supportVec

  b2EdgeShape::b2EdgeShape = (v1, v2) ->
    @__super.b2Shape.call this
    @m_type = b2Shape.e_edgeShape
    @m_prevEdge = null
    @m_nextEdge = null
    @m_v1 = v1
    @m_v2 = v2
    @m_direction.Set @m_v2.x - @m_v1.x, @m_v2.y - @m_v1.y
    @m_length = @m_direction.Normalize()
    @m_normal.Set @m_direction.y, (-@m_direction.x)
    @m_coreV1.Set (-b2Settings.b2_toiSlop * (@m_normal.x - @m_direction.x)) + @m_v1.x, (-b2Settings.b2_toiSlop * (@m_normal.y - @m_direction.y)) + @m_v1.y
    @m_coreV2.Set (-b2Settings.b2_toiSlop * (@m_normal.x + @m_direction.x)) + @m_v2.x, (-b2Settings.b2_toiSlop * (@m_normal.y + @m_direction.y)) + @m_v2.y
    @m_cornerDir1 = @m_normal
    @m_cornerDir2.Set (-@m_normal.x), (-@m_normal.y)
    return

  b2EdgeShape::SetPrevEdge = (edge, core, cornerDir, convex) ->
    @m_prevEdge = edge
    @m_coreV1 = core
    @m_cornerDir1 = cornerDir
    @m_cornerConvex1 = convex
    return

  b2EdgeShape::SetNextEdge = (edge, core, cornerDir, convex) ->
    @m_nextEdge = edge
    @m_coreV2 = core
    @m_cornerDir2 = cornerDir
    @m_cornerConvex2 = convex
    return

  b2MassData.b2MassData = ->
    @mass = 0.0
    @center = new b2Vec2(0, 0)
    @I = 0.0
    return

  Box2D.inherit b2PolygonShape, Box2D.Collision.Shapes.b2Shape
  b2PolygonShape::__super = Box2D.Collision.Shapes.b2Shape::
  b2PolygonShape.b2PolygonShape = ->
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply this, arguments
    return

  b2PolygonShape::Copy = ->
    s = new b2PolygonShape()
    s.Set this
    s

  b2PolygonShape::Set = (other) ->
    @__super.Set.call this, other
    if Box2D.equal(other, b2PolygonShape)
      other2 = ((if other instanceof b2PolygonShape then other else null))
      @m_centroid.SetV other2.m_centroid
      @m_vertexCount = other2.m_vertexCount
      @Reserve @m_vertexCount
      i = 0

      while i < @m_vertexCount
        @m_vertices[i].SetV other2.m_vertices[i]
        @m_normals[i].SetV other2.m_normals[i]
        i++
    return

  b2PolygonShape::SetAsArray = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    v = new Vector()
    i = 0
    tVec = undefined
    i = 0
    while i < vertices.length
      tVec = vertices[i]
      v.push tVec
      ++i
    @SetAsVector v, vertexCount
    return

  b2PolygonShape.AsArray = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsArray vertices, vertexCount
    polygonShape

  b2PolygonShape::SetAsVector = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    vertexCount = vertices.length  if vertexCount is 0
    b2Settings.b2Assert 2 <= vertexCount
    @m_vertexCount = vertexCount
    @Reserve vertexCount
    i = 0
    i = 0
    while i < @m_vertexCount
      @m_vertices[i].SetV vertices[i]
      i++
    i = 0
    while i < @m_vertexCount
      i1 = parseInt(i)
      i2 = parseInt((if i + 1 < @m_vertexCount then i + 1 else 0))
      edge = b2Math.SubtractVV(@m_vertices[i2], @m_vertices[i1])
      b2Settings.b2Assert edge.LengthSquared() > Number.MIN_VALUE
      @m_normals[i].SetV b2Math.CrossVF(edge, 1.0)
      @m_normals[i].Normalize()
      ++i
    @m_centroid = b2PolygonShape.ComputeCentroid(@m_vertices, @m_vertexCount)
    return

  b2PolygonShape.AsVector = (vertices, vertexCount) ->
    vertexCount = 0  if vertexCount is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsVector vertices, vertexCount
    polygonShape

  b2PolygonShape::SetAsBox = (hx, hy) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    @m_vertexCount = 4
    @Reserve 4
    @m_vertices[0].Set (-hx), (-hy)
    @m_vertices[1].Set hx, (-hy)
    @m_vertices[2].Set hx, hy
    @m_vertices[3].Set (-hx), hy
    @m_normals[0].Set 0.0, (-1.0)
    @m_normals[1].Set 1.0, 0.0
    @m_normals[2].Set 0.0, 1.0
    @m_normals[3].Set (-1.0), 0.0
    @m_centroid.SetZero()
    return

  b2PolygonShape.AsBox = (hx, hy) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsBox hx, hy
    polygonShape

  b2PolygonShape::SetAsOrientedBox = (hx, hy, center, angle) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    center = null  if center is `undefined`
    angle = 0.0  if angle is `undefined`
    @m_vertexCount = 4
    @Reserve 4
    @m_vertices[0].Set (-hx), (-hy)
    @m_vertices[1].Set hx, (-hy)
    @m_vertices[2].Set hx, hy
    @m_vertices[3].Set (-hx), hy
    @m_normals[0].Set 0.0, (-1.0)
    @m_normals[1].Set 1.0, 0.0
    @m_normals[2].Set 0.0, 1.0
    @m_normals[3].Set (-1.0), 0.0
    @m_centroid = center
    xf = new b2Transform()
    xf.position = center
    xf.R.Set angle
    i = 0

    while i < @m_vertexCount
      @m_vertices[i] = b2Math.MulX(xf, @m_vertices[i])
      @m_normals[i] = b2Math.MulMV(xf.R, @m_normals[i])
      ++i
    return

  b2PolygonShape.AsOrientedBox = (hx, hy, center, angle) ->
    hx = 0  if hx is `undefined`
    hy = 0  if hy is `undefined`
    center = null  if center is `undefined`
    angle = 0.0  if angle is `undefined`
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsOrientedBox hx, hy, center, angle
    polygonShape

  b2PolygonShape::SetAsEdge = (v1, v2) ->
    @m_vertexCount = 2
    @Reserve 2
    @m_vertices[0].SetV v1
    @m_vertices[1].SetV v2
    @m_centroid.x = 0.5 * (v1.x + v2.x)
    @m_centroid.y = 0.5 * (v1.y + v2.y)
    @m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0)
    @m_normals[0].Normalize()
    @m_normals[1].x = (-@m_normals[0].x)
    @m_normals[1].y = (-@m_normals[0].y)
    return

  b2PolygonShape.AsEdge = (v1, v2) ->
    polygonShape = new b2PolygonShape()
    polygonShape.SetAsEdge v1, v2
    polygonShape

  b2PolygonShape::TestPoint = (xf, p) ->
    tVec = undefined
    tMat = xf.R
    tX = p.x - xf.position.x
    tY = p.y - xf.position.y
    pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y)
    pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y)
    i = 0

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      tX = pLocalX - tVec.x
      tY = pLocalY - tVec.y
      tVec = @m_normals[i]
      dot = (tVec.x * tX + tVec.y * tY)
      return false  if dot > 0.0
      ++i
    true

  b2PolygonShape::RayCast = (output, input, transform) ->
    lower = 0.0
    upper = input.maxFraction
    tX = 0
    tY = 0
    tMat = undefined
    tVec = undefined
    tX = input.p1.x - transform.position.x
    tY = input.p1.y - transform.position.y
    tMat = transform.R
    p1X = (tX * tMat.col1.x + tY * tMat.col1.y)
    p1Y = (tX * tMat.col2.x + tY * tMat.col2.y)
    tX = input.p2.x - transform.position.x
    tY = input.p2.y - transform.position.y
    tMat = transform.R
    p2X = (tX * tMat.col1.x + tY * tMat.col1.y)
    p2Y = (tX * tMat.col2.x + tY * tMat.col2.y)
    dX = p2X - p1X
    dY = p2Y - p1Y
    index = parseInt((-1))
    i = 0

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      tX = tVec.x - p1X
      tY = tVec.y - p1Y
      tVec = @m_normals[i]
      numerator = (tVec.x * tX + tVec.y * tY)
      denominator = (tVec.x * dX + tVec.y * dY)
      if denominator is 0.0
        return false  if numerator < 0.0
      else
        if denominator < 0.0 and numerator < lower * denominator
          lower = numerator / denominator
          index = i
        else upper = numerator / denominator  if denominator > 0.0 and numerator < upper * denominator
      return false  if upper < lower - Number.MIN_VALUE
      ++i
    if index >= 0
      output.fraction = lower
      tMat = transform.R
      tVec = @m_normals[index]
      output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      return true
    false

  b2PolygonShape::ComputeAABB = (aabb, xf) ->
    tMat = xf.R
    tVec = @m_vertices[0]
    lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    upperX = lowerX
    upperY = lowerY
    i = 1

    while i < @m_vertexCount
      tVec = @m_vertices[i]
      vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
      vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
      lowerX = (if lowerX < vX then lowerX else vX)
      lowerY = (if lowerY < vY then lowerY else vY)
      upperX = (if upperX > vX then upperX else vX)
      upperY = (if upperY > vY then upperY else vY)
      ++i
    aabb.lowerBound.x = lowerX - @m_radius
    aabb.lowerBound.y = lowerY - @m_radius
    aabb.upperBound.x = upperX + @m_radius
    aabb.upperBound.y = upperY + @m_radius
    return

  b2PolygonShape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    if @m_vertexCount is 2
      massData.center.x = 0.5 * (@m_vertices[0].x + @m_vertices[1].x)
      massData.center.y = 0.5 * (@m_vertices[0].y + @m_vertices[1].y)
      massData.mass = 0.0
      massData.I = 0.0
      return
    centerX = 0.0
    centerY = 0.0
    area = 0.0
    I = 0.0
    p1X = 0.0
    p1Y = 0.0
    k_inv3 = 1.0 / 3.0
    i = 0

    while i < @m_vertexCount
      p2 = @m_vertices[i]
      p3 = (if i + 1 < @m_vertexCount then @m_vertices[parseInt(i + 1)] else @m_vertices[0])
      e1X = p2.x - p1X
      e1Y = p2.y - p1Y
      e2X = p3.x - p1X
      e2Y = p3.y - p1Y
      D = e1X * e2Y - e1Y * e2X
      triangleArea = 0.5 * D
      area += triangleArea
      centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x)
      centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y)
      px = p1X
      py = p1Y
      ex1 = e1X
      ey1 = e1Y
      ex2 = e2X
      ey2 = e2Y
      intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px
      inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py
      I += D * (intx2 + inty2)
      ++i
    massData.mass = density * area
    centerX *= 1.0 / area
    centerY *= 1.0 / area
    massData.center.Set centerX, centerY
    massData.I = density * I
    return

  b2PolygonShape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    normalL = b2Math.MulTMV(xf.R, normal)
    offsetL = offset - b2Math.Dot(normal, xf.position)
    depths = new Vector_a2j_Number()
    diveCount = 0
    intoIndex = parseInt((-1))
    outoIndex = parseInt((-1))
    lastSubmerged = false
    i = 0
    i = 0
    while i < @m_vertexCount
      depths[i] = b2Math.Dot(normalL, @m_vertices[i]) - offsetL
      isSubmerged = depths[i] < (-Number.MIN_VALUE)
      if i > 0
        if isSubmerged
          unless lastSubmerged
            intoIndex = i - 1
            diveCount++
        else
          if lastSubmerged
            outoIndex = i - 1
            diveCount++
      lastSubmerged = isSubmerged
      ++i
    switch diveCount
      when 0
        if lastSubmerged
          md = new b2MassData()
          @ComputeMass md, 1
          c.SetV b2Math.MulX(xf, md.center)
          return md.mass
        else
          return 0
      when 1
        if intoIndex is (-1)
          intoIndex = @m_vertexCount - 1
        else
          outoIndex = @m_vertexCount - 1
    intoIndex2 = parseInt((intoIndex + 1) % @m_vertexCount)
    outoIndex2 = parseInt((outoIndex + 1) % @m_vertexCount)
    intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex])
    outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex])
    intoVec = new b2Vec2(@m_vertices[intoIndex].x * (1 - intoLamdda) + @m_vertices[intoIndex2].x * intoLamdda, @m_vertices[intoIndex].y * (1 - intoLamdda) + @m_vertices[intoIndex2].y * intoLamdda)
    outoVec = new b2Vec2(@m_vertices[outoIndex].x * (1 - outoLamdda) + @m_vertices[outoIndex2].x * outoLamdda, @m_vertices[outoIndex].y * (1 - outoLamdda) + @m_vertices[outoIndex2].y * outoLamdda)
    area = 0
    center = new b2Vec2()
    p2 = @m_vertices[intoIndex2]
    p3 = undefined
    i = intoIndex2
    until i is outoIndex2
      i = (i + 1) % @m_vertexCount
      if i is outoIndex2
        p3 = outoVec
      else
        p3 = @m_vertices[i]
      triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x))
      area += triangleArea
      center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3
      center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3
      p2 = p3
    center.Multiply 1 / area
    c.SetV b2Math.MulX(xf, center)
    area

  b2PolygonShape::GetVertexCount = ->
    @m_vertexCount

  b2PolygonShape::GetVertices = ->
    @m_vertices

  b2PolygonShape::GetNormals = ->
    @m_normals

  b2PolygonShape::GetSupport = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_vertexCount
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    bestIndex

  b2PolygonShape::GetSupportVertex = (d) ->
    bestIndex = 0
    bestValue = @m_vertices[0].x * d.x + @m_vertices[0].y * d.y
    i = 1

    while i < @m_vertexCount
      value = @m_vertices[i].x * d.x + @m_vertices[i].y * d.y
      if value > bestValue
        bestIndex = i
        bestValue = value
      ++i
    @m_vertices[bestIndex]

  b2PolygonShape::Validate = ->
    false

  b2PolygonShape::b2PolygonShape = ->
    @__super.b2Shape.call this
    @m_type = b2Shape.e_polygonShape
    @m_centroid = new b2Vec2()
    @m_vertices = new Vector()
    @m_normals = new Vector()
    return

  b2PolygonShape::Reserve = (count) ->
    count = 0  if count is `undefined`
    i = parseInt(@m_vertices.length)

    while i < count
      @m_vertices[i] = new b2Vec2()
      @m_normals[i] = new b2Vec2()
      i++
    return

  b2PolygonShape.ComputeCentroid = (vs, count) ->
    count = 0  if count is `undefined`
    c = new b2Vec2()
    area = 0.0
    p1X = 0.0
    p1Y = 0.0
    inv3 = 1.0 / 3.0
    i = 0

    while i < count
      p2 = vs[i]
      p3 = (if i + 1 < count then vs[parseInt(i + 1)] else vs[0])
      e1X = p2.x - p1X
      e1Y = p2.y - p1Y
      e2X = p3.x - p1X
      e2Y = p3.y - p1Y
      D = (e1X * e2Y - e1Y * e2X)
      triangleArea = 0.5 * D
      area += triangleArea
      c.x += triangleArea * inv3 * (p1X + p2.x + p3.x)
      c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y)
      ++i
    c.x *= 1.0 / area
    c.y *= 1.0 / area
    c

  b2PolygonShape.ComputeOBB = (obb, vs, count) ->
    count = 0  if count is `undefined`
    i = 0
    p = new Vector(count + 1)
    i = 0
    while i < count
      p[i] = vs[i]
      ++i
    p[count] = p[0]
    minArea = Number.MAX_VALUE
    i = 1
    while i <= count
      root = p[parseInt(i - 1)]
      uxX = p[i].x - root.x
      uxY = p[i].y - root.y
      length = Math.sqrt(uxX * uxX + uxY * uxY)
      uxX /= length
      uxY /= length
      uyX = (-uxY)
      uyY = uxX
      lowerX = Number.MAX_VALUE
      lowerY = Number.MAX_VALUE
      upperX = (-Number.MAX_VALUE)
      upperY = (-Number.MAX_VALUE)
      j = 0

      while j < count
        dX = p[j].x - root.x
        dY = p[j].y - root.y
        rX = (uxX * dX + uxY * dY)
        rY = (uyX * dX + uyY * dY)
        lowerX = rX  if rX < lowerX
        lowerY = rY  if rY < lowerY
        upperX = rX  if rX > upperX
        upperY = rY  if rY > upperY
        ++j
      area = (upperX - lowerX) * (upperY - lowerY)
      if area < 0.95 * minArea
        minArea = area
        obb.R.col1.x = uxX
        obb.R.col1.y = uxY
        obb.R.col2.x = uyX
        obb.R.col2.y = uyY
        centerX = 0.5 * (lowerX + upperX)
        centerY = 0.5 * (lowerY + upperY)
        tMat = obb.R
        obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY)
        obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY)
        obb.extents.x = 0.5 * (upperX - lowerX)
        obb.extents.y = 0.5 * (upperY - lowerY)
      ++i
    return

  Box2D.postDefs.push ->
    Box2D.Collision.Shapes.b2PolygonShape.s_mat = new b2Mat22()
    return

  b2Shape.b2Shape = ->

  b2Shape::Copy = ->
    null

  b2Shape::Set = (other) ->
    @m_radius = other.m_radius
    return

  b2Shape::GetType = ->
    @m_type

  b2Shape::TestPoint = (xf, p) ->
    false

  b2Shape::RayCast = (output, input, transform) ->
    false

  b2Shape::ComputeAABB = (aabb, xf) ->

  b2Shape::ComputeMass = (massData, density) ->
    density = 0  if density is `undefined`
    return

  b2Shape::ComputeSubmergedArea = (normal, offset, xf, c) ->
    offset = 0  if offset is `undefined`
    0

  b2Shape.TestOverlap = (shape1, transform1, shape2, transform2) ->
    input = new b2DistanceInput()
    input.proxyA = new b2DistanceProxy()
    input.proxyA.Set shape1
    input.proxyB = new b2DistanceProxy()
    input.proxyB.Set shape2
    input.transformA = transform1
    input.transformB = transform2
    input.useRadii = true
    simplexCache = new b2SimplexCache()
    simplexCache.count = 0
    output = new b2DistanceOutput()
    b2Distance.Distance output, simplexCache, input
    output.distance < 10.0 * Number.MIN_VALUE

  b2Shape::b2Shape = ->
    @m_type = b2Shape.e_unknownShape
    @m_radius = b2Settings.b2_linearSlop
    return

  Box2D.postDefs.push ->
    Box2D.Collision.Shapes.b2Shape.e_unknownShape = parseInt((-1))
    Box2D.Collision.Shapes.b2Shape.e_circleShape = 0
    Box2D.Collision.Shapes.b2Shape.e_polygonShape = 1
    Box2D.Collision.Shapes.b2Shape.e_edgeShape = 2
    Box2D.Collision.Shapes.b2Shape.e_shapeTypeCount = 3
    Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1
    Box2D.Collision.Shapes.b2Shape.e_missCollide = 0
    Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = parseInt((-1))
    return

  return
)()
(->
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
  b2Color.b2Color = ->
    @_r = 0
    @_g = 0
    @_b = 0
    return

  b2Color::b2Color = (rr, gg, bb) ->
    rr = 0  if rr is `undefined`
    gg = 0  if gg is `undefined`
    bb = 0  if bb is `undefined`
    @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
    return

  b2Color::Set = (rr, gg, bb) ->
    rr = 0  if rr is `undefined`
    gg = 0  if gg is `undefined`
    bb = 0  if bb is `undefined`
    @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
    @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
    @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
    return

  Object.defineProperty b2Color::, "r",
    enumerable: false
    configurable: true
    set: (rr) ->
      rr = 0  if rr is `undefined`
      @_r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "g",
    enumerable: false
    configurable: true
    set: (gg) ->
      gg = 0  if gg is `undefined`
      @_g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "b",
    enumerable: false
    configurable: true
    set: (bb) ->
      bb = 0  if bb is `undefined`
      @_b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0))
      return

  Object.defineProperty b2Color::, "color",
    enumerable: false
    configurable: true
    get: ->
      (@_r << 16) | (@_g << 8) | (@_b)

  b2Settings.b2Settings = ->

  b2Settings.b2MixFriction = (friction1, friction2) ->
    friction1 = 0  if friction1 is `undefined`
    friction2 = 0  if friction2 is `undefined`
    Math.sqrt friction1 * friction2

  b2Settings.b2MixRestitution = (restitution1, restitution2) ->
    restitution1 = 0  if restitution1 is `undefined`
    restitution2 = 0  if restitution2 is `undefined`
    (if restitution1 > restitution2 then restitution1 else restitution2)

  b2Settings.b2Assert = (a) ->
    throw "Assertion Failed"  unless a
    return

  Box2D.postDefs.push ->
    Box2D.Common.b2Settings.VERSION = "2.1alpha"
    Box2D.Common.b2Settings.USHRT_MAX = 0x0000ffff
    Box2D.Common.b2Settings.b2_pi = Math.PI
    Box2D.Common.b2Settings.b2_maxManifoldPoints = 2
    Box2D.Common.b2Settings.b2_aabbExtension = 0.1
    Box2D.Common.b2Settings.b2_aabbMultiplier = 2.0
    Box2D.Common.b2Settings.b2_polygonRadius = 2.0 * b2Settings.b2_linearSlop
    Box2D.Common.b2Settings.b2_linearSlop = 0.005
    Box2D.Common.b2Settings.b2_angularSlop = 2.0 / 180.0 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_toiSlop = 8.0 * b2Settings.b2_linearSlop
    Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland = 32
    Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland = 32
    Box2D.Common.b2Settings.b2_velocityThreshold = 1.0
    Box2D.Common.b2Settings.b2_maxLinearCorrection = 0.2
    Box2D.Common.b2Settings.b2_maxAngularCorrection = 8.0 / 180.0 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_maxTranslation = 2.0
    Box2D.Common.b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation
    Box2D.Common.b2Settings.b2_maxRotation = 0.5 * b2Settings.b2_pi
    Box2D.Common.b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation
    Box2D.Common.b2Settings.b2_contactBaumgarte = 0.2
    Box2D.Common.b2Settings.b2_timeToSleep = 0.5
    Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.01
    Box2D.Common.b2Settings.b2_angularSleepTolerance = 2.0 / 180.0 * b2Settings.b2_pi
    return

  return
)()
(->
  b2AABB = Box2D.Collision.b2AABB
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
  b2Mat22.b2Mat22 = ->
    @col1 = new b2Vec2()
    @col2 = new b2Vec2()
    return

  b2Mat22::b2Mat22 = ->
    @SetIdentity()
    return

  b2Mat22.FromAngle = (angle) ->
    angle = 0  if angle is `undefined`
    mat = new b2Mat22()
    mat.Set angle
    mat

  b2Mat22.FromVV = (c1, c2) ->
    mat = new b2Mat22()
    mat.SetVV c1, c2
    mat

  b2Mat22::Set = (angle) ->
    angle = 0  if angle is `undefined`
    c = Math.cos(angle)
    s = Math.sin(angle)
    @col1.x = c
    @col2.x = (-s)
    @col1.y = s
    @col2.y = c
    return

  b2Mat22::SetVV = (c1, c2) ->
    @col1.SetV c1
    @col2.SetV c2
    return

  b2Mat22::Copy = ->
    mat = new b2Mat22()
    mat.SetM this
    mat

  b2Mat22::SetM = (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    return

  b2Mat22::AddM = (m) ->
    @col1.x += m.col1.x
    @col1.y += m.col1.y
    @col2.x += m.col2.x
    @col2.y += m.col2.y
    return

  b2Mat22::SetIdentity = ->
    @col1.x = 1.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 1.0
    return

  b2Mat22::SetZero = ->
    @col1.x = 0.0
    @col2.x = 0.0
    @col1.y = 0.0
    @col2.y = 0.0
    return

  b2Mat22::GetAngle = ->
    Math.atan2 @col1.y, @col1.x

  b2Mat22::GetInverse = (out) ->
    a = @col1.x
    b = @col2.x
    c = @col1.y
    d = @col2.y
    det = a * d - b * c
    det = 1.0 / det  unless det is 0.0
    out.col1.x = det * d
    out.col2.x = (-det * b)
    out.col1.y = (-det * c)
    out.col2.y = det * a
    out

  b2Mat22::Solve = (out, bX, bY) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  unless det is 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    out

  b2Mat22::Abs = ->
    @col1.Abs()
    @col2.Abs()
    return

  b2Mat33.b2Mat33 = ->
    @col1 = new b2Vec3()
    @col2 = new b2Vec3()
    @col3 = new b2Vec3()
    return

  b2Mat33::b2Mat33 = (c1, c2, c3) ->
    c1 = null  if c1 is `undefined`
    c2 = null  if c2 is `undefined`
    c3 = null  if c3 is `undefined`
    if not c1 and not c2 and not c3
      @col1.SetZero()
      @col2.SetZero()
      @col3.SetZero()
    else
      @col1.SetV c1
      @col2.SetV c2
      @col3.SetV c3
    return

  b2Mat33::SetVVV = (c1, c2, c3) ->
    @col1.SetV c1
    @col2.SetV c2
    @col3.SetV c3
    return

  b2Mat33::Copy = ->
    new b2Mat33(@col1, @col2, @col3)

  b2Mat33::SetM = (m) ->
    @col1.SetV m.col1
    @col2.SetV m.col2
    @col3.SetV m.col3
    return

  b2Mat33::AddM = (m) ->
    @col1.x += m.col1.x
    @col1.y += m.col1.y
    @col1.z += m.col1.z
    @col2.x += m.col2.x
    @col2.y += m.col2.y
    @col2.z += m.col2.z
    @col3.x += m.col3.x
    @col3.y += m.col3.y
    @col3.z += m.col3.z
    return

  b2Mat33::SetIdentity = ->
    @col1.x = 1.0
    @col2.x = 0.0
    @col3.x = 0.0
    @col1.y = 0.0
    @col2.y = 1.0
    @col3.y = 0.0
    @col1.z = 0.0
    @col2.z = 0.0
    @col3.z = 1.0
    return

  b2Mat33::SetZero = ->
    @col1.x = 0.0
    @col2.x = 0.0
    @col3.x = 0.0
    @col1.y = 0.0
    @col2.y = 0.0
    @col3.y = 0.0
    @col1.z = 0.0
    @col2.z = 0.0
    @col3.z = 0.0
    return

  b2Mat33::Solve22 = (out, bX, bY) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    a11 = @col1.x
    a12 = @col2.x
    a21 = @col1.y
    a22 = @col2.y
    det = a11 * a22 - a12 * a21
    det = 1.0 / det  unless det is 0.0
    out.x = det * (a22 * bX - a12 * bY)
    out.y = det * (a11 * bY - a21 * bX)
    out

  b2Mat33::Solve33 = (out, bX, bY, bZ) ->
    bX = 0  if bX is `undefined`
    bY = 0  if bY is `undefined`
    bZ = 0  if bZ is `undefined`
    a11 = @col1.x
    a21 = @col1.y
    a31 = @col1.z
    a12 = @col2.x
    a22 = @col2.y
    a32 = @col2.z
    a13 = @col3.x
    a23 = @col3.y
    a33 = @col3.z
    det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13)
    det = 1.0 / det  unless det is 0.0
    out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13))
    out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13))
    out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX))
    out

  b2Math.b2Math = ->

  b2Math.IsValid = (x) ->
    x = 0  if x is `undefined`
    isFinite x

  b2Math.Dot = (a, b) ->
    a.x * b.x + a.y * b.y

  b2Math.CrossVV = (a, b) ->
    a.x * b.y - a.y * b.x

  b2Math.CrossVF = (a, s) ->
    s = 0  if s is `undefined`
    v = new b2Vec2(s * a.y, (-s * a.x))
    v

  b2Math.CrossFV = (s, a) ->
    s = 0  if s is `undefined`
    v = new b2Vec2((-s * a.y), s * a.x)
    v

  b2Math.MulMV = (A, v) ->
    u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y)
    u

  b2Math.MulTMV = (A, v) ->
    u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2))
    u

  b2Math.MulX = (T, v) ->
    a = b2Math.MulMV(T.R, v)
    a.x += T.position.x
    a.y += T.position.y
    a

  b2Math.MulXT = (T, v) ->
    a = b2Math.SubtractVV(v, T.position)
    tX = (a.x * T.R.col1.x + a.y * T.R.col1.y)
    a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y)
    a.x = tX
    a

  b2Math.AddVV = (a, b) ->
    v = new b2Vec2(a.x + b.x, a.y + b.y)
    v

  b2Math.SubtractVV = (a, b) ->
    v = new b2Vec2(a.x - b.x, a.y - b.y)
    v

  b2Math.Distance = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    Math.sqrt cX * cX + cY * cY

  b2Math.DistanceSquared = (a, b) ->
    cX = a.x - b.x
    cY = a.y - b.y
    cX * cX + cY * cY

  b2Math.MulFV = (s, a) ->
    s = 0  if s is `undefined`
    v = new b2Vec2(s * a.x, s * a.y)
    v

  b2Math.AddMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2))
    C

  b2Math.MulMM = (A, B) ->
    C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2))
    C

  b2Math.MulTMM = (A, B) ->
    c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1))
    c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2))
    C = b2Mat22.FromVV(c1, c2)
    C

  b2Math.Abs = (a) ->
    a = 0  if a is `undefined`
    (if a > 0.0 then a else (-a))

  b2Math.AbsV = (a) ->
    b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y))
    b

  b2Math.AbsM = (A) ->
    B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2))
    B

  b2Math.Min = (a, b) ->
    a = 0  if a is `undefined`
    b = 0  if b is `undefined`
    (if a < b then a else b)

  b2Math.MinV = (a, b) ->
    c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y))
    c

  b2Math.Max = (a, b) ->
    a = 0  if a is `undefined`
    b = 0  if b is `undefined`
    (if a > b then a else b)

  b2Math.MaxV = (a, b) ->
    c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y))
    c

  b2Math.Clamp = (a, low, high) ->
    a = 0  if a is `undefined`
    low = 0  if low is `undefined`
    high = 0  if high is `undefined`
    (if a < low then low else (if a > high then high else a))

  b2Math.ClampV = (a, low, high) ->
    b2Math.MaxV low, b2Math.MinV(a, high)

  b2Math.Swap = (a, b) ->
    tmp = a[0]
    a[0] = b[0]
    b[0] = tmp
    return

  b2Math.Random = ->
    Math.random() * 2 - 1

  b2Math.RandomRange = (lo, hi) ->
    lo = 0  if lo is `undefined`
    hi = 0  if hi is `undefined`
    r = Math.random()
    r = (hi - lo) * r + lo
    r

  b2Math.NextPowerOfTwo = (x) ->
    x = 0  if x is `undefined`
    x |= (x >> 1) & 0x7FFFFFFF
    x |= (x >> 2) & 0x3FFFFFFF
    x |= (x >> 4) & 0x0FFFFFFF
    x |= (x >> 8) & 0x00FFFFFF
    x |= (x >> 16) & 0x0000FFFF
    x + 1

  b2Math.IsPowerOfTwo = (x) ->
    x = 0  if x is `undefined`
    result = x > 0 and (x & (x - 1)) is 0
    result

  Box2D.postDefs.push ->
    Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0)
    Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0))
    Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity)
    return

  b2Sweep.b2Sweep = ->
    @localCenter = new b2Vec2()
    @c0 = new b2Vec2
    @c = new b2Vec2()
    return

  b2Sweep::Set = (other) ->
    @localCenter.SetV other.localCenter
    @c0.SetV other.c0
    @c.SetV other.c
    @a0 = other.a0
    @a = other.a
    @t0 = other.t0
    return

  b2Sweep::Copy = ->
    copy = new b2Sweep()
    copy.localCenter.SetV @localCenter
    copy.c0.SetV @c0
    copy.c.SetV @c
    copy.a0 = @a0
    copy.a = @a
    copy.t0 = @t0
    copy

  b2Sweep::GetTransform = (xf, alpha) ->
    alpha = 0  if alpha is `undefined`
    xf.position.x = (1.0 - alpha) * @c0.x + alpha * @c.x
    xf.position.y = (1.0 - alpha) * @c0.y + alpha * @c.y
    angle = (1.0 - alpha) * @a0 + alpha * @a
    xf.R.Set angle
    tMat = xf.R
    xf.position.x -= (tMat.col1.x * @localCenter.x + tMat.col2.x * @localCenter.y)
    xf.position.y -= (tMat.col1.y * @localCenter.x + tMat.col2.y * @localCenter.y)
    return

  b2Sweep::Advance = (t) ->
    t = 0  if t is `undefined`
    if @t0 < t and 1.0 - @t0 > Number.MIN_VALUE
      alpha = (t - @t0) / (1.0 - @t0)
      @c0.x = (1.0 - alpha) * @c0.x + alpha * @c.x
      @c0.y = (1.0 - alpha) * @c0.y + alpha * @c.y
      @a0 = (1.0 - alpha) * @a0 + alpha * @a
      @t0 = t
    return

  b2Transform.b2Transform = ->
    @position = new b2Vec2
    @R = new b2Mat22()
    return

  b2Transform::b2Transform = (pos, r) ->
    pos = null  if pos is `undefined`
    r = null  if r is `undefined`
    if pos
      @position.SetV pos
      @R.SetM r
    return

  b2Transform::Initialize = (pos, r) ->
    @position.SetV pos
    @R.SetM r
    return

  b2Transform::SetIdentity = ->
    @position.SetZero()
    @R.SetIdentity()
    return

  b2Transform::Set = (x) ->
    @position.SetV x.position
    @R.SetM x.R
    return

  b2Transform::GetAngle = ->
    Math.atan2 @R.col1.y, @R.col1.x

  b2Vec2.b2Vec2 = ->

  b2Vec2::b2Vec2 = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    @x = x_
    @y = y_
    return

  b2Vec2::SetZero = ->
    @x = 0.0
    @y = 0.0
    return

  b2Vec2::Set = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    @x = x_
    @y = y_
    return

  b2Vec2::SetV = (v) ->
    @x = v.x
    @y = v.y
    return

  b2Vec2::GetNegative = ->
    new b2Vec2((-@x), (-@y))

  b2Vec2::NegativeSelf = ->
    @x = (-@x)
    @y = (-@y)
    return

  b2Vec2.Make = (x_, y_) ->
    x_ = 0  if x_ is `undefined`
    y_ = 0  if y_ is `undefined`
    new b2Vec2(x_, y_)

  b2Vec2::Copy = ->
    new b2Vec2(@x, @y)

  b2Vec2::Add = (v) ->
    @x += v.x
    @y += v.y
    return

  b2Vec2::Subtract = (v) ->
    @x -= v.x
    @y -= v.y
    return

  b2Vec2::Multiply = (a) ->
    a = 0  if a is `undefined`
    @x *= a
    @y *= a
    return

  b2Vec2::MulM = (A) ->
    tX = @x
    @x = A.col1.x * tX + A.col2.x * @y
    @y = A.col1.y * tX + A.col2.y * @y
    return

  b2Vec2::MulTM = (A) ->
    tX = b2Math.Dot(this, A.col1)
    @y = b2Math.Dot(this, A.col2)
    @x = tX
    return

  b2Vec2::CrossVF = (s) ->
    s = 0  if s is `undefined`
    tX = @x
    @x = s * @y
    @y = (-s * tX)
    return

  b2Vec2::CrossFV = (s) ->
    s = 0  if s is `undefined`
    tX = @x
    @x = (-s * @y)
    @y = s * tX
    return

  b2Vec2::MinV = (b) ->
    @x = (if @x < b.x then @x else b.x)
    @y = (if @y < b.y then @y else b.y)
    return

  b2Vec2::MaxV = (b) ->
    @x = (if @x > b.x then @x else b.x)
    @y = (if @y > b.y then @y else b.y)
    return

  b2Vec2::Abs = ->
    @x = (-@x)  if @x < 0
    @y = (-@y)  if @y < 0
    return

  b2Vec2::Length = ->
    Math.sqrt @x * @x + @y * @y

  b2Vec2::LengthSquared = ->
    @x * @x + @y * @y

  b2Vec2::Normalize = ->
    length = Math.sqrt(@x * @x + @y * @y)
    return 0.0  if length < Number.MIN_VALUE
    invLength = 1.0 / length
    @x *= invLength
    @y *= invLength
    length

  b2Vec2::IsValid = ->
    b2Math.IsValid(@x) and b2Math.IsValid(@y)

  b2Vec3.b2Vec3 = ->

  b2Vec3::b2Vec3 = (x, y, z) ->
    x = 0  if x is `undefined`
    y = 0  if y is `undefined`
    z = 0  if z is `undefined`
    @x = x
    @y = y
    @z = z
    return

  b2Vec3::SetZero = ->
    @x = @y = @z = 0.0
    return

  b2Vec3::Set = (x, y, z) ->
    x = 0  if x is `undefined`
    y = 0  if y is `undefined`
    z = 0  if z is `undefined`
    @x = x
    @y = y
    @z = z
    return

  b2Vec3::SetV = (v) ->
    @x = v.x
    @y = v.y
    @z = v.z
    return

  b2Vec3::GetNegative = ->
    new b2Vec3((-@x), (-@y), (-@z))

  b2Vec3::NegativeSelf = ->
    @x = (-@x)
    @y = (-@y)
    @z = (-@z)
    return

  b2Vec3::Copy = ->
    new b2Vec3(@x, @y, @z)

  b2Vec3::Add = (v) ->
    @x += v.x
    @y += v.y
    @z += v.z
    return

  b2Vec3::Subtract = (v) ->
    @x -= v.x
    @y -= v.y
    @z -= v.z
    return

  b2Vec3::Multiply = (a) ->
    a = 0  if a is `undefined`
    @x *= a
    @y *= a
    @z *= a
    return

  return
)()
(->
  b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
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
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
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
  b2Controller = Box2D.Dynamics.Controllers.b2Controller
  b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint
  b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef
  b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint
  b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef
  b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint
  b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef
  b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian
  b2Joint = Box2D.Dynamics.Joints.b2Joint
  b2JointDef = Box2D.Dynamics.Joints.b2JointDef
  b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge
  b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint
  b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef
  b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint
  b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
  b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint
  b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef
  b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint
  b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef
  b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint
  b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
  b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint
  b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
  b2Body.b2Body = ->
    @m_xf = new b2Transform()
    @m_sweep = new b2Sweep()
    @m_linearVelocity = new b2Vec2()
    @m_force = new b2Vec2()
    return

  b2Body::connectEdges = (s1, s2, angle1) ->
    angle1 = 0  if angle1 is `undefined`
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
    angle2

  b2Body::CreateFixture = (def) ->
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
    @m_world.m_flags |= b2World.e_newFixture
    fixture

  b2Body::CreateFixture2 = (shape, density) ->
    density = 0.0  if density is `undefined`
    def = new b2FixtureDef()
    def.shape = shape
    def.density = density
    @CreateFixture def

  b2Body::DestroyFixture = (fixture) ->
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

  b2Body::SetPositionAndAngle = (position, angle) ->
    angle = 0  if angle is `undefined`
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

  b2Body::SetTransform = (xf) ->
    @SetPositionAndAngle xf.position, xf.GetAngle()
    return

  b2Body::GetTransform = ->
    @m_xf

  b2Body::GetPosition = ->
    @m_xf.position

  b2Body::SetPosition = (position) ->
    @SetPositionAndAngle position, @GetAngle()
    return

  b2Body::GetAngle = ->
    @m_sweep.a

  b2Body::SetAngle = (angle) ->
    angle = 0  if angle is `undefined`
    @SetPositionAndAngle @GetPosition(), angle
    return

  b2Body::GetWorldCenter = ->
    @m_sweep.c

  b2Body::GetLocalCenter = ->
    @m_sweep.localCenter

  b2Body::SetLinearVelocity = (v) ->
    return  if @m_type is b2Body.b2_staticBody
    @m_linearVelocity.SetV v
    return

  b2Body::GetLinearVelocity = ->
    @m_linearVelocity

  b2Body::SetAngularVelocity = (omega) ->
    omega = 0  if omega is `undefined`
    return  if @m_type is b2Body.b2_staticBody
    @m_angularVelocity = omega
    return

  b2Body::GetAngularVelocity = ->
    @m_angularVelocity

  b2Body::GetDefinition = ->
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
    bd

  b2Body::ApplyForce = (force, point) ->
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_force.x += force.x
    @m_force.y += force.y
    @m_torque += ((point.x - @m_sweep.c.x) * force.y - (point.y - @m_sweep.c.y) * force.x)
    return

  b2Body::ApplyTorque = (torque) ->
    torque = 0  if torque is `undefined`
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_torque += torque
    return

  b2Body::ApplyImpulse = (impulse, point) ->
    return  unless @m_type is b2Body.b2_dynamicBody
    @SetAwake true  if @IsAwake() is false
    @m_linearVelocity.x += @m_invMass * impulse.x
    @m_linearVelocity.y += @m_invMass * impulse.y
    @m_angularVelocity += @m_invI * ((point.x - @m_sweep.c.x) * impulse.y - (point.y - @m_sweep.c.y) * impulse.x)
    return

  b2Body::Split = (callback) ->
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
    body2

  b2Body::Merge = (other) ->
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

  b2Body::GetMass = ->
    @m_mass

  b2Body::GetInertia = ->
    @m_I

  b2Body::GetMassData = (data) ->
    data.mass = @m_mass
    data.I = @m_I
    data.center.SetV @m_sweep.localCenter
    return

  b2Body::SetMassData = (massData) ->
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

  b2Body::ResetMassData = ->
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

  b2Body::GetWorldPoint = (localPoint) ->
    A = @m_xf.R
    u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y)
    u.x += @m_xf.position.x
    u.y += @m_xf.position.y
    u

  b2Body::GetWorldVector = (localVector) ->
    b2Math.MulMV @m_xf.R, localVector

  b2Body::GetLocalPoint = (worldPoint) ->
    b2Math.MulXT @m_xf, worldPoint

  b2Body::GetLocalVector = (worldVector) ->
    b2Math.MulTMV @m_xf.R, worldVector

  b2Body::GetLinearVelocityFromWorldPoint = (worldPoint) ->
    new b2Vec2(@m_linearVelocity.x - @m_angularVelocity * (worldPoint.y - @m_sweep.c.y), @m_linearVelocity.y + @m_angularVelocity * (worldPoint.x - @m_sweep.c.x))

  b2Body::GetLinearVelocityFromLocalPoint = (localPoint) ->
    A = @m_xf.R
    worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y)
    worldPoint.x += @m_xf.position.x
    worldPoint.y += @m_xf.position.y
    new b2Vec2(@m_linearVelocity.x - @m_angularVelocity * (worldPoint.y - @m_sweep.c.y), @m_linearVelocity.y + @m_angularVelocity * (worldPoint.x - @m_sweep.c.x))

  b2Body::GetLinearDamping = ->
    @m_linearDamping

  b2Body::SetLinearDamping = (linearDamping) ->
    linearDamping = 0  if linearDamping is `undefined`
    @m_linearDamping = linearDamping
    return

  b2Body::GetAngularDamping = ->
    @m_angularDamping

  b2Body::SetAngularDamping = (angularDamping) ->
    angularDamping = 0  if angularDamping is `undefined`
    @m_angularDamping = angularDamping
    return

  b2Body::SetType = (type) ->
    type = 0  if type is `undefined`
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

  b2Body::GetType = ->
    @m_type

  b2Body::SetBullet = (flag) ->
    if flag
      @m_flags |= b2Body.e_bulletFlag
    else
      @m_flags &= ~b2Body.e_bulletFlag
    return

  b2Body::IsBullet = ->
    (@m_flags & b2Body.e_bulletFlag) is b2Body.e_bulletFlag

  b2Body::SetSleepingAllowed = (flag) ->
    if flag
      @m_flags |= b2Body.e_allowSleepFlag
    else
      @m_flags &= ~b2Body.e_allowSleepFlag
      @SetAwake true
    return

  b2Body::SetAwake = (flag) ->
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

  b2Body::IsAwake = ->
    (@m_flags & b2Body.e_awakeFlag) is b2Body.e_awakeFlag

  b2Body::SetFixedRotation = (fixed) ->
    if fixed
      @m_flags |= b2Body.e_fixedRotationFlag
    else
      @m_flags &= ~b2Body.e_fixedRotationFlag
    @ResetMassData()
    return

  b2Body::IsFixedRotation = ->
    (@m_flags & b2Body.e_fixedRotationFlag) is b2Body.e_fixedRotationFlag

  b2Body::SetActive = (flag) ->
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

  b2Body::IsActive = ->
    (@m_flags & b2Body.e_activeFlag) is b2Body.e_activeFlag

  b2Body::IsSleepingAllowed = ->
    (@m_flags & b2Body.e_allowSleepFlag) is b2Body.e_allowSleepFlag

  b2Body::GetFixtureList = ->
    @m_fixtureList

  b2Body::GetJointList = ->
    @m_jointList

  b2Body::GetControllerList = ->
    @m_controllerList

  b2Body::GetContactList = ->
    @m_contactList

  b2Body::GetNext = ->
    @m_next

  b2Body::GetUserData = ->
    @m_userData

  b2Body::SetUserData = (data) ->
    @m_userData = data
    return

  b2Body::GetWorld = ->
    @m_world

  b2Body::b2Body = (bd, world) ->
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

  b2Body::SynchronizeFixtures = ->
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

  b2Body::SynchronizeTransform = ->
    @m_xf.R.Set @m_sweep.a
    tMat = @m_xf.R
    tVec = @m_sweep.localCenter
    @m_xf.position.x = @m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    @m_xf.position.y = @m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    return

  b2Body::ShouldCollide = (other) ->
    return false  if @m_type isnt b2Body.b2_dynamicBody and other.m_type isnt b2Body.b2_dynamicBody
    jn = @m_jointList

    while jn
      return false  if jn.joint.m_collideConnected is false  if jn.other is other
      jn = jn.next
    true

  b2Body::Advance = (t) ->
    t = 0  if t is `undefined`
    @m_sweep.Advance t
    @m_sweep.c.SetV @m_sweep.c0
    @m_sweep.a = @m_sweep.a0
    @SynchronizeTransform()
    return

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2Body.s_xf1 = new b2Transform()
    Box2D.Dynamics.b2Body.e_islandFlag = 0x0001
    Box2D.Dynamics.b2Body.e_awakeFlag = 0x0002
    Box2D.Dynamics.b2Body.e_allowSleepFlag = 0x0004
    Box2D.Dynamics.b2Body.e_bulletFlag = 0x0008
    Box2D.Dynamics.b2Body.e_fixedRotationFlag = 0x0010
    Box2D.Dynamics.b2Body.e_activeFlag = 0x0020
    Box2D.Dynamics.b2Body.b2_staticBody = 0
    Box2D.Dynamics.b2Body.b2_kinematicBody = 1
    Box2D.Dynamics.b2Body.b2_dynamicBody = 2
    return

  b2BodyDef.b2BodyDef = ->
    @position = new b2Vec2()
    @linearVelocity = new b2Vec2()
    return

  b2BodyDef::b2BodyDef = ->
    @userData = null
    @position.Set 0.0, 0.0
    @angle = 0.0
    @linearVelocity.Set 0, 0
    @angularVelocity = 0.0
    @linearDamping = 0.0
    @angularDamping = 0.0
    @allowSleep = true
    @awake = true
    @fixedRotation = false
    @bullet = false
    @type = b2Body.b2_staticBody
    @active = true
    @inertiaScale = 1.0
    return

  b2ContactFilter.b2ContactFilter = ->

  b2ContactFilter::ShouldCollide = (fixtureA, fixtureB) ->
    filter1 = fixtureA.GetFilterData()
    filter2 = fixtureB.GetFilterData()
    return filter1.groupIndex > 0  if filter1.groupIndex is filter2.groupIndex and filter1.groupIndex isnt 0
    collide = (filter1.maskBits & filter2.categoryBits) isnt 0 and (filter1.categoryBits & filter2.maskBits) isnt 0
    collide

  b2ContactFilter::RayCollide = (userData, fixture) ->
    return true  unless userData
    @ShouldCollide ((if userData instanceof b2Fixture then userData else null)), fixture

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = new b2ContactFilter()
    return

  b2ContactImpulse.b2ContactImpulse = ->
    @normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints)
    @tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints)
    return

  b2ContactListener.b2ContactListener = ->

  b2ContactListener::BeginContact = (contact) ->

  b2ContactListener::EndContact = (contact) ->

  b2ContactListener::PreSolve = (contact, oldManifold) ->

  b2ContactListener::PostSolve = (contact, impulse) ->

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2ContactListener.b2_defaultListener = new b2ContactListener()
    return

  b2ContactManager.b2ContactManager = ->

  b2ContactManager::b2ContactManager = ->
    @m_world = null
    @m_contactCount = 0
    @m_contactFilter = b2ContactFilter.b2_defaultFilter
    @m_contactListener = b2ContactListener.b2_defaultListener
    @m_contactFactory = new b2ContactFactory(@m_allocator)
    @m_broadPhase = new b2DynamicTreeBroadPhase()
    return

  b2ContactManager::AddPair = (proxyUserDataA, proxyUserDataB) ->
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

  b2ContactManager::FindNewContacts = ->
    @m_broadPhase.UpdatePairs Box2D.generateCallback(this, @AddPair)
    return

  b2ContactManager::Destroy = (c) ->
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

  b2ContactManager::Collide = ->
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

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2ContactManager.s_evalCP = new b2ContactPoint()
    return

  b2DebugDraw.b2DebugDraw = ->

  b2DebugDraw::b2DebugDraw = ->

  b2DebugDraw::SetFlags = (flags) ->
    flags = 0  if flags is `undefined`
    return

  b2DebugDraw::GetFlags = ->

  b2DebugDraw::AppendFlags = (flags) ->
    flags = 0  if flags is `undefined`
    return

  b2DebugDraw::ClearFlags = (flags) ->
    flags = 0  if flags is `undefined`
    return

  b2DebugDraw::SetSprite = (sprite) ->

  b2DebugDraw::GetSprite = ->

  b2DebugDraw::SetDrawScale = (drawScale) ->
    drawScale = 0  if drawScale is `undefined`
    return

  b2DebugDraw::GetDrawScale = ->

  b2DebugDraw::SetLineThickness = (lineThickness) ->
    lineThickness = 0  if lineThickness is `undefined`
    return

  b2DebugDraw::GetLineThickness = ->

  b2DebugDraw::SetAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    return

  b2DebugDraw::GetAlpha = ->

  b2DebugDraw::SetFillAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    return

  b2DebugDraw::GetFillAlpha = ->

  b2DebugDraw::SetXFormScale = (xformScale) ->
    xformScale = 0  if xformScale is `undefined`
    return

  b2DebugDraw::GetXFormScale = ->

  b2DebugDraw::DrawPolygon = (vertices, vertexCount, color) ->
    vertexCount = 0  if vertexCount is `undefined`
    return

  b2DebugDraw::DrawSolidPolygon = (vertices, vertexCount, color) ->
    vertexCount = 0  if vertexCount is `undefined`
    return

  b2DebugDraw::DrawCircle = (center, radius, color) ->
    radius = 0  if radius is `undefined`
    return

  b2DebugDraw::DrawSolidCircle = (center, radius, axis, color) ->
    radius = 0  if radius is `undefined`
    return

  b2DebugDraw::DrawSegment = (p1, p2, color) ->

  b2DebugDraw::DrawTransform = (xf) ->

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2DebugDraw.e_shapeBit = 0x0001
    Box2D.Dynamics.b2DebugDraw.e_jointBit = 0x0002
    Box2D.Dynamics.b2DebugDraw.e_aabbBit = 0x0004
    Box2D.Dynamics.b2DebugDraw.e_pairBit = 0x0008
    Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit = 0x0010
    Box2D.Dynamics.b2DebugDraw.e_controllerBit = 0x0020
    return

  b2DestructionListener.b2DestructionListener = ->

  b2DestructionListener::SayGoodbyeJoint = (joint) ->

  b2DestructionListener::SayGoodbyeFixture = (fixture) ->

  b2FilterData.b2FilterData = ->
    @categoryBits = 0x0001
    @maskBits = 0xFFFF
    @groupIndex = 0
    return

  b2FilterData::Copy = ->
    copy = new b2FilterData()
    copy.categoryBits = @categoryBits
    copy.maskBits = @maskBits
    copy.groupIndex = @groupIndex
    copy

  b2Fixture.b2Fixture = ->
    @m_filter = new b2FilterData()
    return

  b2Fixture::GetType = ->
    @m_shape.GetType()

  b2Fixture::GetShape = ->
    @m_shape

  b2Fixture::SetSensor = (sensor) ->
    return  if @m_isSensor is sensor
    @m_isSensor = sensor
    return  unless @m_body?
    edge = @m_body.GetContactList()
    while edge
      contact = edge.contact
      fixtureA = contact.GetFixtureA()
      fixtureB = contact.GetFixtureB()
      contact.SetSensor fixtureA.IsSensor() or fixtureB.IsSensor()  if fixtureA is this or fixtureB is this
      edge = edge.next
    return

  b2Fixture::IsSensor = ->
    @m_isSensor

  b2Fixture::SetFilterData = (filter) ->
    @m_filter = filter.Copy()
    return  if @m_body
    edge = @m_body.GetContactList()
    while edge
      contact = edge.contact
      fixtureA = contact.GetFixtureA()
      fixtureB = contact.GetFixtureB()
      contact.FlagForFiltering()  if fixtureA is this or fixtureB is this
      edge = edge.next
    return

  b2Fixture::GetFilterData = ->
    @m_filter.Copy()

  b2Fixture::GetBody = ->
    @m_body

  b2Fixture::GetNext = ->
    @m_next

  b2Fixture::GetUserData = ->
    @m_userData

  b2Fixture::SetUserData = (data) ->
    @m_userData = data
    return

  b2Fixture::TestPoint = (p) ->
    @m_shape.TestPoint @m_body.GetTransform(), p

  b2Fixture::RayCast = (output, input) ->
    @m_shape.RayCast output, input, @m_body.GetTransform()

  b2Fixture::GetMassData = (massData) ->
    massData = null  if massData is `undefined`
    massData = new b2MassData()  unless massData?
    @m_shape.ComputeMass massData, @m_density
    massData

  b2Fixture::SetDensity = (density) ->
    density = 0  if density is `undefined`
    @m_density = density
    return

  b2Fixture::GetDensity = ->
    @m_density

  b2Fixture::GetFriction = ->
    @m_friction

  b2Fixture::SetFriction = (friction) ->
    friction = 0  if friction is `undefined`
    @m_friction = friction
    return

  b2Fixture::GetRestitution = ->
    @m_restitution

  b2Fixture::SetRestitution = (restitution) ->
    restitution = 0  if restitution is `undefined`
    @m_restitution = restitution
    return

  b2Fixture::GetAABB = ->
    @m_aabb

  b2Fixture::b2Fixture = ->
    @m_aabb = new b2AABB()
    @m_userData = null
    @m_body = null
    @m_next = null
    @m_shape = null
    @m_density = 0.0
    @m_friction = 0.0
    @m_restitution = 0.0
    return

  b2Fixture::Create = (body, xf, def) ->
    @m_userData = def.userData
    @m_friction = def.friction
    @m_restitution = def.restitution
    @m_body = body
    @m_next = null
    @m_filter = def.filter.Copy()
    @m_isSensor = def.isSensor
    @m_shape = def.shape.Copy()
    @m_density = def.density
    return

  b2Fixture::Destroy = ->
    @m_shape = null
    return

  b2Fixture::CreateProxy = (broadPhase, xf) ->
    @m_shape.ComputeAABB @m_aabb, xf
    @m_proxy = broadPhase.CreateProxy(@m_aabb, this)
    return

  b2Fixture::DestroyProxy = (broadPhase) ->
    return  unless @m_proxy?
    broadPhase.DestroyProxy @m_proxy
    @m_proxy = null
    return

  b2Fixture::Synchronize = (broadPhase, transform1, transform2) ->
    return  unless @m_proxy
    aabb1 = new b2AABB()
    aabb2 = new b2AABB()
    @m_shape.ComputeAABB aabb1, transform1
    @m_shape.ComputeAABB aabb2, transform2
    @m_aabb.Combine aabb1, aabb2
    displacement = b2Math.SubtractVV(transform2.position, transform1.position)
    broadPhase.MoveProxy @m_proxy, @m_aabb, displacement
    return

  b2FixtureDef.b2FixtureDef = ->
    @filter = new b2FilterData()
    return

  b2FixtureDef::b2FixtureDef = ->
    @shape = null
    @userData = null
    @friction = 0.2
    @restitution = 0.0
    @density = 0.0
    @filter.categoryBits = 0x0001
    @filter.maskBits = 0xFFFF
    @filter.groupIndex = 0
    @isSensor = false
    return

  b2Island.b2Island = ->

  b2Island::b2Island = ->
    @m_bodies = new Vector()
    @m_contacts = new Vector()
    @m_joints = new Vector()
    return

  b2Island::Initialize = (bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) ->
    bodyCapacity = 0  if bodyCapacity is `undefined`
    contactCapacity = 0  if contactCapacity is `undefined`
    jointCapacity = 0  if jointCapacity is `undefined`
    i = 0
    @m_bodyCapacity = bodyCapacity
    @m_contactCapacity = contactCapacity
    @m_jointCapacity = jointCapacity
    @m_bodyCount = 0
    @m_contactCount = 0
    @m_jointCount = 0
    @m_allocator = allocator
    @m_listener = listener
    @m_contactSolver = contactSolver
    i = @m_bodies.length
    while i < bodyCapacity
      @m_bodies[i] = null
      i++
    i = @m_contacts.length
    while i < contactCapacity
      @m_contacts[i] = null
      i++
    i = @m_joints.length
    while i < jointCapacity
      @m_joints[i] = null
      i++
    return

  b2Island::Clear = ->
    @m_bodyCount = 0
    @m_contactCount = 0
    @m_jointCount = 0
    return

  b2Island::Solve = (step, gravity, allowSleep) ->
    i = 0
    j = 0
    b = undefined
    joint = undefined
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  unless b.GetType() is b2Body.b2_dynamicBody
      b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x)
      b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y)
      b.m_angularVelocity += step.dt * b.m_invI * b.m_torque
      b.m_linearVelocity.Multiply b2Math.Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0)
      b.m_angularVelocity *= b2Math.Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0)
      ++i
    @m_contactSolver.Initialize step, @m_contacts, @m_contactCount, @m_allocator
    contactSolver = @m_contactSolver
    contactSolver.InitVelocityConstraints step
    i = 0
    while i < @m_jointCount
      joint = @m_joints[i]
      joint.InitVelocityConstraints step
      ++i
    i = 0
    while i < step.velocityIterations
      j = 0
      while j < @m_jointCount
        joint = @m_joints[j]
        joint.SolveVelocityConstraints step
        ++j
      contactSolver.SolveVelocityConstraints()
      ++i
    i = 0
    while i < @m_jointCount
      joint = @m_joints[i]
      joint.FinalizeVelocityConstraints()
      ++i
    contactSolver.FinalizeVelocityConstraints()
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  if b.GetType() is b2Body.b2_staticBody
      translationX = step.dt * b.m_linearVelocity.x
      translationY = step.dt * b.m_linearVelocity.y
      if (translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared
        b.m_linearVelocity.Normalize()
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt
      rotation = step.dt * b.m_angularVelocity
      if rotation * rotation > b2Settings.b2_maxRotationSquared
        if b.m_angularVelocity < 0.0
          b.m_angularVelocity = (-b2Settings.b2_maxRotation * step.inv_dt)
        else
          b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt
      b.m_sweep.c0.SetV b.m_sweep.c
      b.m_sweep.a0 = b.m_sweep.a
      b.m_sweep.c.x += step.dt * b.m_linearVelocity.x
      b.m_sweep.c.y += step.dt * b.m_linearVelocity.y
      b.m_sweep.a += step.dt * b.m_angularVelocity
      b.SynchronizeTransform()
      ++i
    i = 0
    while i < step.positionIterations
      contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
      jointsOkay = true
      j = 0
      while j < @m_jointCount
        joint = @m_joints[j]
        jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
        jointsOkay = jointsOkay and jointOkay
        ++j
      break  if contactsOkay and jointsOkay
      ++i
    @Report contactSolver.m_constraints
    if allowSleep
      minSleepTime = Number.MAX_VALUE
      linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance
      angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance
      i = 0
      while i < @m_bodyCount
        b = @m_bodies[i]
        continue  if b.GetType() is b2Body.b2_staticBody
        if (b.m_flags & b2Body.e_allowSleepFlag) is 0
          b.m_sleepTime = 0.0
          minSleepTime = 0.0
        if (b.m_flags & b2Body.e_allowSleepFlag) is 0 or b.m_angularVelocity * b.m_angularVelocity > angTolSqr or b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr
          b.m_sleepTime = 0.0
          minSleepTime = 0.0
        else
          b.m_sleepTime += step.dt
          minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime)
        ++i
      if minSleepTime >= b2Settings.b2_timeToSleep
        i = 0
        while i < @m_bodyCount
          b = @m_bodies[i]
          b.SetAwake false
          ++i
    return

  b2Island::SolveTOI = (subStep) ->
    i = 0
    j = 0
    @m_contactSolver.Initialize subStep, @m_contacts, @m_contactCount, @m_allocator
    contactSolver = @m_contactSolver
    i = 0
    while i < @m_jointCount
      @m_joints[i].InitVelocityConstraints subStep
      ++i
    i = 0
    while i < subStep.velocityIterations
      contactSolver.SolveVelocityConstraints()
      j = 0
      while j < @m_jointCount
        @m_joints[j].SolveVelocityConstraints subStep
        ++j
      ++i
    i = 0
    while i < @m_bodyCount
      b = @m_bodies[i]
      continue  if b.GetType() is b2Body.b2_staticBody
      translationX = subStep.dt * b.m_linearVelocity.x
      translationY = subStep.dt * b.m_linearVelocity.y
      if (translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared
        b.m_linearVelocity.Normalize()
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt
      rotation = subStep.dt * b.m_angularVelocity
      if rotation * rotation > b2Settings.b2_maxRotationSquared
        if b.m_angularVelocity < 0.0
          b.m_angularVelocity = (-b2Settings.b2_maxRotation * subStep.inv_dt)
        else
          b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt
      b.m_sweep.c0.SetV b.m_sweep.c
      b.m_sweep.a0 = b.m_sweep.a
      b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x
      b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y
      b.m_sweep.a += subStep.dt * b.m_angularVelocity
      b.SynchronizeTransform()
      ++i
    k_toiBaumgarte = 0.75
    i = 0
    while i < subStep.positionIterations
      contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte)
      jointsOkay = true
      j = 0
      while j < @m_jointCount
        jointOkay = @m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte)
        jointsOkay = jointsOkay and jointOkay
        ++j
      break  if contactsOkay and jointsOkay
      ++i
    @Report contactSolver.m_constraints
    return

  b2Island::Report = (constraints) ->
    return  unless @m_listener?
    i = 0

    while i < @m_contactCount
      c = @m_contacts[i]
      cc = constraints[i]
      j = 0

      while j < cc.pointCount
        b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse
        b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse
        ++j
      @m_listener.PostSolve c, b2Island.s_impulse
      ++i
    return

  b2Island::AddBody = (body) ->
    body.m_islandIndex = @m_bodyCount
    @m_bodies[@m_bodyCount++] = body
    return

  b2Island::AddContact = (contact) ->
    @m_contacts[@m_contactCount++] = contact
    return

  b2Island::AddJoint = (joint) ->
    @m_joints[@m_jointCount++] = joint
    return

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2Island.s_impulse = new b2ContactImpulse()
    return

  b2TimeStep.b2TimeStep = ->

  b2TimeStep::Set = (step) ->
    @dt = step.dt
    @inv_dt = step.inv_dt
    @positionIterations = step.positionIterations
    @velocityIterations = step.velocityIterations
    @warmStarting = step.warmStarting
    return

  b2World.b2World = ->
    @s_stack = new Vector()
    @m_contactManager = new b2ContactManager()
    @m_contactSolver = new b2ContactSolver()
    @m_island = new b2Island()
    return

  b2World::b2World = (gravity, doSleep) ->
    @m_destructionListener = null
    @m_debugDraw = null
    @m_bodyList = null
    @m_contactList = null
    @m_jointList = null
    @m_controllerList = null
    @m_bodyCount = 0
    @m_contactCount = 0
    @m_jointCount = 0
    @m_controllerCount = 0
    b2World.m_warmStarting = true
    b2World.m_continuousPhysics = true
    @m_allowSleep = doSleep
    @m_gravity = gravity
    @m_inv_dt0 = 0.0
    @m_contactManager.m_world = this
    bd = new b2BodyDef()
    @m_groundBody = @CreateBody(bd)
    return

  b2World::SetDestructionListener = (listener) ->
    @m_destructionListener = listener
    return

  b2World::SetContactFilter = (filter) ->
    @m_contactManager.m_contactFilter = filter
    return

  b2World::SetContactListener = (listener) ->
    @m_contactManager.m_contactListener = listener
    return

  b2World::SetDebugDraw = (debugDraw) ->
    @m_debugDraw = debugDraw
    return

  b2World::SetBroadPhase = (broadPhase) ->
    oldBroadPhase = @m_contactManager.m_broadPhase
    @m_contactManager.m_broadPhase = broadPhase
    b = @m_bodyList

    while b
      f = b.m_fixtureList

      while f
        f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f)
        f = f.m_next
      b = b.m_next
    return

  b2World::Validate = ->
    @m_contactManager.m_broadPhase.Validate()
    return

  b2World::GetProxyCount = ->
    @m_contactManager.m_broadPhase.GetProxyCount()

  b2World::CreateBody = (def) ->
    return null  if @IsLocked() is true
    b = new b2Body(def, this)
    b.m_prev = null
    b.m_next = @m_bodyList
    @m_bodyList.m_prev = b  if @m_bodyList
    @m_bodyList = b
    ++@m_bodyCount
    b

  b2World::DestroyBody = (b) ->
    return  if @IsLocked() is true
    jn = b.m_jointList
    while jn
      jn0 = jn
      jn = jn.next
      @m_destructionListener.SayGoodbyeJoint jn0.joint  if @m_destructionListener
      @DestroyJoint jn0.joint
    coe = b.m_controllerList
    while coe
      coe0 = coe
      coe = coe.nextController
      coe0.controller.RemoveBody b
    ce = b.m_contactList
    while ce
      ce0 = ce
      ce = ce.next
      @m_contactManager.Destroy ce0.contact
    b.m_contactList = null
    f = b.m_fixtureList
    while f
      f0 = f
      f = f.m_next
      @m_destructionListener.SayGoodbyeFixture f0  if @m_destructionListener
      f0.DestroyProxy @m_contactManager.m_broadPhase
      f0.Destroy()
    b.m_fixtureList = null
    b.m_fixtureCount = 0
    b.m_prev.m_next = b.m_next  if b.m_prev
    b.m_next.m_prev = b.m_prev  if b.m_next
    @m_bodyList = b.m_next  if b is @m_bodyList
    --@m_bodyCount
    return

  b2World::CreateJoint = (def) ->
    j = b2Joint.Create(def, null)
    j.m_prev = null
    j.m_next = @m_jointList
    @m_jointList.m_prev = j  if @m_jointList
    @m_jointList = j
    ++@m_jointCount
    j.m_edgeA.joint = j
    j.m_edgeA.other = j.m_bodyB
    j.m_edgeA.prev = null
    j.m_edgeA.next = j.m_bodyA.m_jointList
    j.m_bodyA.m_jointList.prev = j.m_edgeA  if j.m_bodyA.m_jointList
    j.m_bodyA.m_jointList = j.m_edgeA
    j.m_edgeB.joint = j
    j.m_edgeB.other = j.m_bodyA
    j.m_edgeB.prev = null
    j.m_edgeB.next = j.m_bodyB.m_jointList
    j.m_bodyB.m_jointList.prev = j.m_edgeB  if j.m_bodyB.m_jointList
    j.m_bodyB.m_jointList = j.m_edgeB
    bodyA = def.bodyA
    bodyB = def.bodyB
    if def.collideConnected is false
      edge = bodyB.GetContactList()
      while edge
        edge.contact.FlagForFiltering()  if edge.other is bodyA
        edge = edge.next
    j

  b2World::DestroyJoint = (j) ->
    collideConnected = j.m_collideConnected
    j.m_prev.m_next = j.m_next  if j.m_prev
    j.m_next.m_prev = j.m_prev  if j.m_next
    @m_jointList = j.m_next  if j is @m_jointList
    bodyA = j.m_bodyA
    bodyB = j.m_bodyB
    bodyA.SetAwake true
    bodyB.SetAwake true
    j.m_edgeA.prev.next = j.m_edgeA.next  if j.m_edgeA.prev
    j.m_edgeA.next.prev = j.m_edgeA.prev  if j.m_edgeA.next
    bodyA.m_jointList = j.m_edgeA.next  if j.m_edgeA is bodyA.m_jointList
    j.m_edgeA.prev = null
    j.m_edgeA.next = null
    j.m_edgeB.prev.next = j.m_edgeB.next  if j.m_edgeB.prev
    j.m_edgeB.next.prev = j.m_edgeB.prev  if j.m_edgeB.next
    bodyB.m_jointList = j.m_edgeB.next  if j.m_edgeB is bodyB.m_jointList
    j.m_edgeB.prev = null
    j.m_edgeB.next = null
    b2Joint.Destroy j, null
    --@m_jointCount
    if collideConnected is false
      edge = bodyB.GetContactList()
      while edge
        edge.contact.FlagForFiltering()  if edge.other is bodyA
        edge = edge.next
    return

  b2World::AddController = (c) ->
    c.m_next = @m_controllerList
    c.m_prev = null
    @m_controllerList = c
    c.m_world = this
    @m_controllerCount++
    c

  b2World::RemoveController = (c) ->
    c.m_prev.m_next = c.m_next  if c.m_prev
    c.m_next.m_prev = c.m_prev  if c.m_next
    @m_controllerList = c.m_next  if @m_controllerList is c
    @m_controllerCount--
    return

  b2World::CreateController = (controller) ->
    throw new Error("Controller can only be a member of one world")  unless controller.m_world is this
    controller.m_next = @m_controllerList
    controller.m_prev = null
    @m_controllerList.m_prev = controller  if @m_controllerList
    @m_controllerList = controller
    ++@m_controllerCount
    controller.m_world = this
    controller

  b2World::DestroyController = (controller) ->
    controller.Clear()
    controller.m_next.m_prev = controller.m_prev  if controller.m_next
    controller.m_prev.m_next = controller.m_next  if controller.m_prev
    @m_controllerList = controller.m_next  if controller is @m_controllerList
    --@m_controllerCount
    return

  b2World::SetWarmStarting = (flag) ->
    b2World.m_warmStarting = flag
    return

  b2World::SetContinuousPhysics = (flag) ->
    b2World.m_continuousPhysics = flag
    return

  b2World::GetBodyCount = ->
    @m_bodyCount

  b2World::GetJointCount = ->
    @m_jointCount

  b2World::GetContactCount = ->
    @m_contactCount

  b2World::SetGravity = (gravity) ->
    @m_gravity = gravity
    return

  b2World::GetGravity = ->
    @m_gravity

  b2World::GetGroundBody = ->
    @m_groundBody

  b2World::Step = (dt, velocityIterations, positionIterations) ->
    dt = 0  if dt is `undefined`
    velocityIterations = 0  if velocityIterations is `undefined`
    positionIterations = 0  if positionIterations is `undefined`
    if @m_flags & b2World.e_newFixture
      @m_contactManager.FindNewContacts()
      @m_flags &= ~b2World.e_newFixture
    @m_flags |= b2World.e_locked
    step = b2World.s_timestep2
    step.dt = dt
    step.velocityIterations = velocityIterations
    step.positionIterations = positionIterations
    if dt > 0.0
      step.inv_dt = 1.0 / dt
    else
      step.inv_dt = 0.0
    step.dtRatio = @m_inv_dt0 * dt
    step.warmStarting = b2World.m_warmStarting
    @m_contactManager.Collide()
    @Solve step  if step.dt > 0.0
    @SolveTOI step  if b2World.m_continuousPhysics and step.dt > 0.0
    @m_inv_dt0 = step.inv_dt  if step.dt > 0.0
    @m_flags &= ~b2World.e_locked
    return

  b2World::ClearForces = ->
    body = @m_bodyList

    while body
      body.m_force.SetZero()
      body.m_torque = 0.0
      body = body.m_next
    return

  b2World::DrawDebugData = ->
    return  unless @m_debugDraw?
    @m_debugDraw.m_sprite.graphics.clear()
    flags = @m_debugDraw.GetFlags()
    i = 0
    b = undefined
    f = undefined
    s = undefined
    j = undefined
    bp = undefined
    invQ = new b2Vec2
    x1 = new b2Vec2
    x2 = new b2Vec2
    xf = undefined
    b1 = new b2AABB()
    b2 = new b2AABB()
    vs = [
      new b2Vec2()
      new b2Vec2()
      new b2Vec2()
      new b2Vec2()
    ]
    color = new b2Color(0, 0, 0)
    if flags & b2DebugDraw.e_shapeBit
      b = @m_bodyList
      while b
        xf = b.m_xf
        f = b.GetFixtureList()
        while f
          s = f.GetShape()
          if b.IsActive() is false
            color.Set 0.5, 0.5, 0.3
            @DrawShape s, xf, color
          else if b.GetType() is b2Body.b2_staticBody
            color.Set 0.5, 0.9, 0.5
            @DrawShape s, xf, color
          else if b.GetType() is b2Body.b2_kinematicBody
            color.Set 0.5, 0.5, 0.9
            @DrawShape s, xf, color
          else if b.IsAwake() is false
            color.Set 0.6, 0.6, 0.6
            @DrawShape s, xf, color
          else
            color.Set 0.9, 0.7, 0.7
            @DrawShape s, xf, color
          f = f.m_next
        b = b.m_next
    if flags & b2DebugDraw.e_jointBit
      j = @m_jointList
      while j
        @DrawJoint j
        j = j.m_next
    if flags & b2DebugDraw.e_controllerBit
      c = @m_controllerList

      while c
        c.Draw @m_debugDraw
        c = c.m_next
    if flags & b2DebugDraw.e_pairBit
      color.Set 0.3, 0.9, 0.9
      contact = @m_contactManager.m_contactList

      while contact
        fixtureA = contact.GetFixtureA()
        fixtureB = contact.GetFixtureB()
        cA = fixtureA.GetAABB().GetCenter()
        cB = fixtureB.GetAABB().GetCenter()
        @m_debugDraw.DrawSegment cA, cB, color
        contact = contact.GetNext()
    if flags & b2DebugDraw.e_aabbBit
      bp = @m_contactManager.m_broadPhase
      vs = [
        new b2Vec2()
        new b2Vec2()
        new b2Vec2()
        new b2Vec2()
      ]
      b = @m_bodyList
      while b
        continue  if b.IsActive() is false
        f = b.GetFixtureList()
        while f
          aabb = bp.GetFatAABB(f.m_proxy)
          vs[0].Set aabb.lowerBound.x, aabb.lowerBound.y
          vs[1].Set aabb.upperBound.x, aabb.lowerBound.y
          vs[2].Set aabb.upperBound.x, aabb.upperBound.y
          vs[3].Set aabb.lowerBound.x, aabb.upperBound.y
          @m_debugDraw.DrawPolygon vs, 4, color
          f = f.GetNext()
        b = b.GetNext()
    if flags & b2DebugDraw.e_centerOfMassBit
      b = @m_bodyList
      while b
        xf = b2World.s_xf
        xf.R = b.m_xf.R
        xf.position = b.GetWorldCenter()
        @m_debugDraw.DrawTransform xf
        b = b.m_next
    return

  b2World::QueryAABB = (callback, aabb) ->
    WorldQueryWrapper = (proxy) ->
      callback broadPhase.GetUserData(proxy)
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    broadPhase.Query WorldQueryWrapper, aabb
    return

  b2World::QueryShape = (callback, shape, transform) ->
    WorldQueryWrapper = (proxy) ->
      fixture = ((if broadPhase.GetUserData(proxy) instanceof b2Fixture then broadPhase.GetUserData(proxy) else null))
      return callback(fixture)  if b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())
      true
    __this = this
    transform = null  if transform is `undefined`
    unless transform?
      transform = new b2Transform()
      transform.SetIdentity()
    broadPhase = __this.m_contactManager.m_broadPhase
    aabb = new b2AABB()
    shape.ComputeAABB aabb, transform
    broadPhase.Query WorldQueryWrapper, aabb
    return

  b2World::QueryPoint = (callback, p) ->
    WorldQueryWrapper = (proxy) ->
      fixture = ((if broadPhase.GetUserData(proxy) instanceof b2Fixture then broadPhase.GetUserData(proxy) else null))
      return callback(fixture)  if fixture.TestPoint(p)
      true
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    aabb = new b2AABB()
    aabb.lowerBound.Set p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop
    aabb.upperBound.Set p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop
    broadPhase.Query WorldQueryWrapper, aabb
    return

  b2World::RayCast = (callback, point1, point2) ->
    RayCastWrapper = (input, proxy) ->
      userData = broadPhase.GetUserData(proxy)
      fixture = ((if userData instanceof b2Fixture then userData else null))
      hit = fixture.RayCast(output, input)
      if hit
        fraction = output.fraction
        point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y)
        return callback(fixture, point, output.normal, fraction)
      input.maxFraction
    __this = this
    broadPhase = __this.m_contactManager.m_broadPhase
    output = new b2RayCastOutput
    input = new b2RayCastInput(point1, point2)
    broadPhase.RayCast RayCastWrapper, input
    return

  b2World::RayCastOne = (point1, point2) ->
    RayCastOneWrapper = (fixture, point, normal, fraction) ->
      fraction = 0  if fraction is `undefined`
      result = fixture
      fraction
    __this = this
    result = undefined
    __this.RayCast RayCastOneWrapper, point1, point2
    result

  b2World::RayCastAll = (point1, point2) ->
    RayCastAllWrapper = (fixture, point, normal, fraction) ->
      fraction = 0  if fraction is `undefined`
      result[result.length] = fixture
      1
    __this = this
    result = new Vector()
    __this.RayCast RayCastAllWrapper, point1, point2
    result

  b2World::GetBodyList = ->
    @m_bodyList

  b2World::GetJointList = ->
    @m_jointList

  b2World::GetContactList = ->
    @m_contactList

  b2World::IsLocked = ->
    (@m_flags & b2World.e_locked) > 0

  b2World::Solve = (step) ->
    b = undefined
    controller = @m_controllerList

    while controller
      controller.Step step
      controller = controller.m_next
    island = @m_island
    island.Initialize @m_bodyCount, @m_contactCount, @m_jointCount, null, @m_contactManager.m_contactListener, @m_contactSolver
    b = @m_bodyList
    while b
      b.m_flags &= ~b2Body.e_islandFlag
      b = b.m_next
    c = @m_contactList

    while c
      c.m_flags &= ~b2Contact.e_islandFlag
      c = c.m_next
    j = @m_jointList

    while j
      j.m_islandFlag = false
      j = j.m_next
    stackSize = parseInt(@m_bodyCount)
    stack = @s_stack
    seed = @m_bodyList

    while seed
      continue  if seed.m_flags & b2Body.e_islandFlag
      continue  if seed.IsAwake() is false or seed.IsActive() is false
      continue  if seed.GetType() is b2Body.b2_staticBody
      island.Clear()
      stackCount = 0
      stack[stackCount++] = seed
      seed.m_flags |= b2Body.e_islandFlag
      while stackCount > 0
        b = stack[--stackCount]
        island.AddBody b
        b.SetAwake true  if b.IsAwake() is false
        continue  if b.GetType() is b2Body.b2_staticBody
        other = undefined
        ce = b.m_contactList

        while ce
          continue  if ce.contact.m_flags & b2Contact.e_islandFlag
          continue  if ce.contact.IsSensor() is true or ce.contact.IsEnabled() is false or ce.contact.IsTouching() is false
          island.AddContact ce.contact
          ce.contact.m_flags |= b2Contact.e_islandFlag
          other = ce.other
          continue  if other.m_flags & b2Body.e_islandFlag
          stack[stackCount++] = other
          other.m_flags |= b2Body.e_islandFlag
          ce = ce.next
        jn = b.m_jointList

        while jn
          continue  if jn.joint.m_islandFlag is true
          other = jn.other
          continue  if other.IsActive() is false
          island.AddJoint jn.joint
          jn.joint.m_islandFlag = true
          continue  if other.m_flags & b2Body.e_islandFlag
          stack[stackCount++] = other
          other.m_flags |= b2Body.e_islandFlag
          jn = jn.next
      island.Solve step, @m_gravity, @m_allowSleep
      i = 0

      while i < island.m_bodyCount
        b = island.m_bodies[i]
        b.m_flags &= ~b2Body.e_islandFlag  if b.GetType() is b2Body.b2_staticBody
        ++i
      seed = seed.m_next
    i = 0
    while i < stack.length
      break  unless stack[i]
      stack[i] = null
      ++i
    b = @m_bodyList
    while b
      continue  if b.IsAwake() is false or b.IsActive() is false
      continue  if b.GetType() is b2Body.b2_staticBody
      b.SynchronizeFixtures()
      b = b.m_next
    @m_contactManager.FindNewContacts()
    return

  b2World::SolveTOI = (step) ->
    b = undefined
    fA = undefined
    fB = undefined
    bA = undefined
    bB = undefined
    cEdge = undefined
    j = undefined
    island = @m_island
    island.Initialize @m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, @m_contactManager.m_contactListener, @m_contactSolver
    queue = b2World.s_queue
    b = @m_bodyList
    while b
      b.m_flags &= ~b2Body.e_islandFlag
      b.m_sweep.t0 = 0.0
      b = b.m_next
    c = undefined
    c = @m_contactList
    while c
      c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag)
      c = c.m_next
    j = @m_jointList
    while j
      j.m_islandFlag = false
      j = j.m_next
    loop
      minContact = null
      minTOI = 1.0
      c = @m_contactList
      while c
        continue  if c.IsSensor() is true or c.IsEnabled() is false or c.IsContinuous() is false
        toi = 1.0
        if c.m_flags & b2Contact.e_toiFlag
          toi = c.m_toi
        else
          fA = c.m_fixtureA
          fB = c.m_fixtureB
          bA = fA.m_body
          bB = fB.m_body
          continue  if (bA.GetType() isnt b2Body.b2_dynamicBody or bA.IsAwake() is false) and (bB.GetType() isnt b2Body.b2_dynamicBody or bB.IsAwake() is false)
          t0 = bA.m_sweep.t0
          if bA.m_sweep.t0 < bB.m_sweep.t0
            t0 = bB.m_sweep.t0
            bA.m_sweep.Advance t0
          else if bB.m_sweep.t0 < bA.m_sweep.t0
            t0 = bA.m_sweep.t0
            bB.m_sweep.Advance t0
          toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep)
          b2Settings.b2Assert 0.0 <= toi and toi <= 1.0
          if toi > 0.0 and toi < 1.0
            toi = (1.0 - toi) * t0 + toi
            toi = 1  if toi > 1
          c.m_toi = toi
          c.m_flags |= b2Contact.e_toiFlag
        if Number.MIN_VALUE < toi and toi < minTOI
          minContact = c
          minTOI = toi
        c = c.m_next
      break  if not minContact? or 1.0 - 100.0 * Number.MIN_VALUE < minTOI
      fA = minContact.m_fixtureA
      fB = minContact.m_fixtureB
      bA = fA.m_body
      bB = fB.m_body
      b2World.s_backupA.Set bA.m_sweep
      b2World.s_backupB.Set bB.m_sweep
      bA.Advance minTOI
      bB.Advance minTOI
      minContact.Update @m_contactManager.m_contactListener
      minContact.m_flags &= ~b2Contact.e_toiFlag
      if minContact.IsSensor() is true or minContact.IsEnabled() is false
        bA.m_sweep.Set b2World.s_backupA
        bB.m_sweep.Set b2World.s_backupB
        bA.SynchronizeTransform()
        bB.SynchronizeTransform()
        continue
      continue  if minContact.IsTouching() is false
      seed = bA
      seed = bB  unless seed.GetType() is b2Body.b2_dynamicBody
      island.Clear()
      queueStart = 0
      queueSize = 0
      queue[queueStart + queueSize++] = seed
      seed.m_flags |= b2Body.e_islandFlag
      while queueSize > 0
        b = queue[queueStart++]
        --queueSize
        island.AddBody b
        b.SetAwake true  if b.IsAwake() is false
        continue  unless b.GetType() is b2Body.b2_dynamicBody
        cEdge = b.m_contactList
        while cEdge
          break  if island.m_contactCount is island.m_contactCapacity
          continue  if cEdge.contact.m_flags & b2Contact.e_islandFlag
          continue  if cEdge.contact.IsSensor() is true or cEdge.contact.IsEnabled() is false or cEdge.contact.IsTouching() is false
          island.AddContact cEdge.contact
          cEdge.contact.m_flags |= b2Contact.e_islandFlag
          other = cEdge.other
          continue  if other.m_flags & b2Body.e_islandFlag
          unless other.GetType() is b2Body.b2_staticBody
            other.Advance minTOI
            other.SetAwake true
          queue[queueStart + queueSize] = other
          ++queueSize
          other.m_flags |= b2Body.e_islandFlag
          cEdge = cEdge.next
        jEdge = b.m_jointList

        while jEdge
          continue  if island.m_jointCount is island.m_jointCapacity
          continue  if jEdge.joint.m_islandFlag is true
          other = jEdge.other
          continue  if other.IsActive() is false
          island.AddJoint jEdge.joint
          jEdge.joint.m_islandFlag = true
          continue  if other.m_flags & b2Body.e_islandFlag
          unless other.GetType() is b2Body.b2_staticBody
            other.Advance minTOI
            other.SetAwake true
          queue[queueStart + queueSize] = other
          ++queueSize
          other.m_flags |= b2Body.e_islandFlag
          jEdge = jEdge.next
      subStep = b2World.s_timestep
      subStep.warmStarting = false
      subStep.dt = (1.0 - minTOI) * step.dt
      subStep.inv_dt = 1.0 / subStep.dt
      subStep.dtRatio = 0.0
      subStep.velocityIterations = step.velocityIterations
      subStep.positionIterations = step.positionIterations
      island.SolveTOI subStep
      i = 0
      i = 0
      while i < island.m_bodyCount
        b = island.m_bodies[i]
        b.m_flags &= ~b2Body.e_islandFlag
        continue  if b.IsAwake() is false
        continue  unless b.GetType() is b2Body.b2_dynamicBody
        b.SynchronizeFixtures()
        cEdge = b.m_contactList
        while cEdge
          cEdge.contact.m_flags &= ~b2Contact.e_toiFlag
          cEdge = cEdge.next
        ++i
      i = 0
      while i < island.m_contactCount
        c = island.m_contacts[i]
        c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag)
        ++i
      i = 0
      while i < island.m_jointCount
        j = island.m_joints[i]
        j.m_islandFlag = false
        ++i
      @m_contactManager.FindNewContacts()
    return

  b2World::DrawJoint = (joint) ->
    b1 = joint.GetBodyA()
    b2 = joint.GetBodyB()
    xf1 = b1.m_xf
    xf2 = b2.m_xf
    x1 = xf1.position
    x2 = xf2.position
    p1 = joint.GetAnchorA()
    p2 = joint.GetAnchorB()
    color = b2World.s_jointColor
    switch joint.m_type
      when b2Joint.e_distanceJoint
        @m_debugDraw.DrawSegment p1, p2, color
      when b2Joint.e_pulleyJoint
        pulley = ((if joint instanceof b2PulleyJoint then joint else null))
        s1 = pulley.GetGroundAnchorA()
        s2 = pulley.GetGroundAnchorB()
        @m_debugDraw.DrawSegment s1, p1, color
        @m_debugDraw.DrawSegment s2, p2, color
        @m_debugDraw.DrawSegment s1, s2, color
      when b2Joint.e_mouseJoint
        @m_debugDraw.DrawSegment p1, p2, color
      else
        @m_debugDraw.DrawSegment x1, p1, color  unless b1 is @m_groundBody
        @m_debugDraw.DrawSegment p1, p2, color
        @m_debugDraw.DrawSegment x2, p2, color  unless b2 is @m_groundBody
    return

  b2World::DrawShape = (shape, xf, color) ->
    switch shape.m_type
      when b2Shape.e_circleShape
        circle = ((if shape instanceof b2CircleShape then shape else null))
        center = b2Math.MulX(xf, circle.m_p)
        radius = circle.m_radius
        axis = xf.R.col1
        @m_debugDraw.DrawSolidCircle center, radius, axis, color
      when b2Shape.e_polygonShape
        i = 0
        poly = ((if shape instanceof b2PolygonShape then shape else null))
        vertexCount = parseInt(poly.GetVertexCount())
        localVertices = poly.GetVertices()
        vertices = new Vector(vertexCount)
        i = 0
        while i < vertexCount
          vertices[i] = b2Math.MulX(xf, localVertices[i])
          ++i
        @m_debugDraw.DrawSolidPolygon vertices, vertexCount, color
      when b2Shape.e_edgeShape
        edge = ((if shape instanceof b2EdgeShape then shape else null))
        @m_debugDraw.DrawSegment b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color

  Box2D.postDefs.push ->
    Box2D.Dynamics.b2World.s_timestep2 = new b2TimeStep()
    Box2D.Dynamics.b2World.s_xf = new b2Transform()
    Box2D.Dynamics.b2World.s_backupA = new b2Sweep()
    Box2D.Dynamics.b2World.s_backupB = new b2Sweep()
    Box2D.Dynamics.b2World.s_timestep = new b2TimeStep()
    Box2D.Dynamics.b2World.s_queue = new Vector()
    Box2D.Dynamics.b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8)
    Box2D.Dynamics.b2World.e_newFixture = 0x0001
    Box2D.Dynamics.b2World.e_locked = 0x0002
    return

  return
)()
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
(->
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
  b2Mat22 = Box2D.Common.Math.b2Mat22
  b2Mat33 = Box2D.Common.Math.b2Mat33
  b2Math = Box2D.Common.Math.b2Math
  b2Sweep = Box2D.Common.Math.b2Sweep
  b2Transform = Box2D.Common.Math.b2Transform
  b2Vec2 = Box2D.Common.Math.b2Vec2
  b2Vec3 = Box2D.Common.Math.b2Vec3
  b2Color = Box2D.Common.b2Color
  b2internal = Box2D.Common.b2internal
  b2Settings = Box2D.Common.b2Settings
  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
  b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef
  b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape
  b2MassData = Box2D.Collision.Shapes.b2MassData
  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
  b2Shape = Box2D.Collision.Shapes.b2Shape
  b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController
  b2ConstantAccelController = Box2D.Dynamics.Controllers.b2ConstantAccelController
  b2ConstantForceController = Box2D.Dynamics.Controllers.b2ConstantForceController
  b2Controller = Box2D.Dynamics.Controllers.b2Controller
  b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge
  b2GravityController = Box2D.Dynamics.Controllers.b2GravityController
  b2TensorDampingController = Box2D.Dynamics.Controllers.b2TensorDampingController
  Box2D.inherit b2BuoyancyController, Box2D.Dynamics.Controllers.b2Controller
  b2BuoyancyController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2BuoyancyController.b2BuoyancyController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @normal = new b2Vec2(0, (-1))
    @offset = 0
    @density = 0
    @velocity = new b2Vec2(0, 0)
    @linearDrag = 2
    @angularDrag = 1
    @useDensity = false
    @useWorldGravity = true
    @gravity = null
    return

  b2BuoyancyController::Step = (step) ->
    return  unless @m_bodyList
    @gravity = @GetWorld().GetGravity().Copy()  if @useWorldGravity
    i = @m_bodyList

    while i
      body = i.body
      continue  if body.IsAwake() is false
      areac = new b2Vec2()
      massc = new b2Vec2()
      area = 0.0
      mass = 0.0
      fixture = body.GetFixtureList()

      while fixture
        sc = new b2Vec2()
        sarea = fixture.GetShape().ComputeSubmergedArea(@normal, @offset, body.GetTransform(), sc)
        area += sarea
        areac.x += sarea * sc.x
        areac.y += sarea * sc.y
        shapeDensity = 0
        if @useDensity
          shapeDensity = 1
        else
          shapeDensity = 1
        mass += sarea * shapeDensity
        massc.x += sarea * sc.x * shapeDensity
        massc.y += sarea * sc.y * shapeDensity
        fixture = fixture.GetNext()
      areac.x /= area
      areac.y /= area
      massc.x /= mass
      massc.y /= mass
      continue  if area < Number.MIN_VALUE
      buoyancyForce = @gravity.GetNegative()
      buoyancyForce.Multiply @density * area
      body.ApplyForce buoyancyForce, massc
      dragForce = body.GetLinearVelocityFromWorldPoint(areac)
      dragForce.Subtract @velocity
      dragForce.Multiply (-@linearDrag * area)
      body.ApplyForce dragForce, areac
      body.ApplyTorque (-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * @angularDrag)
      i = i.nextBody
    return

  b2BuoyancyController::Draw = (debugDraw) ->
    r = 1000
    p1 = new b2Vec2()
    p2 = new b2Vec2()
    p1.x = @normal.x * @offset + @normal.y * r
    p1.y = @normal.y * @offset - @normal.x * r
    p2.x = @normal.x * @offset - @normal.y * r
    p2.y = @normal.y * @offset + @normal.x * r
    color = new b2Color(0, 0, 1)
    debugDraw.DrawSegment p1, p2, color
    return

  Box2D.inherit b2ConstantAccelController, Box2D.Dynamics.Controllers.b2Controller
  b2ConstantAccelController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2ConstantAccelController.b2ConstantAccelController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @A = new b2Vec2(0, 0)
    return

  b2ConstantAccelController::Step = (step) ->
    smallA = new b2Vec2(@A.x * step.dt, @A.y * step.dt)
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.SetLinearVelocity new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y)
      i = i.nextBody
    return

  Box2D.inherit b2ConstantForceController, Box2D.Dynamics.Controllers.b2Controller
  b2ConstantForceController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2ConstantForceController.b2ConstantForceController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @F = new b2Vec2(0, 0)
    return

  b2ConstantForceController::Step = (step) ->
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      body.ApplyForce @F, body.GetWorldCenter()
      i = i.nextBody
    return

  b2Controller.b2Controller = ->

  b2Controller::Step = (step) ->

  b2Controller::Draw = (debugDraw) ->

  b2Controller::AddBody = (body) ->
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

  b2Controller::RemoveBody = (body) ->
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

  b2Controller::Clear = ->
    @RemoveBody @m_bodyList.body  while @m_bodyList
    return

  b2Controller::GetNext = ->
    @m_next

  b2Controller::GetWorld = ->
    @m_world

  b2Controller::GetBodyList = ->
    @m_bodyList

  b2ControllerEdge.b2ControllerEdge = ->

  Box2D.inherit b2GravityController, Box2D.Dynamics.Controllers.b2Controller
  b2GravityController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2GravityController.b2GravityController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @G = 1
    @invSqr = true
    return

  b2GravityController::Step = (step) ->
    i = null
    body1 = null
    p1 = null
    mass1 = 0
    j = null
    body2 = null
    p2 = null
    dx = 0
    dy = 0
    r2 = 0
    f = null
    if @invSqr
      i = @m_bodyList
      while i
        body1 = i.body
        p1 = body1.GetWorldCenter()
        mass1 = body1.GetMass()
        j = @m_bodyList
        while j isnt i
          body2 = j.body
          p2 = body2.GetWorldCenter()
          dx = p2.x - p1.x
          dy = p2.y - p1.y
          r2 = dx * dx + dy * dy
          continue  if r2 < Number.MIN_VALUE
          f = new b2Vec2(dx, dy)
          f.Multiply @G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass()
          body1.ApplyForce f, p1  if body1.IsAwake()
          f.Multiply (-1)
          body2.ApplyForce f, p2  if body2.IsAwake()
          j = j.nextBody
        i = i.nextBody
    else
      i = @m_bodyList
      while i
        body1 = i.body
        p1 = body1.GetWorldCenter()
        mass1 = body1.GetMass()
        j = @m_bodyList
        while j isnt i
          body2 = j.body
          p2 = body2.GetWorldCenter()
          dx = p2.x - p1.x
          dy = p2.y - p1.y
          r2 = dx * dx + dy * dy
          continue  if r2 < Number.MIN_VALUE
          f = new b2Vec2(dx, dy)
          f.Multiply @G / r2 * mass1 * body2.GetMass()
          body1.ApplyForce f, p1  if body1.IsAwake()
          f.Multiply (-1)
          body2.ApplyForce f, p2  if body2.IsAwake()
          j = j.nextBody
        i = i.nextBody
    return

  Box2D.inherit b2TensorDampingController, Box2D.Dynamics.Controllers.b2Controller
  b2TensorDampingController::__super = Box2D.Dynamics.Controllers.b2Controller::
  b2TensorDampingController.b2TensorDampingController = ->
    Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply this, arguments
    @T = new b2Mat22()
    @maxTimestep = 0
    return

  b2TensorDampingController::SetAxisAligned = (xDamping, yDamping) ->
    xDamping = 0  if xDamping is `undefined`
    yDamping = 0  if yDamping is `undefined`
    @T.col1.x = (-xDamping)
    @T.col1.y = 0
    @T.col2.x = 0
    @T.col2.y = (-yDamping)
    if xDamping > 0 or yDamping > 0
      @maxTimestep = 1 / Math.max(xDamping, yDamping)
    else
      @maxTimestep = 0
    return

  b2TensorDampingController::Step = (step) ->
    timestep = step.dt
    return  if timestep <= Number.MIN_VALUE
    timestep = @maxTimestep  if timestep > @maxTimestep and @maxTimestep > 0
    i = @m_bodyList

    while i
      body = i.body
      continue  unless body.IsAwake()
      damping = body.GetWorldVector(b2Math.MulMV(@T, body.GetLocalVector(body.GetLinearVelocity())))
      body.SetLinearVelocity new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep)
      i = i.nextBody
    return

  return
)()
(->
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
  b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint
  b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef
  b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint
  b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef
  b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint
  b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef
  b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian
  b2Joint = Box2D.Dynamics.Joints.b2Joint
  b2JointDef = Box2D.Dynamics.Joints.b2JointDef
  b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge
  b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint
  b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef
  b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint
  b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
  b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint
  b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef
  b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint
  b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef
  b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint
  b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
  b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint
  b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
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
  Box2D.inherit b2DistanceJoint, Box2D.Dynamics.Joints.b2Joint
  b2DistanceJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2DistanceJoint.b2DistanceJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u = new b2Vec2()
    return

  b2DistanceJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2DistanceJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2DistanceJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_u.x, inv_dt * @m_impulse * @m_u.y)

  b2DistanceJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2DistanceJoint::GetLength = ->
    @m_length

  b2DistanceJoint::SetLength = (length) ->
    length = 0  if length is `undefined`
    @m_length = length
    return

  b2DistanceJoint::GetFrequency = ->
    @m_frequencyHz

  b2DistanceJoint::SetFrequency = (hz) ->
    hz = 0  if hz is `undefined`
    @m_frequencyHz = hz
    return

  b2DistanceJoint::GetDampingRatio = ->
    @m_dampingRatio

  b2DistanceJoint::SetDampingRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_dampingRatio = ratio
    return

  b2DistanceJoint::b2DistanceJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_length = def.length
    @m_frequencyHz = def.frequencyHz
    @m_dampingRatio = def.dampingRatio
    @m_impulse = 0.0
    @m_gamma = 0.0
    @m_bias = 0.0
    return

  b2DistanceJoint::InitVelocityConstraints = (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    @m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    @m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    length = Math.sqrt(@m_u.x * @m_u.x + @m_u.y * @m_u.y)
    if length > b2Settings.b2_linearSlop
      @m_u.Multiply 1.0 / length
    else
      @m_u.SetZero()
    cr1u = (r1X * @m_u.y - r1Y * @m_u.x)
    cr2u = (r2X * @m_u.y - r2Y * @m_u.x)
    invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u
    @m_mass = (if invMass isnt 0.0 then 1.0 / invMass else 0.0)
    if @m_frequencyHz > 0.0
      C = length - @m_length
      omega = 2.0 * Math.PI * @m_frequencyHz
      d = 2.0 * @m_mass * @m_dampingRatio * omega
      k = @m_mass * omega * omega
      @m_gamma = step.dt * (d + step.dt * k)
      @m_gamma = (if @m_gamma isnt 0.0 then 1 / @m_gamma else 0.0)
      @m_bias = C * step.dt * k * @m_gamma
      @m_mass = invMass + @m_gamma
      @m_mass = (if @m_mass isnt 0.0 then 1.0 / @m_mass else 0.0)
    if step.warmStarting
      @m_impulse *= step.dtRatio
      PX = @m_impulse * @m_u.x
      PY = @m_impulse * @m_u.y
      bA.m_linearVelocity.x -= bA.m_invMass * PX
      bA.m_linearVelocity.y -= bA.m_invMass * PY
      bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX)
      bB.m_linearVelocity.x += bB.m_invMass * PX
      bB.m_linearVelocity.y += bB.m_invMass * PY
      bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX)
    else
      @m_impulse = 0.0
    return

  b2DistanceJoint::SolveVelocityConstraints = (step) ->
    tMat = undefined
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
    v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
    v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
    v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
    Cdot = (@m_u.x * (v2X - v1X) + @m_u.y * (v2Y - v1Y))
    impulse = (-@m_mass * (Cdot + @m_bias + @m_gamma * @m_impulse))
    @m_impulse += impulse
    PX = impulse * @m_u.x
    PY = impulse * @m_u.y
    bA.m_linearVelocity.x -= bA.m_invMass * PX
    bA.m_linearVelocity.y -= bA.m_invMass * PY
    bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX)
    bB.m_linearVelocity.x += bB.m_invMass * PX
    bB.m_linearVelocity.y += bB.m_invMass * PY
    bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX)
    return

  b2DistanceJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    tMat = undefined
    return true  if @m_frequencyHz > 0.0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    length = Math.sqrt(dX * dX + dY * dY)
    dX /= length
    dY /= length
    C = length - @m_length
    C = b2Math.Clamp(C, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
    impulse = (-@m_mass * C)
    @m_u.Set dX, dY
    PX = impulse * @m_u.x
    PY = impulse * @m_u.y
    bA.m_sweep.c.x -= bA.m_invMass * PX
    bA.m_sweep.c.y -= bA.m_invMass * PY
    bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX)
    bB.m_sweep.c.x += bB.m_invMass * PX
    bB.m_sweep.c.y += bB.m_invMass * PY
    bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    b2Math.Abs(C) < b2Settings.b2_linearSlop

  Box2D.inherit b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2DistanceJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2DistanceJointDef.b2DistanceJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2DistanceJointDef::b2DistanceJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_distanceJoint
    @length = 1.0
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

  b2DistanceJointDef::Initialize = (bA, bB, anchorA, anchorB) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchorA)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchorB)
    dX = anchorB.x - anchorA.x
    dY = anchorB.y - anchorA.y
    @length = Math.sqrt(dX * dX + dY * dY)
    @frequencyHz = 0.0
    @dampingRatio = 0.0
    return

  Box2D.inherit b2FrictionJoint, Box2D.Dynamics.Joints.b2Joint
  b2FrictionJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2FrictionJoint.b2FrictionJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_linearMass = new b2Mat22()
    @m_linearImpulse = new b2Vec2()
    return

  b2FrictionJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  b2FrictionJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  b2FrictionJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_linearImpulse.x, inv_dt * @m_linearImpulse.y)

  b2FrictionJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_angularImpulse

  b2FrictionJoint::SetMaxForce = (force) ->
    force = 0  if force is `undefined`
    @m_maxForce = force
    return

  b2FrictionJoint::GetMaxForce = ->
    @m_maxForce

  b2FrictionJoint::SetMaxTorque = (torque) ->
    torque = 0  if torque is `undefined`
    @m_maxTorque = torque
    return

  b2FrictionJoint::GetMaxTorque = ->
    @m_maxTorque

  b2FrictionJoint::b2FrictionJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_linearMass.SetZero()
    @m_angularMass = 0.0
    @m_linearImpulse.SetZero()
    @m_angularImpulse = 0.0
    @m_maxForce = def.maxForce
    @m_maxTorque = def.maxTorque
    return

  b2FrictionJoint::InitVelocityConstraints = (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    K = new b2Mat22()
    K.col1.x = mA + mB
    K.col2.x = 0.0
    K.col1.y = 0.0
    K.col2.y = mA + mB
    K.col1.x += iA * rAY * rAY
    K.col2.x += (-iA * rAX * rAY)
    K.col1.y += (-iA * rAX * rAY)
    K.col2.y += iA * rAX * rAX
    K.col1.x += iB * rBY * rBY
    K.col2.x += (-iB * rBX * rBY)
    K.col1.y += (-iB * rBX * rBY)
    K.col2.y += iB * rBX * rBX
    K.GetInverse @m_linearMass
    @m_angularMass = iA + iB
    @m_angularMass = 1.0 / @m_angularMass  if @m_angularMass > 0.0
    if step.warmStarting
      @m_linearImpulse.x *= step.dtRatio
      @m_linearImpulse.y *= step.dtRatio
      @m_angularImpulse *= step.dtRatio
      P = @m_linearImpulse
      bA.m_linearVelocity.x -= mA * P.x
      bA.m_linearVelocity.y -= mA * P.y
      bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + @m_angularImpulse)
      bB.m_linearVelocity.x += mB * P.x
      bB.m_linearVelocity.y += mB * P.y
      bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + @m_angularImpulse)
    else
      @m_linearImpulse.SetZero()
      @m_angularImpulse = 0.0
    return

  b2FrictionJoint::SolveVelocityConstraints = (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    vA = bA.m_linearVelocity
    wA = bA.m_angularVelocity
    vB = bB.m_linearVelocity
    wB = bB.m_angularVelocity
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    maxImpulse = 0
    Cdot = wB - wA
    impulse = (-@m_angularMass * Cdot)
    oldImpulse = @m_angularImpulse
    maxImpulse = step.dt * @m_maxTorque
    @m_angularImpulse = b2Math.Clamp(@m_angularImpulse + impulse, (-maxImpulse), maxImpulse)
    impulse = @m_angularImpulse - oldImpulse
    wA -= iA * impulse
    wB += iB * impulse
    CdotX = vB.x - wB * rBY - vA.x + wA * rAY
    CdotY = vB.y + wB * rBX - vA.y - wA * rAX
    impulseV = b2Math.MulMV(@m_linearMass, new b2Vec2((-CdotX), (-CdotY)))
    oldImpulseV = @m_linearImpulse.Copy()
    @m_linearImpulse.Add impulseV
    maxImpulse = step.dt * @m_maxForce
    if @m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse
      @m_linearImpulse.Normalize()
      @m_linearImpulse.Multiply maxImpulse
    impulseV = b2Math.SubtractVV(@m_linearImpulse, oldImpulseV)
    vA.x -= mA * impulseV.x
    vA.y -= mA * impulseV.y
    wA -= iA * (rAX * impulseV.y - rAY * impulseV.x)
    vB.x += mB * impulseV.x
    vB.y += mB * impulseV.y
    wB += iB * (rBX * impulseV.y - rBY * impulseV.x)
    bA.m_angularVelocity = wA
    bB.m_angularVelocity = wB
    return

  b2FrictionJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    true

  Box2D.inherit b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2FrictionJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2FrictionJointDef.b2FrictionJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2FrictionJointDef::b2FrictionJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_frictionJoint
    @maxForce = 0.0
    @maxTorque = 0.0
    return

  b2FrictionJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    return

  Box2D.inherit b2GearJoint, Box2D.Dynamics.Joints.b2Joint
  b2GearJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2GearJoint.b2GearJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_J = new b2Jacobian()
    return

  b2GearJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2GearJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2GearJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_J.linearB.x, inv_dt * @m_impulse * @m_J.linearB.y)

  b2GearJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    tMat = @m_bodyB.m_xf.R
    rX = @m_localAnchor1.x - @m_bodyB.m_sweep.localCenter.x
    rY = @m_localAnchor1.y - @m_bodyB.m_sweep.localCenter.y
    tX = tMat.col1.x * rX + tMat.col2.x * rY
    rY = tMat.col1.y * rX + tMat.col2.y * rY
    rX = tX
    PX = @m_impulse * @m_J.linearB.x
    PY = @m_impulse * @m_J.linearB.y
    inv_dt * (@m_impulse * @m_J.angularB - rX * PY + rY * PX)

  b2GearJoint::GetRatio = ->
    @m_ratio

  b2GearJoint::SetRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_ratio = ratio
    return

  b2GearJoint::b2GearJoint = (def) ->
    @__super.b2Joint.call this, def
    type1 = parseInt(def.joint1.m_type)
    type2 = parseInt(def.joint2.m_type)
    @m_revolute1 = null
    @m_prismatic1 = null
    @m_revolute2 = null
    @m_prismatic2 = null
    coordinate1 = 0
    coordinate2 = 0
    @m_ground1 = def.joint1.GetBodyA()
    @m_bodyA = def.joint1.GetBodyB()
    if type1 is b2Joint.e_revoluteJoint
      @m_revolute1 = ((if def.joint1 instanceof b2RevoluteJoint then def.joint1 else null))
      @m_groundAnchor1.SetV @m_revolute1.m_localAnchor1
      @m_localAnchor1.SetV @m_revolute1.m_localAnchor2
      coordinate1 = @m_revolute1.GetJointAngle()
    else
      @m_prismatic1 = ((if def.joint1 instanceof b2PrismaticJoint then def.joint1 else null))
      @m_groundAnchor1.SetV @m_prismatic1.m_localAnchor1
      @m_localAnchor1.SetV @m_prismatic1.m_localAnchor2
      coordinate1 = @m_prismatic1.GetJointTranslation()
    @m_ground2 = def.joint2.GetBodyA()
    @m_bodyB = def.joint2.GetBodyB()
    if type2 is b2Joint.e_revoluteJoint
      @m_revolute2 = ((if def.joint2 instanceof b2RevoluteJoint then def.joint2 else null))
      @m_groundAnchor2.SetV @m_revolute2.m_localAnchor1
      @m_localAnchor2.SetV @m_revolute2.m_localAnchor2
      coordinate2 = @m_revolute2.GetJointAngle()
    else
      @m_prismatic2 = ((if def.joint2 instanceof b2PrismaticJoint then def.joint2 else null))
      @m_groundAnchor2.SetV @m_prismatic2.m_localAnchor1
      @m_localAnchor2.SetV @m_prismatic2.m_localAnchor2
      coordinate2 = @m_prismatic2.GetJointTranslation()
    @m_ratio = def.ratio
    @m_constant = coordinate1 + @m_ratio * coordinate2
    @m_impulse = 0.0
    return

  b2GearJoint::InitVelocityConstraints = (step) ->
    g1 = @m_ground1
    g2 = @m_ground2
    bA = @m_bodyA
    bB = @m_bodyB
    ugX = 0
    ugY = 0
    rX = 0
    rY = 0
    tMat = undefined
    tVec = undefined
    crug = 0
    tX = 0
    K = 0.0
    @m_J.SetZero()
    if @m_revolute1
      @m_J.angularA = (-1.0)
      K += bA.m_invI
    else
      tMat = g1.m_xf.R
      tVec = @m_prismatic1.m_localXAxis1
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tMat = bA.m_xf.R
      rX = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      rY = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = tMat.col1.x * rX + tMat.col2.x * rY
      rY = tMat.col1.y * rX + tMat.col2.y * rY
      rX = tX
      crug = rX * ugY - rY * ugX
      @m_J.linearA.Set (-ugX), (-ugY)
      @m_J.angularA = (-crug)
      K += bA.m_invMass + bA.m_invI * crug * crug
    if @m_revolute2
      @m_J.angularB = (-@m_ratio)
      K += @m_ratio * @m_ratio * bB.m_invI
    else
      tMat = g2.m_xf.R
      tVec = @m_prismatic2.m_localXAxis1
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
      tMat = bB.m_xf.R
      rX = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      rY = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = tMat.col1.x * rX + tMat.col2.x * rY
      rY = tMat.col1.y * rX + tMat.col2.y * rY
      rX = tX
      crug = rX * ugY - rY * ugX
      @m_J.linearB.Set (-@m_ratio * ugX), (-@m_ratio * ugY)
      @m_J.angularB = (-@m_ratio * crug)
      K += @m_ratio * @m_ratio * (bB.m_invMass + bB.m_invI * crug * crug)
    @m_mass = (if K > 0.0 then 1.0 / K else 0.0)
    if step.warmStarting
      bA.m_linearVelocity.x += bA.m_invMass * @m_impulse * @m_J.linearA.x
      bA.m_linearVelocity.y += bA.m_invMass * @m_impulse * @m_J.linearA.y
      bA.m_angularVelocity += bA.m_invI * @m_impulse * @m_J.angularA
      bB.m_linearVelocity.x += bB.m_invMass * @m_impulse * @m_J.linearB.x
      bB.m_linearVelocity.y += bB.m_invMass * @m_impulse * @m_J.linearB.y
      bB.m_angularVelocity += bB.m_invI * @m_impulse * @m_J.angularB
    else
      @m_impulse = 0.0
    return

  b2GearJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    Cdot = @m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity)
    impulse = (-@m_mass * Cdot)
    @m_impulse += impulse
    bA.m_linearVelocity.x += bA.m_invMass * impulse * @m_J.linearA.x
    bA.m_linearVelocity.y += bA.m_invMass * impulse * @m_J.linearA.y
    bA.m_angularVelocity += bA.m_invI * impulse * @m_J.angularA
    bB.m_linearVelocity.x += bB.m_invMass * impulse * @m_J.linearB.x
    bB.m_linearVelocity.y += bB.m_invMass * impulse * @m_J.linearB.y
    bB.m_angularVelocity += bB.m_invI * impulse * @m_J.angularB
    return

  b2GearJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    linearError = 0.0
    bA = @m_bodyA
    bB = @m_bodyB
    coordinate1 = 0
    coordinate2 = 0
    if @m_revolute1
      coordinate1 = @m_revolute1.GetJointAngle()
    else
      coordinate1 = @m_prismatic1.GetJointTranslation()
    if @m_revolute2
      coordinate2 = @m_revolute2.GetJointAngle()
    else
      coordinate2 = @m_prismatic2.GetJointTranslation()
    C = @m_constant - (coordinate1 + @m_ratio * coordinate2)
    impulse = (-@m_mass * C)
    bA.m_sweep.c.x += bA.m_invMass * impulse * @m_J.linearA.x
    bA.m_sweep.c.y += bA.m_invMass * impulse * @m_J.linearA.y
    bA.m_sweep.a += bA.m_invI * impulse * @m_J.angularA
    bB.m_sweep.c.x += bB.m_invMass * impulse * @m_J.linearB.x
    bB.m_sweep.c.y += bB.m_invMass * impulse * @m_J.linearB.y
    bB.m_sweep.a += bB.m_invI * impulse * @m_J.angularB
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError < b2Settings.b2_linearSlop

  Box2D.inherit b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2GearJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2GearJointDef.b2GearJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    return

  b2GearJointDef::b2GearJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_gearJoint
    @joint1 = null
    @joint2 = null
    @ratio = 1.0
    return

  b2Jacobian.b2Jacobian = ->
    @linearA = new b2Vec2()
    @linearB = new b2Vec2()
    return

  b2Jacobian::SetZero = ->
    @linearA.SetZero()
    @angularA = 0.0
    @linearB.SetZero()
    @angularB = 0.0
    return

  b2Jacobian::Set = (x1, a1, x2, a2) ->
    a1 = 0  if a1 is `undefined`
    a2 = 0  if a2 is `undefined`
    @linearA.SetV x1
    @angularA = a1
    @linearB.SetV x2
    @angularB = a2
    return

  b2Jacobian::Compute = (x1, a1, x2, a2) ->
    a1 = 0  if a1 is `undefined`
    a2 = 0  if a2 is `undefined`
    (@linearA.x * x1.x + @linearA.y * x1.y) + @angularA * a1 + (@linearB.x * x2.x + @linearB.y * x2.y) + @angularB * a2

  b2Joint.b2Joint = ->
    @m_edgeA = new b2JointEdge()
    @m_edgeB = new b2JointEdge()
    @m_localCenterA = new b2Vec2()
    @m_localCenterB = new b2Vec2()
    return

  b2Joint::GetType = ->
    @m_type

  b2Joint::GetAnchorA = ->
    null

  b2Joint::GetAnchorB = ->
    null

  b2Joint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    null

  b2Joint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2Joint::GetBodyA = ->
    @m_bodyA

  b2Joint::GetBodyB = ->
    @m_bodyB

  b2Joint::GetNext = ->
    @m_next

  b2Joint::GetUserData = ->
    @m_userData

  b2Joint::SetUserData = (data) ->
    @m_userData = data
    return

  b2Joint::IsActive = ->
    @m_bodyA.IsActive() and @m_bodyB.IsActive()

  b2Joint.Create = (def, allocator) ->
    joint = null
    switch def.type
      when b2Joint.e_distanceJoint
        joint = new b2DistanceJoint(((if def instanceof b2DistanceJointDef then def else null)))
      when b2Joint.e_mouseJoint
        joint = new b2MouseJoint(((if def instanceof b2MouseJointDef then def else null)))
      when b2Joint.e_prismaticJoint
        joint = new b2PrismaticJoint(((if def instanceof b2PrismaticJointDef then def else null)))
      when b2Joint.e_revoluteJoint
        joint = new b2RevoluteJoint(((if def instanceof b2RevoluteJointDef then def else null)))
      when b2Joint.e_pulleyJoint
        joint = new b2PulleyJoint(((if def instanceof b2PulleyJointDef then def else null)))
      when b2Joint.e_gearJoint
        joint = new b2GearJoint(((if def instanceof b2GearJointDef then def else null)))
      when b2Joint.e_lineJoint
        joint = new b2LineJoint(((if def instanceof b2LineJointDef then def else null)))
      when b2Joint.e_weldJoint
        joint = new b2WeldJoint(((if def instanceof b2WeldJointDef then def else null)))
      when b2Joint.e_frictionJoint
        joint = new b2FrictionJoint(((if def instanceof b2FrictionJointDef then def else null)))
      else
    joint

  b2Joint.Destroy = (joint, allocator) ->

  b2Joint::b2Joint = (def) ->
    b2Settings.b2Assert def.bodyA isnt def.bodyB
    @m_type = def.type
    @m_prev = null
    @m_next = null
    @m_bodyA = def.bodyA
    @m_bodyB = def.bodyB
    @m_collideConnected = def.collideConnected
    @m_islandFlag = false
    @m_userData = def.userData
    return

  b2Joint::InitVelocityConstraints = (step) ->

  b2Joint::SolveVelocityConstraints = (step) ->

  b2Joint::FinalizeVelocityConstraints = ->

  b2Joint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    false

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0
    Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1
    Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2
    Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3
    Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4
    Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5
    Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6
    Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7
    Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8
    Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9
    Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0
    Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1
    Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2
    Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3
    return

  b2JointDef.b2JointDef = ->

  b2JointDef::b2JointDef = ->
    @type = b2Joint.e_unknownJoint
    @userData = null
    @bodyA = null
    @bodyB = null
    @collideConnected = false
    return

  b2JointEdge.b2JointEdge = ->

  Box2D.inherit b2LineJoint, Box2D.Dynamics.Joints.b2Joint
  b2LineJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2LineJoint.b2LineJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_localXAxis1 = new b2Vec2()
    @m_localYAxis1 = new b2Vec2()
    @m_axis = new b2Vec2()
    @m_perp = new b2Vec2()
    @m_K = new b2Mat22()
    @m_impulse = new b2Vec2()
    return

  b2LineJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2LineJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2LineJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * (@m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.y) * @m_axis.x), inv_dt * (@m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.y) * @m_axis.y))

  b2LineJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.y

  b2LineJoint::GetJointTranslation = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    p1 = bA.GetWorldPoint(@m_localAnchor1)
    p2 = bB.GetWorldPoint(@m_localAnchor2)
    dX = p2.x - p1.x
    dY = p2.y - p1.y
    axis = bA.GetWorldVector(@m_localXAxis1)
    translation = axis.x * dX + axis.y * dY
    translation

  b2LineJoint::GetJointSpeed = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    dX = p2X - p1X
    dY = p2Y - p1Y
    axis = bA.GetWorldVector(@m_localXAxis1)
    v1 = bA.m_linearVelocity
    v2 = bB.m_linearVelocity
    w1 = bA.m_angularVelocity
    w2 = bB.m_angularVelocity
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)))
    speed

  b2LineJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2LineJoint::EnableLimit = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableLimit = flag
    return

  b2LineJoint::GetLowerLimit = ->
    @m_lowerTranslation

  b2LineJoint::GetUpperLimit = ->
    @m_upperTranslation

  b2LineJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_lowerTranslation = lower
    @m_upperTranslation = upper
    return

  b2LineJoint::IsMotorEnabled = ->
    @m_enableMotor

  b2LineJoint::EnableMotor = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor = flag
    return

  b2LineJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2LineJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2LineJoint::SetMaxMotorForce = (force) ->
    force = 0  if force is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_maxMotorForce = force
    return

  b2LineJoint::GetMaxMotorForce = ->
    @m_maxMotorForce

  b2LineJoint::GetMotorForce = ->
    @m_motorImpulse

  b2LineJoint::b2LineJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_localXAxis1.SetV def.localAxisA
    @m_localYAxis1.x = (-@m_localXAxis1.y)
    @m_localYAxis1.y = @m_localXAxis1.x
    @m_impulse.SetZero()
    @m_motorMass = 0.0
    @m_motorImpulse = 0.0
    @m_lowerTranslation = def.lowerTranslation
    @m_upperTranslation = def.upperTranslation
    @m_maxMotorForce = def.maxMotorForce
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    @m_axis.SetZero()
    @m_perp.SetZero()
    return

  b2LineJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    @m_localCenterA.SetV bA.GetLocalCenter()
    @m_localCenterB.SetV bB.GetLocalCenter()
    xf1 = bA.GetTransform()
    xf2 = bB.GetTransform()
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @m_invMassA = bA.m_invMass
    @m_invMassB = bB.m_invMass
    @m_invIA = bA.m_invI
    @m_invIB = bB.m_invI
    @m_axis.SetV b2Math.MulMV(xf1.R, @m_localXAxis1)
    @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
    @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
    @m_motorMass = @m_invMassA + @m_invMassB + @m_invIA * @m_a1 * @m_a1 + @m_invIB * @m_a2 * @m_a2
    @m_motorMass = (if @m_motorMass > Number.MIN_VALUE then 1.0 / @m_motorMass else 0.0)
    @m_perp.SetV b2Math.MulMV(xf1.R, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    m1 = @m_invMassA
    m2 = @m_invMassB
    i1 = @m_invIA
    i2 = @m_invIB
    @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
    @m_K.col1.y = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
    @m_K.col2.x = @m_K.col1.y
    @m_K.col2.y = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
    if @m_enableLimit
      jointTransition = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointTransition <= @m_lowerTranslation
        unless @m_limitState is b2Joint.e_atLowerLimit
          @m_limitState = b2Joint.e_atLowerLimit
          @m_impulse.y = 0.0
      else if jointTransition >= @m_upperTranslation
        unless @m_limitState is b2Joint.e_atUpperLimit
          @m_limitState = b2Joint.e_atUpperLimit
          @m_impulse.y = 0.0
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.y = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.y) * @m_axis.x
      PY = @m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.y) * @m_axis.y
      L1 = @m_impulse.x * @m_s1 + (@m_motorImpulse + @m_impulse.y) * @m_a1
      L2 = @m_impulse.x * @m_s2 + (@m_motorImpulse + @m_impulse.y) * @m_a2
      bA.m_linearVelocity.x -= @m_invMassA * PX
      bA.m_linearVelocity.y -= @m_invMassA * PY
      bA.m_angularVelocity -= @m_invIA * L1
      bB.m_linearVelocity.x += @m_invMassB * PX
      bB.m_linearVelocity.y += @m_invMassB * PY
      bB.m_angularVelocity += @m_invIB * L2
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2LineJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    PX = 0
    PY = 0
    L1 = 0
    L2 = 0
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      impulse = @m_motorMass * (@m_motorSpeed - Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorForce
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      PX = impulse * @m_axis.x
      PY = impulse * @m_axis.y
      L1 = impulse * @m_a1
      L2 = impulse * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    Cdot1 = @m_perp.x * (v2.x - v1.x) + @m_perp.y * (v2.y - v1.y) + @m_s2 * w2 - @m_s1 * w1
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      Cdot2 = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      f1 = @m_impulse.Copy()
      df = @m_K.Solve(new b2Vec2(), (-Cdot1), (-Cdot2))
      @m_impulse.Add df
      if @m_limitState is b2Joint.e_atLowerLimit
        @m_impulse.y = b2Math.Max(@m_impulse.y, 0.0)
      else @m_impulse.y = b2Math.Min(@m_impulse.y, 0.0)  if @m_limitState is b2Joint.e_atUpperLimit
      b = (-Cdot1) - (@m_impulse.y - f1.y) * @m_K.col2.x
      f2r = 0
      unless @m_K.col1.x is 0.0
        f2r = b / @m_K.col1.x + f1.x
      else
        f2r = f1.x
      @m_impulse.x = f2r
      df.x = @m_impulse.x - f1.x
      df.y = @m_impulse.y - f1.y
      PX = df.x * @m_perp.x + df.y * @m_axis.x
      PY = df.x * @m_perp.y + df.y * @m_axis.y
      L1 = df.x * @m_s1 + df.y * @m_a1
      L2 = df.x * @m_s2 + df.y * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    else
      df2 = 0
      unless @m_K.col1.x is 0.0
        df2 = (-Cdot1) / @m_K.col1.x
      else
        df2 = 0.0
      @m_impulse.x += df2
      PX = df2 * @m_perp.x
      PY = df2 * @m_perp.y
      L1 = df2 * @m_s1
      L2 = df2 * @m_s2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2LineJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    limitC = 0
    oldLimitImpulse = 0
    bA = @m_bodyA
    bB = @m_bodyB
    c1 = bA.m_sweep.c
    a1 = bA.m_sweep.a
    c2 = bB.m_sweep.c
    a2 = bB.m_sweep.a
    tMat = undefined
    tX = 0
    m1 = 0
    m2 = 0
    i1 = 0
    i2 = 0
    linearError = 0.0
    angularError = 0.0
    active = false
    C2 = 0.0
    R1 = b2Mat22.FromAngle(a1)
    R2 = b2Mat22.FromAngle(a2)
    tMat = R1
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = R2
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = c2.x + r2X - c1.x - r1X
    dY = c2.y + r2Y - c1.y - r1Y
    if @m_enableLimit
      @m_axis = b2Math.MulMV(R1, @m_localXAxis1)
      @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
      @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
      translation = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
        linearError = b2Math.Abs(translation)
        active = true
      else if translation <= @m_lowerTranslation
        C2 = b2Math.Clamp(translation - @m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
        linearError = @m_lowerTranslation - translation
        active = true
      else if translation >= @m_upperTranslation
        C2 = b2Math.Clamp(translation - @m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection)
        linearError = translation - @m_upperTranslation
        active = true
    @m_perp = b2Math.MulMV(R1, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    impulse = new b2Vec2()
    C1 = @m_perp.x * dX + @m_perp.y * dY
    linearError = b2Math.Max(linearError, b2Math.Abs(C1))
    angularError = 0.0
    if active
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      @m_K.col1.y = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
      @m_K.col2.x = @m_K.col1.y
      @m_K.col2.y = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
      @m_K.Solve impulse, (-C1), (-C2)
    else
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      k11 = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      impulse1 = 0
      unless k11 is 0.0
        impulse1 = (-C1) / k11
      else
        impulse1 = 0.0
      impulse.x = impulse1
      impulse.y = 0.0
    PX = impulse.x * @m_perp.x + impulse.y * @m_axis.x
    PY = impulse.x * @m_perp.y + impulse.y * @m_axis.y
    L1 = impulse.x * @m_s1 + impulse.y * @m_a1
    L2 = impulse.x * @m_s2 + impulse.y * @m_a2
    c1.x -= @m_invMassA * PX
    c1.y -= @m_invMassA * PY
    a1 -= @m_invIA * L1
    c2.x += @m_invMassB * PX
    c2.y += @m_invMassB * PY
    a2 += @m_invIB * L2
    bA.m_sweep.a = a1
    bB.m_sweep.a = a2
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.inherit b2LineJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2LineJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2LineJointDef.b2LineJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAxisA = new b2Vec2()
    return

  b2LineJointDef::b2LineJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_lineJoint
    @localAxisA.Set 1.0, 0.0
    @enableLimit = false
    @lowerTranslation = 0.0
    @upperTranslation = 0.0
    @enableMotor = false
    @maxMotorForce = 0.0
    @motorSpeed = 0.0
    return

  b2LineJointDef::Initialize = (bA, bB, anchor, axis) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @localAxisA = @bodyA.GetLocalVector(axis)
    return

  Box2D.inherit b2MouseJoint, Box2D.Dynamics.Joints.b2Joint
  b2MouseJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2MouseJoint.b2MouseJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @K = new b2Mat22()
    @K1 = new b2Mat22()
    @K2 = new b2Mat22()
    @m_localAnchor = new b2Vec2()
    @m_target = new b2Vec2()
    @m_impulse = new b2Vec2()
    @m_mass = new b2Mat22()
    @m_C = new b2Vec2()
    return

  b2MouseJoint::GetAnchorA = ->
    @m_target

  b2MouseJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor

  b2MouseJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2MouseJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2MouseJoint::GetTarget = ->
    @m_target

  b2MouseJoint::SetTarget = (target) ->
    @m_bodyB.SetAwake true  if @m_bodyB.IsAwake() is false
    @m_target = target
    return

  b2MouseJoint::GetMaxForce = ->
    @m_maxForce

  b2MouseJoint::SetMaxForce = (maxForce) ->
    maxForce = 0  if maxForce is `undefined`
    @m_maxForce = maxForce
    return

  b2MouseJoint::GetFrequency = ->
    @m_frequencyHz

  b2MouseJoint::SetFrequency = (hz) ->
    hz = 0  if hz is `undefined`
    @m_frequencyHz = hz
    return

  b2MouseJoint::GetDampingRatio = ->
    @m_dampingRatio

  b2MouseJoint::SetDampingRatio = (ratio) ->
    ratio = 0  if ratio is `undefined`
    @m_dampingRatio = ratio
    return

  b2MouseJoint::b2MouseJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_target.SetV def.target
    tX = @m_target.x - @m_bodyB.m_xf.position.x
    tY = @m_target.y - @m_bodyB.m_xf.position.y
    tMat = @m_bodyB.m_xf.R
    @m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y)
    @m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y)
    @m_maxForce = def.maxForce
    @m_impulse.SetZero()
    @m_frequencyHz = def.frequencyHz
    @m_dampingRatio = def.dampingRatio
    @m_beta = 0.0
    @m_gamma = 0.0
    return

  b2MouseJoint::InitVelocityConstraints = (step) ->
    b = @m_bodyB
    mass = b.GetMass()
    omega = 2.0 * Math.PI * @m_frequencyHz
    d = 2.0 * mass * @m_dampingRatio * omega
    k = mass * omega * omega
    @m_gamma = step.dt * (d + step.dt * k)
    @m_gamma = (if @m_gamma isnt 0 then 1 / @m_gamma else 0.0)
    @m_beta = step.dt * k * @m_gamma
    tMat = undefined
    tMat = b.m_xf.R
    rX = @m_localAnchor.x - b.m_sweep.localCenter.x
    rY = @m_localAnchor.y - b.m_sweep.localCenter.y
    tX = (tMat.col1.x * rX + tMat.col2.x * rY)
    rY = (tMat.col1.y * rX + tMat.col2.y * rY)
    rX = tX
    invMass = b.m_invMass
    invI = b.m_invI
    @K1.col1.x = invMass
    @K1.col2.x = 0.0
    @K1.col1.y = 0.0
    @K1.col2.y = invMass
    @K2.col1.x = invI * rY * rY
    @K2.col2.x = (-invI * rX * rY)
    @K2.col1.y = (-invI * rX * rY)
    @K2.col2.y = invI * rX * rX
    @K.SetM @K1
    @K.AddM @K2
    @K.col1.x += @m_gamma
    @K.col2.y += @m_gamma
    @K.GetInverse @m_mass
    @m_C.x = b.m_sweep.c.x + rX - @m_target.x
    @m_C.y = b.m_sweep.c.y + rY - @m_target.y
    b.m_angularVelocity *= 0.98
    @m_impulse.x *= step.dtRatio
    @m_impulse.y *= step.dtRatio
    b.m_linearVelocity.x += invMass * @m_impulse.x
    b.m_linearVelocity.y += invMass * @m_impulse.y
    b.m_angularVelocity += invI * (rX * @m_impulse.y - rY * @m_impulse.x)
    return

  b2MouseJoint::SolveVelocityConstraints = (step) ->
    b = @m_bodyB
    tMat = undefined
    tX = 0
    tY = 0
    tMat = b.m_xf.R
    rX = @m_localAnchor.x - b.m_sweep.localCenter.x
    rY = @m_localAnchor.y - b.m_sweep.localCenter.y
    tX = (tMat.col1.x * rX + tMat.col2.x * rY)
    rY = (tMat.col1.y * rX + tMat.col2.y * rY)
    rX = tX
    CdotX = b.m_linearVelocity.x + (-b.m_angularVelocity * rY)
    CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX)
    tMat = @m_mass
    tX = CdotX + @m_beta * @m_C.x + @m_gamma * @m_impulse.x
    tY = CdotY + @m_beta * @m_C.y + @m_gamma * @m_impulse.y
    impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY))
    impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY))
    oldImpulseX = @m_impulse.x
    oldImpulseY = @m_impulse.y
    @m_impulse.x += impulseX
    @m_impulse.y += impulseY
    maxImpulse = step.dt * @m_maxForce
    @m_impulse.Multiply maxImpulse / @m_impulse.Length()  if @m_impulse.LengthSquared() > maxImpulse * maxImpulse
    impulseX = @m_impulse.x - oldImpulseX
    impulseY = @m_impulse.y - oldImpulseY
    b.m_linearVelocity.x += b.m_invMass * impulseX
    b.m_linearVelocity.y += b.m_invMass * impulseY
    b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX)
    return

  b2MouseJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    true

  Box2D.inherit b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2MouseJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2MouseJointDef.b2MouseJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @target = new b2Vec2()
    return

  b2MouseJointDef::b2MouseJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_mouseJoint
    @maxForce = 0.0
    @frequencyHz = 5.0
    @dampingRatio = 0.7
    return

  Box2D.inherit b2PrismaticJoint, Box2D.Dynamics.Joints.b2Joint
  b2PrismaticJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2PrismaticJoint.b2PrismaticJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_localXAxis1 = new b2Vec2()
    @m_localYAxis1 = new b2Vec2()
    @m_axis = new b2Vec2()
    @m_perp = new b2Vec2()
    @m_K = new b2Mat33()
    @m_impulse = new b2Vec3()
    return

  b2PrismaticJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2PrismaticJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2PrismaticJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * (@m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.z) * @m_axis.x), inv_dt * (@m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.z) * @m_axis.y))

  b2PrismaticJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.y

  b2PrismaticJoint::GetJointTranslation = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    p1 = bA.GetWorldPoint(@m_localAnchor1)
    p2 = bB.GetWorldPoint(@m_localAnchor2)
    dX = p2.x - p1.x
    dY = p2.y - p1.y
    axis = bA.GetWorldVector(@m_localXAxis1)
    translation = axis.x * dX + axis.y * dY
    translation

  b2PrismaticJoint::GetJointSpeed = ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    dX = p2X - p1X
    dY = p2Y - p1Y
    axis = bA.GetWorldVector(@m_localXAxis1)
    v1 = bA.m_linearVelocity
    v2 = bB.m_linearVelocity
    w1 = bA.m_angularVelocity
    w2 = bB.m_angularVelocity
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)))
    speed

  b2PrismaticJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2PrismaticJoint::EnableLimit = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableLimit = flag
    return

  b2PrismaticJoint::GetLowerLimit = ->
    @m_lowerTranslation

  b2PrismaticJoint::GetUpperLimit = ->
    @m_upperTranslation

  b2PrismaticJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_lowerTranslation = lower
    @m_upperTranslation = upper
    return

  b2PrismaticJoint::IsMotorEnabled = ->
    @m_enableMotor

  b2PrismaticJoint::EnableMotor = (flag) ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor = flag
    return

  b2PrismaticJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2PrismaticJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2PrismaticJoint::SetMaxMotorForce = (force) ->
    force = 0  if force is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_maxMotorForce = force
    return

  b2PrismaticJoint::GetMotorForce = ->
    @m_motorImpulse

  b2PrismaticJoint::b2PrismaticJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_localXAxis1.SetV def.localAxisA
    @m_localYAxis1.x = (-@m_localXAxis1.y)
    @m_localYAxis1.y = @m_localXAxis1.x
    @m_refAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_motorMass = 0.0
    @m_motorImpulse = 0.0
    @m_lowerTranslation = def.lowerTranslation
    @m_upperTranslation = def.upperTranslation
    @m_maxMotorForce = def.maxMotorForce
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    @m_axis.SetZero()
    @m_perp.SetZero()
    return

  b2PrismaticJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    @m_localCenterA.SetV bA.GetLocalCenter()
    @m_localCenterB.SetV bB.GetLocalCenter()
    xf1 = bA.GetTransform()
    xf2 = bB.GetTransform()
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @m_invMassA = bA.m_invMass
    @m_invMassB = bB.m_invMass
    @m_invIA = bA.m_invI
    @m_invIB = bB.m_invI
    @m_axis.SetV b2Math.MulMV(xf1.R, @m_localXAxis1)
    @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
    @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
    @m_motorMass = @m_invMassA + @m_invMassB + @m_invIA * @m_a1 * @m_a1 + @m_invIB * @m_a2 * @m_a2
    @m_motorMass = 1.0 / @m_motorMass  if @m_motorMass > Number.MIN_VALUE
    @m_perp.SetV b2Math.MulMV(xf1.R, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    m1 = @m_invMassA
    m2 = @m_invMassB
    i1 = @m_invIA
    i2 = @m_invIB
    @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
    @m_K.col1.y = i1 * @m_s1 + i2 * @m_s2
    @m_K.col1.z = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
    @m_K.col2.x = @m_K.col1.y
    @m_K.col2.y = i1 + i2
    @m_K.col2.z = i1 * @m_a1 + i2 * @m_a2
    @m_K.col3.x = @m_K.col1.z
    @m_K.col3.y = @m_K.col2.z
    @m_K.col3.z = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
    if @m_enableLimit
      jointTransition = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointTransition <= @m_lowerTranslation
        unless @m_limitState is b2Joint.e_atLowerLimit
          @m_limitState = b2Joint.e_atLowerLimit
          @m_impulse.z = 0.0
      else if jointTransition >= @m_upperTranslation
        unless @m_limitState is b2Joint.e_atUpperLimit
          @m_limitState = b2Joint.e_atUpperLimit
          @m_impulse.z = 0.0
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.z = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x * @m_perp.x + (@m_motorImpulse + @m_impulse.z) * @m_axis.x
      PY = @m_impulse.x * @m_perp.y + (@m_motorImpulse + @m_impulse.z) * @m_axis.y
      L1 = @m_impulse.x * @m_s1 + @m_impulse.y + (@m_motorImpulse + @m_impulse.z) * @m_a1
      L2 = @m_impulse.x * @m_s2 + @m_impulse.y + (@m_motorImpulse + @m_impulse.z) * @m_a2
      bA.m_linearVelocity.x -= @m_invMassA * PX
      bA.m_linearVelocity.y -= @m_invMassA * PY
      bA.m_angularVelocity -= @m_invIA * L1
      bB.m_linearVelocity.x += @m_invMassB * PX
      bB.m_linearVelocity.y += @m_invMassB * PY
      bB.m_angularVelocity += @m_invIB * L2
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2PrismaticJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    PX = 0
    PY = 0
    L1 = 0
    L2 = 0
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      impulse = @m_motorMass * (@m_motorSpeed - Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorForce
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      PX = impulse * @m_axis.x
      PY = impulse * @m_axis.y
      L1 = impulse * @m_a1
      L2 = impulse * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    Cdot1X = @m_perp.x * (v2.x - v1.x) + @m_perp.y * (v2.y - v1.y) + @m_s2 * w2 - @m_s1 * w1
    Cdot1Y = w2 - w1
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      Cdot2 = @m_axis.x * (v2.x - v1.x) + @m_axis.y * (v2.y - v1.y) + @m_a2 * w2 - @m_a1 * w1
      f1 = @m_impulse.Copy()
      df = @m_K.Solve33(new b2Vec3(), (-Cdot1X), (-Cdot1Y), (-Cdot2))
      @m_impulse.Add df
      if @m_limitState is b2Joint.e_atLowerLimit
        @m_impulse.z = b2Math.Max(@m_impulse.z, 0.0)
      else @m_impulse.z = b2Math.Min(@m_impulse.z, 0.0)  if @m_limitState is b2Joint.e_atUpperLimit
      bX = (-Cdot1X) - (@m_impulse.z - f1.z) * @m_K.col3.x
      bY = (-Cdot1Y) - (@m_impulse.z - f1.z) * @m_K.col3.y
      f2r = @m_K.Solve22(new b2Vec2(), bX, bY)
      f2r.x += f1.x
      f2r.y += f1.y
      @m_impulse.x = f2r.x
      @m_impulse.y = f2r.y
      df.x = @m_impulse.x - f1.x
      df.y = @m_impulse.y - f1.y
      df.z = @m_impulse.z - f1.z
      PX = df.x * @m_perp.x + df.z * @m_axis.x
      PY = df.x * @m_perp.y + df.z * @m_axis.y
      L1 = df.x * @m_s1 + df.y + df.z * @m_a1
      L2 = df.x * @m_s2 + df.y + df.z * @m_a2
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    else
      df2 = @m_K.Solve22(new b2Vec2(), (-Cdot1X), (-Cdot1Y))
      @m_impulse.x += df2.x
      @m_impulse.y += df2.y
      PX = df2.x * @m_perp.x
      PY = df2.x * @m_perp.y
      L1 = df2.x * @m_s1 + df2.y
      L2 = df2.x * @m_s2 + df2.y
      v1.x -= @m_invMassA * PX
      v1.y -= @m_invMassA * PY
      w1 -= @m_invIA * L1
      v2.x += @m_invMassB * PX
      v2.y += @m_invMassB * PY
      w2 += @m_invIB * L2
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2PrismaticJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    limitC = 0
    oldLimitImpulse = 0
    bA = @m_bodyA
    bB = @m_bodyB
    c1 = bA.m_sweep.c
    a1 = bA.m_sweep.a
    c2 = bB.m_sweep.c
    a2 = bB.m_sweep.a
    tMat = undefined
    tX = 0
    m1 = 0
    m2 = 0
    i1 = 0
    i2 = 0
    linearError = 0.0
    angularError = 0.0
    active = false
    C2 = 0.0
    R1 = b2Mat22.FromAngle(a1)
    R2 = b2Mat22.FromAngle(a2)
    tMat = R1
    r1X = @m_localAnchor1.x - @m_localCenterA.x
    r1Y = @m_localAnchor1.y - @m_localCenterA.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = R2
    r2X = @m_localAnchor2.x - @m_localCenterB.x
    r2Y = @m_localAnchor2.y - @m_localCenterB.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    dX = c2.x + r2X - c1.x - r1X
    dY = c2.y + r2Y - c1.y - r1Y
    if @m_enableLimit
      @m_axis = b2Math.MulMV(R1, @m_localXAxis1)
      @m_a1 = (dX + r1X) * @m_axis.y - (dY + r1Y) * @m_axis.x
      @m_a2 = r2X * @m_axis.y - r2Y * @m_axis.x
      translation = @m_axis.x * dX + @m_axis.y * dY
      if b2Math.Abs(@m_upperTranslation - @m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop
        C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection)
        linearError = b2Math.Abs(translation)
        active = true
      else if translation <= @m_lowerTranslation
        C2 = b2Math.Clamp(translation - @m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
        linearError = @m_lowerTranslation - translation
        active = true
      else if translation >= @m_upperTranslation
        C2 = b2Math.Clamp(translation - @m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection)
        linearError = translation - @m_upperTranslation
        active = true
    @m_perp = b2Math.MulMV(R1, @m_localYAxis1)
    @m_s1 = (dX + r1X) * @m_perp.y - (dY + r1Y) * @m_perp.x
    @m_s2 = r2X * @m_perp.y - r2Y * @m_perp.x
    impulse = new b2Vec3()
    C1X = @m_perp.x * dX + @m_perp.y * dY
    C1Y = a2 - a1 - @m_refAngle
    linearError = b2Math.Max(linearError, b2Math.Abs(C1X))
    angularError = b2Math.Abs(C1Y)
    if active
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      @m_K.col1.x = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      @m_K.col1.y = i1 * @m_s1 + i2 * @m_s2
      @m_K.col1.z = i1 * @m_s1 * @m_a1 + i2 * @m_s2 * @m_a2
      @m_K.col2.x = @m_K.col1.y
      @m_K.col2.y = i1 + i2
      @m_K.col2.z = i1 * @m_a1 + i2 * @m_a2
      @m_K.col3.x = @m_K.col1.z
      @m_K.col3.y = @m_K.col2.z
      @m_K.col3.z = m1 + m2 + i1 * @m_a1 * @m_a1 + i2 * @m_a2 * @m_a2
      @m_K.Solve33 impulse, (-C1X), (-C1Y), (-C2)
    else
      m1 = @m_invMassA
      m2 = @m_invMassB
      i1 = @m_invIA
      i2 = @m_invIB
      k11 = m1 + m2 + i1 * @m_s1 * @m_s1 + i2 * @m_s2 * @m_s2
      k12 = i1 * @m_s1 + i2 * @m_s2
      k22 = i1 + i2
      @m_K.col1.Set k11, k12, 0.0
      @m_K.col2.Set k12, k22, 0.0
      impulse1 = @m_K.Solve22(new b2Vec2(), (-C1X), (-C1Y))
      impulse.x = impulse1.x
      impulse.y = impulse1.y
      impulse.z = 0.0
    PX = impulse.x * @m_perp.x + impulse.z * @m_axis.x
    PY = impulse.x * @m_perp.y + impulse.z * @m_axis.y
    L1 = impulse.x * @m_s1 + impulse.y + impulse.z * @m_a1
    L2 = impulse.x * @m_s2 + impulse.y + impulse.z * @m_a2
    c1.x -= @m_invMassA * PX
    c1.y -= @m_invMassA * PY
    a1 -= @m_invIA * L1
    c2.x += @m_invMassB * PX
    c2.y += @m_invMassB * PY
    a2 += @m_invIB * L2
    bA.m_sweep.a = a1
    bB.m_sweep.a = a2
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    linearError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.inherit b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2PrismaticJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2PrismaticJointDef.b2PrismaticJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    @localAxisA = new b2Vec2()
    return

  b2PrismaticJointDef::b2PrismaticJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_prismaticJoint
    @localAxisA.Set 1.0, 0.0
    @referenceAngle = 0.0
    @enableLimit = false
    @lowerTranslation = 0.0
    @upperTranslation = 0.0
    @enableMotor = false
    @maxMotorForce = 0.0
    @motorSpeed = 0.0
    return

  b2PrismaticJointDef::Initialize = (bA, bB, anchor, axis) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @localAxisA = @bodyA.GetLocalVector(axis)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  Box2D.inherit b2PulleyJoint, Box2D.Dynamics.Joints.b2Joint
  b2PulleyJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2PulleyJoint.b2PulleyJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_groundAnchor1 = new b2Vec2()
    @m_groundAnchor2 = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_u1 = new b2Vec2()
    @m_u2 = new b2Vec2()
    return

  b2PulleyJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2PulleyJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2PulleyJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse * @m_u2.x, inv_dt * @m_impulse * @m_u2.y)

  b2PulleyJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    0.0

  b2PulleyJoint::GetGroundAnchorA = ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor1
    a

  b2PulleyJoint::GetGroundAnchorB = ->
    a = @m_ground.m_xf.position.Copy()
    a.Add @m_groundAnchor2
    a

  b2PulleyJoint::GetLength1 = ->
    p = @m_bodyA.GetWorldPoint(@m_localAnchor1)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    dX = p.x - sX
    dY = p.y - sY
    Math.sqrt dX * dX + dY * dY

  b2PulleyJoint::GetLength2 = ->
    p = @m_bodyB.GetWorldPoint(@m_localAnchor2)
    sX = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    sY = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    dX = p.x - sX
    dY = p.y - sY
    Math.sqrt dX * dX + dY * dY

  b2PulleyJoint::GetRatio = ->
    @m_ratio

  b2PulleyJoint::b2PulleyJoint = (def) ->
    @__super.b2Joint.call this, def
    tMat = undefined
    tX = 0
    tY = 0
    @m_ground = @m_bodyA.m_world.m_groundBody
    @m_groundAnchor1.x = def.groundAnchorA.x - @m_ground.m_xf.position.x
    @m_groundAnchor1.y = def.groundAnchorA.y - @m_ground.m_xf.position.y
    @m_groundAnchor2.x = def.groundAnchorB.x - @m_ground.m_xf.position.x
    @m_groundAnchor2.y = def.groundAnchorB.y - @m_ground.m_xf.position.y
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_ratio = def.ratio
    @m_constant = def.lengthA + @m_ratio * def.lengthB
    @m_maxLength1 = b2Math.Min(def.maxLengthA, @m_constant - @m_ratio * b2PulleyJoint.b2_minPulleyLength)
    @m_maxLength2 = b2Math.Min(def.maxLengthB, (@m_constant - b2PulleyJoint.b2_minPulleyLength) / @m_ratio)
    @m_impulse = 0.0
    @m_limitImpulse1 = 0.0
    @m_limitImpulse2 = 0.0
    return

  b2PulleyJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    p1X = bA.m_sweep.c.x + r1X
    p1Y = bA.m_sweep.c.y + r1Y
    p2X = bB.m_sweep.c.x + r2X
    p2Y = bB.m_sweep.c.y + r2Y
    s1X = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    s1Y = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    s2X = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    s2Y = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    @m_u1.Set p1X - s1X, p1Y - s1Y
    @m_u2.Set p2X - s2X, p2Y - s2Y
    length1 = @m_u1.Length()
    length2 = @m_u2.Length()
    if length1 > b2Settings.b2_linearSlop
      @m_u1.Multiply 1.0 / length1
    else
      @m_u1.SetZero()
    if length2 > b2Settings.b2_linearSlop
      @m_u2.Multiply 1.0 / length2
    else
      @m_u2.SetZero()
    C = @m_constant - length1 - @m_ratio * length2
    if C > 0.0
      @m_state = b2Joint.e_inactiveLimit
      @m_impulse = 0.0
    else
      @m_state = b2Joint.e_atUpperLimit
    if length1 < @m_maxLength1
      @m_limitState1 = b2Joint.e_inactiveLimit
      @m_limitImpulse1 = 0.0
    else
      @m_limitState1 = b2Joint.e_atUpperLimit
    if length2 < @m_maxLength2
      @m_limitState2 = b2Joint.e_inactiveLimit
      @m_limitImpulse2 = 0.0
    else
      @m_limitState2 = b2Joint.e_atUpperLimit
    cr1u1 = r1X * @m_u1.y - r1Y * @m_u1.x
    cr2u2 = r2X * @m_u2.y - r2Y * @m_u2.x
    @m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1
    @m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2
    @m_pulleyMass = @m_limitMass1 + @m_ratio * @m_ratio * @m_limitMass2
    @m_limitMass1 = 1.0 / @m_limitMass1
    @m_limitMass2 = 1.0 / @m_limitMass2
    @m_pulleyMass = 1.0 / @m_pulleyMass
    if step.warmStarting
      @m_impulse *= step.dtRatio
      @m_limitImpulse1 *= step.dtRatio
      @m_limitImpulse2 *= step.dtRatio
      P1X = ((-@m_impulse) - @m_limitImpulse1) * @m_u1.x
      P1Y = ((-@m_impulse) - @m_limitImpulse1) * @m_u1.y
      P2X = ((-@m_ratio * @m_impulse) - @m_limitImpulse2) * @m_u2.x
      P2Y = ((-@m_ratio * @m_impulse) - @m_limitImpulse2) * @m_u2.y
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    else
      @m_impulse = 0.0
      @m_limitImpulse1 = 0.0
      @m_limitImpulse2 = 0.0
    return

  b2PulleyJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    v1X = 0
    v1Y = 0
    v2X = 0
    v2Y = 0
    P1X = 0
    P1Y = 0
    P2X = 0
    P2Y = 0
    Cdot = 0
    impulse = 0
    oldImpulse = 0
    if @m_state is b2Joint.e_atUpperLimit
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
      Cdot = (-(@m_u1.x * v1X + @m_u1.y * v1Y)) - @m_ratio * (@m_u2.x * v2X + @m_u2.y * v2Y)
      impulse = @m_pulleyMass * (-Cdot)
      oldImpulse = @m_impulse
      @m_impulse = b2Math.Max(0.0, @m_impulse + impulse)
      impulse = @m_impulse - oldImpulse
      P1X = (-impulse * @m_u1.x)
      P1Y = (-impulse * @m_u1.y)
      P2X = (-@m_ratio * impulse * @m_u2.x)
      P2Y = (-@m_ratio * impulse * @m_u2.y)
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    if @m_limitState1 is b2Joint.e_atUpperLimit
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y)
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X)
      Cdot = (-(@m_u1.x * v1X + @m_u1.y * v1Y))
      impulse = (-@m_limitMass1 * Cdot)
      oldImpulse = @m_limitImpulse1
      @m_limitImpulse1 = b2Math.Max(0.0, @m_limitImpulse1 + impulse)
      impulse = @m_limitImpulse1 - oldImpulse
      P1X = (-impulse * @m_u1.x)
      P1Y = (-impulse * @m_u1.y)
      bA.m_linearVelocity.x += bA.m_invMass * P1X
      bA.m_linearVelocity.y += bA.m_invMass * P1Y
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X)
    if @m_limitState2 is b2Joint.e_atUpperLimit
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y)
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X)
      Cdot = (-(@m_u2.x * v2X + @m_u2.y * v2Y))
      impulse = (-@m_limitMass2 * Cdot)
      oldImpulse = @m_limitImpulse2
      @m_limitImpulse2 = b2Math.Max(0.0, @m_limitImpulse2 + impulse)
      impulse = @m_limitImpulse2 - oldImpulse
      P2X = (-impulse * @m_u2.x)
      P2Y = (-impulse * @m_u2.y)
      bB.m_linearVelocity.x += bB.m_invMass * P2X
      bB.m_linearVelocity.y += bB.m_invMass * P2Y
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X)
    return

  b2PulleyJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    s1X = @m_ground.m_xf.position.x + @m_groundAnchor1.x
    s1Y = @m_ground.m_xf.position.y + @m_groundAnchor1.y
    s2X = @m_ground.m_xf.position.x + @m_groundAnchor2.x
    s2Y = @m_ground.m_xf.position.y + @m_groundAnchor2.y
    r1X = 0
    r1Y = 0
    r2X = 0
    r2Y = 0
    p1X = 0
    p1Y = 0
    p2X = 0
    p2Y = 0
    length1 = 0
    length2 = 0
    C = 0
    impulse = 0
    oldImpulse = 0
    oldLimitPositionImpulse = 0
    tX = 0
    linearError = 0.0
    if @m_state is b2Joint.e_atUpperLimit
      tMat = bA.m_xf.R
      r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
      r1X = tX
      tMat = bB.m_xf.R
      r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
      r2X = tX
      p1X = bA.m_sweep.c.x + r1X
      p1Y = bA.m_sweep.c.y + r1Y
      p2X = bB.m_sweep.c.x + r2X
      p2Y = bB.m_sweep.c.y + r2Y
      @m_u1.Set p1X - s1X, p1Y - s1Y
      @m_u2.Set p2X - s2X, p2Y - s2Y
      length1 = @m_u1.Length()
      length2 = @m_u2.Length()
      if length1 > b2Settings.b2_linearSlop
        @m_u1.Multiply 1.0 / length1
      else
        @m_u1.SetZero()
      if length2 > b2Settings.b2_linearSlop
        @m_u2.Multiply 1.0 / length2
      else
        @m_u2.SetZero()
      C = @m_constant - length1 - @m_ratio * length2
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_pulleyMass * C)
      p1X = (-impulse * @m_u1.x)
      p1Y = (-impulse * @m_u1.y)
      p2X = (-@m_ratio * impulse * @m_u2.x)
      p2Y = (-@m_ratio * impulse * @m_u2.y)
      bA.m_sweep.c.x += bA.m_invMass * p1X
      bA.m_sweep.c.y += bA.m_invMass * p1Y
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X)
      bB.m_sweep.c.x += bB.m_invMass * p2X
      bB.m_sweep.c.y += bB.m_invMass * p2Y
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X)
      bA.SynchronizeTransform()
      bB.SynchronizeTransform()
    if @m_limitState1 is b2Joint.e_atUpperLimit
      tMat = bA.m_xf.R
      r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
      r1X = tX
      p1X = bA.m_sweep.c.x + r1X
      p1Y = bA.m_sweep.c.y + r1Y
      @m_u1.Set p1X - s1X, p1Y - s1Y
      length1 = @m_u1.Length()
      if length1 > b2Settings.b2_linearSlop
        @m_u1.x *= 1.0 / length1
        @m_u1.y *= 1.0 / length1
      else
        @m_u1.SetZero()
      C = @m_maxLength1 - length1
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_limitMass1 * C)
      p1X = (-impulse * @m_u1.x)
      p1Y = (-impulse * @m_u1.y)
      bA.m_sweep.c.x += bA.m_invMass * p1X
      bA.m_sweep.c.y += bA.m_invMass * p1Y
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X)
      bA.SynchronizeTransform()
    if @m_limitState2 is b2Joint.e_atUpperLimit
      tMat = bB.m_xf.R
      r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
      r2X = tX
      p2X = bB.m_sweep.c.x + r2X
      p2Y = bB.m_sweep.c.y + r2Y
      @m_u2.Set p2X - s2X, p2Y - s2Y
      length2 = @m_u2.Length()
      if length2 > b2Settings.b2_linearSlop
        @m_u2.x *= 1.0 / length2
        @m_u2.y *= 1.0 / length2
      else
        @m_u2.SetZero()
      C = @m_maxLength2 - length2
      linearError = b2Math.Max(linearError, (-C))
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0)
      impulse = (-@m_limitMass2 * C)
      p2X = (-impulse * @m_u2.x)
      p2Y = (-impulse * @m_u2.y)
      bB.m_sweep.c.x += bB.m_invMass * p2X
      bB.m_sweep.c.y += bB.m_invMass * p2Y
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X)
      bB.SynchronizeTransform()
    linearError < b2Settings.b2_linearSlop

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength = 2.0
    return

  Box2D.inherit b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2PulleyJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2PulleyJointDef.b2PulleyJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @groundAnchorA = new b2Vec2()
    @groundAnchorB = new b2Vec2()
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2PulleyJointDef::b2PulleyJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_pulleyJoint
    @groundAnchorA.Set (-1.0), 1.0
    @groundAnchorB.Set 1.0, 1.0
    @localAnchorA.Set (-1.0), 0.0
    @localAnchorB.Set 1.0, 0.0
    @lengthA = 0.0
    @maxLengthA = 0.0
    @lengthB = 0.0
    @maxLengthB = 0.0
    @ratio = 1.0
    @collideConnected = true
    return

  b2PulleyJointDef::Initialize = (bA, bB, gaA, gaB, anchorA, anchorB, r) ->
    r = 0  if r is `undefined`
    @bodyA = bA
    @bodyB = bB
    @groundAnchorA.SetV gaA
    @groundAnchorB.SetV gaB
    @localAnchorA = @bodyA.GetLocalPoint(anchorA)
    @localAnchorB = @bodyB.GetLocalPoint(anchorB)
    d1X = anchorA.x - gaA.x
    d1Y = anchorA.y - gaA.y
    @lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y)
    d2X = anchorB.x - gaB.x
    d2Y = anchorB.y - gaB.y
    @lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y)
    @ratio = r
    C = @lengthA + @ratio * @lengthB
    @maxLengthA = C - @ratio * b2PulleyJoint.b2_minPulleyLength
    @maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / @ratio
    return

  Box2D.inherit b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint
  b2RevoluteJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2RevoluteJoint.b2RevoluteJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @K = new b2Mat22()
    @K1 = new b2Mat22()
    @K2 = new b2Mat22()
    @K3 = new b2Mat22()
    @impulse3 = new b2Vec3()
    @impulse2 = new b2Vec2()
    @reduced = new b2Vec2()
    @m_localAnchor1 = new b2Vec2()
    @m_localAnchor2 = new b2Vec2()
    @m_impulse = new b2Vec3()
    @m_mass = new b2Mat33()
    return

  b2RevoluteJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchor1

  b2RevoluteJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchor2

  b2RevoluteJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2RevoluteJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.z

  b2RevoluteJoint::GetJointAngle = ->
    @m_bodyB.m_sweep.a - @m_bodyA.m_sweep.a - @m_referenceAngle

  b2RevoluteJoint::GetJointSpeed = ->
    @m_bodyB.m_angularVelocity - @m_bodyA.m_angularVelocity

  b2RevoluteJoint::IsLimitEnabled = ->
    @m_enableLimit

  b2RevoluteJoint::EnableLimit = (flag) ->
    @m_enableLimit = flag
    return

  b2RevoluteJoint::GetLowerLimit = ->
    @m_lowerAngle

  b2RevoluteJoint::GetUpperLimit = ->
    @m_upperAngle

  b2RevoluteJoint::SetLimits = (lower, upper) ->
    lower = 0  if lower is `undefined`
    upper = 0  if upper is `undefined`
    @m_lowerAngle = lower
    @m_upperAngle = upper
    return

  b2RevoluteJoint::IsMotorEnabled = ->
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_enableMotor

  b2RevoluteJoint::EnableMotor = (flag) ->
    @m_enableMotor = flag
    return

  b2RevoluteJoint::SetMotorSpeed = (speed) ->
    speed = 0  if speed is `undefined`
    @m_bodyA.SetAwake true
    @m_bodyB.SetAwake true
    @m_motorSpeed = speed
    return

  b2RevoluteJoint::GetMotorSpeed = ->
    @m_motorSpeed

  b2RevoluteJoint::SetMaxMotorTorque = (torque) ->
    torque = 0  if torque is `undefined`
    @m_maxMotorTorque = torque
    return

  b2RevoluteJoint::GetMotorTorque = ->
    @m_maxMotorTorque

  b2RevoluteJoint::b2RevoluteJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchor1.SetV def.localAnchorA
    @m_localAnchor2.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_motorImpulse = 0.0
    @m_lowerAngle = def.lowerAngle
    @m_upperAngle = def.upperAngle
    @m_maxMotorTorque = def.maxMotorTorque
    @m_motorSpeed = def.motorSpeed
    @m_enableLimit = def.enableLimit
    @m_enableMotor = def.enableMotor
    @m_limitState = b2Joint.e_inactiveLimit
    return

  b2RevoluteJoint::InitVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    
    ###*
    todo: looks like a bug
    ###
    
    #if (this.m_enableMotor || this.m_enableLimit) {}
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    m1 = bA.m_invMass
    m2 = bB.m_invMass
    i1 = bA.m_invI
    i2 = bB.m_invI
    @m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2
    @m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2
    @m_mass.col3.x = (-r1Y * i1) - r2Y * i2
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2
    @m_mass.col3.y = r1X * i1 + r2X * i2
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = i1 + i2
    @m_motorMass = 1.0 / (i1 + i2)
    @m_motorImpulse = 0.0  if @m_enableMotor is false
    if @m_enableLimit
      jointAngle = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
      if b2Math.Abs(@m_upperAngle - @m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop
        @m_limitState = b2Joint.e_equalLimits
      else if jointAngle <= @m_lowerAngle
        @m_impulse.z = 0.0  unless @m_limitState is b2Joint.e_atLowerLimit
        @m_limitState = b2Joint.e_atLowerLimit
      else if jointAngle >= @m_upperAngle
        @m_impulse.z = 0.0  unless @m_limitState is b2Joint.e_atUpperLimit
        @m_limitState = b2Joint.e_atUpperLimit
      else
        @m_limitState = b2Joint.e_inactiveLimit
        @m_impulse.z = 0.0
    else
      @m_limitState = b2Joint.e_inactiveLimit
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_motorImpulse *= step.dtRatio
      PX = @m_impulse.x
      PY = @m_impulse.y
      bA.m_linearVelocity.x -= m1 * PX
      bA.m_linearVelocity.y -= m1 * PY
      bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + @m_motorImpulse + @m_impulse.z)
      bB.m_linearVelocity.x += m2 * PX
      bB.m_linearVelocity.y += m2 * PY
      bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + @m_motorImpulse + @m_impulse.z)
    else
      @m_impulse.SetZero()
      @m_motorImpulse = 0.0
    return

  b2RevoluteJoint::SolveVelocityConstraints = (step) ->
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = undefined
    tX = 0
    newImpulse = 0
    r1X = 0
    r1Y = 0
    r2X = 0
    r2Y = 0
    v1 = bA.m_linearVelocity
    w1 = bA.m_angularVelocity
    v2 = bB.m_linearVelocity
    w2 = bB.m_angularVelocity
    m1 = bA.m_invMass
    m2 = bB.m_invMass
    i1 = bA.m_invI
    i2 = bB.m_invI
    if @m_enableMotor and @m_limitState isnt b2Joint.e_equalLimits
      Cdot = w2 - w1 - @m_motorSpeed
      impulse = @m_motorMass * (-Cdot)
      oldImpulse = @m_motorImpulse
      maxImpulse = step.dt * @m_maxMotorTorque
      @m_motorImpulse = b2Math.Clamp(@m_motorImpulse + impulse, (-maxImpulse), maxImpulse)
      impulse = @m_motorImpulse - oldImpulse
      w1 -= i1 * impulse
      w2 += i2 * impulse
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      tMat = bA.m_xf.R
      r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
      r1X = tX
      tMat = bB.m_xf.R
      r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
      r2X = tX
      Cdot1X = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y)
      Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X)
      Cdot2 = w2 - w1
      @m_mass.Solve33 @impulse3, (-Cdot1X), (-Cdot1Y), (-Cdot2)
      if @m_limitState is b2Joint.e_equalLimits
        @m_impulse.Add @impulse3
      else if @m_limitState is b2Joint.e_atLowerLimit
        newImpulse = @m_impulse.z + @impulse3.z
        if newImpulse < 0.0
          @m_mass.Solve22 @reduced, (-Cdot1X), (-Cdot1Y)
          @impulse3.x = @reduced.x
          @impulse3.y = @reduced.y
          @impulse3.z = (-@m_impulse.z)
          @m_impulse.x += @reduced.x
          @m_impulse.y += @reduced.y
          @m_impulse.z = 0.0
      else if @m_limitState is b2Joint.e_atUpperLimit
        newImpulse = @m_impulse.z + @impulse3.z
        if newImpulse > 0.0
          @m_mass.Solve22 @reduced, (-Cdot1X), (-Cdot1Y)
          @impulse3.x = @reduced.x
          @impulse3.y = @reduced.y
          @impulse3.z = (-@m_impulse.z)
          @m_impulse.x += @reduced.x
          @m_impulse.y += @reduced.y
          @m_impulse.z = 0.0
      v1.x -= m1 * @impulse3.x
      v1.y -= m1 * @impulse3.y
      w1 -= i1 * (r1X * @impulse3.y - r1Y * @impulse3.x + @impulse3.z)
      v2.x += m2 * @impulse3.x
      v2.y += m2 * @impulse3.y
      w2 += i2 * (r2X * @impulse3.y - r2Y * @impulse3.x + @impulse3.z)
    else
      tMat = bA.m_xf.R
      r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
      r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
      r1X = tX
      tMat = bB.m_xf.R
      r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
      r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
      r2X = tX
      CdotX = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y)
      CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X)
      @m_mass.Solve22 @impulse2, (-CdotX), (-CdotY)
      @m_impulse.x += @impulse2.x
      @m_impulse.y += @impulse2.y
      v1.x -= m1 * @impulse2.x
      v1.y -= m1 * @impulse2.y
      w1 -= i1 * (r1X * @impulse2.y - r1Y * @impulse2.x)
      v2.x += m2 * @impulse2.x
      v2.y += m2 * @impulse2.y
      w2 += i2 * (r2X * @impulse2.y - r2Y * @impulse2.x)
    bA.m_linearVelocity.SetV v1
    bA.m_angularVelocity = w1
    bB.m_linearVelocity.SetV v2
    bB.m_angularVelocity = w2
    return

  b2RevoluteJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    oldLimitImpulse = 0
    C = 0
    tMat = undefined
    bA = @m_bodyA
    bB = @m_bodyB
    angularError = 0.0
    positionError = 0.0
    tX = 0
    impulseX = 0
    impulseY = 0
    if @m_enableLimit and @m_limitState isnt b2Joint.e_inactiveLimit
      angle = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
      limitImpulse = 0.0
      if @m_limitState is b2Joint.e_equalLimits
        C = b2Math.Clamp(angle - @m_lowerAngle, (-b2Settings.b2_maxAngularCorrection), b2Settings.b2_maxAngularCorrection)
        limitImpulse = (-@m_motorMass * C)
        angularError = b2Math.Abs(C)
      else if @m_limitState is b2Joint.e_atLowerLimit
        C = angle - @m_lowerAngle
        angularError = (-C)
        C = b2Math.Clamp(C + b2Settings.b2_angularSlop, (-b2Settings.b2_maxAngularCorrection), 0.0)
        limitImpulse = (-@m_motorMass * C)
      else if @m_limitState is b2Joint.e_atUpperLimit
        C = angle - @m_upperAngle
        angularError = C
        C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection)
        limitImpulse = (-@m_motorMass * C)
      bA.m_sweep.a -= bA.m_invI * limitImpulse
      bB.m_sweep.a += bB.m_invI * limitImpulse
      bA.SynchronizeTransform()
      bB.SynchronizeTransform()
    tMat = bA.m_xf.R
    r1X = @m_localAnchor1.x - bA.m_sweep.localCenter.x
    r1Y = @m_localAnchor1.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y)
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y)
    r1X = tX
    tMat = bB.m_xf.R
    r2X = @m_localAnchor2.x - bB.m_sweep.localCenter.x
    r2Y = @m_localAnchor2.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y)
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y)
    r2X = tX
    CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
    CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    CLengthSquared = CX * CX + CY * CY
    CLength = Math.sqrt(CLengthSquared)
    positionError = CLength
    invMass1 = bA.m_invMass
    invMass2 = bB.m_invMass
    invI1 = bA.m_invI
    invI2 = bB.m_invI
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop
    if CLengthSquared > k_allowedStretch * k_allowedStretch
      uX = CX / CLength
      uY = CY / CLength
      k = invMass1 + invMass2
      m = 1.0 / k
      impulseX = m * (-CX)
      impulseY = m * (-CY)
      k_beta = 0.5
      bA.m_sweep.c.x -= k_beta * invMass1 * impulseX
      bA.m_sweep.c.y -= k_beta * invMass1 * impulseY
      bB.m_sweep.c.x += k_beta * invMass2 * impulseX
      bB.m_sweep.c.y += k_beta * invMass2 * impulseY
      CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X
      CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y
    @K1.col1.x = invMass1 + invMass2
    @K1.col2.x = 0.0
    @K1.col1.y = 0.0
    @K1.col2.y = invMass1 + invMass2
    @K2.col1.x = invI1 * r1Y * r1Y
    @K2.col2.x = (-invI1 * r1X * r1Y)
    @K2.col1.y = (-invI1 * r1X * r1Y)
    @K2.col2.y = invI1 * r1X * r1X
    @K3.col1.x = invI2 * r2Y * r2Y
    @K3.col2.x = (-invI2 * r2X * r2Y)
    @K3.col1.y = (-invI2 * r2X * r2Y)
    @K3.col2.y = invI2 * r2X * r2X
    @K.SetM @K1
    @K.AddM @K2
    @K.AddM @K3
    @K.Solve b2RevoluteJoint.tImpulse, (-CX), (-CY)
    impulseX = b2RevoluteJoint.tImpulse.x
    impulseY = b2RevoluteJoint.tImpulse.y
    bA.m_sweep.c.x -= bA.m_invMass * impulseX
    bA.m_sweep.c.y -= bA.m_invMass * impulseY
    bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX)
    bB.m_sweep.c.x += bB.m_invMass * impulseX
    bB.m_sweep.c.y += bB.m_invMass * impulseY
    bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    positionError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.postDefs.push ->
    Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2()
    return

  Box2D.inherit b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2RevoluteJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2RevoluteJointDef.b2RevoluteJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2RevoluteJointDef::b2RevoluteJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_revoluteJoint
    @localAnchorA.Set 0.0, 0.0
    @localAnchorB.Set 0.0, 0.0
    @referenceAngle = 0.0
    @lowerAngle = 0.0
    @upperAngle = 0.0
    @maxMotorTorque = 0.0
    @motorSpeed = 0.0
    @enableLimit = false
    @enableMotor = false
    return

  b2RevoluteJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA = @bodyA.GetLocalPoint(anchor)
    @localAnchorB = @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  Box2D.inherit b2WeldJoint, Box2D.Dynamics.Joints.b2Joint
  b2WeldJoint::__super = Box2D.Dynamics.Joints.b2Joint::
  b2WeldJoint.b2WeldJoint = ->
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply this, arguments
    @m_localAnchorA = new b2Vec2()
    @m_localAnchorB = new b2Vec2()
    @m_impulse = new b2Vec3()
    @m_mass = new b2Mat33()
    return

  b2WeldJoint::GetAnchorA = ->
    @m_bodyA.GetWorldPoint @m_localAnchorA

  b2WeldJoint::GetAnchorB = ->
    @m_bodyB.GetWorldPoint @m_localAnchorB

  b2WeldJoint::GetReactionForce = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    new b2Vec2(inv_dt * @m_impulse.x, inv_dt * @m_impulse.y)

  b2WeldJoint::GetReactionTorque = (inv_dt) ->
    inv_dt = 0  if inv_dt is `undefined`
    inv_dt * @m_impulse.z

  b2WeldJoint::b2WeldJoint = (def) ->
    @__super.b2Joint.call this, def
    @m_localAnchorA.SetV def.localAnchorA
    @m_localAnchorB.SetV def.localAnchorB
    @m_referenceAngle = def.referenceAngle
    @m_impulse.SetZero()
    @m_mass = new b2Mat33()
    return

  b2WeldJoint::InitVelocityConstraints = (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    @m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB
    @m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB
    @m_mass.col3.x = (-rAY * iA) - rBY * iB
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB
    @m_mass.col3.y = rAX * iA + rBX * iB
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = iA + iB
    if step.warmStarting
      @m_impulse.x *= step.dtRatio
      @m_impulse.y *= step.dtRatio
      @m_impulse.z *= step.dtRatio
      bA.m_linearVelocity.x -= mA * @m_impulse.x
      bA.m_linearVelocity.y -= mA * @m_impulse.y
      bA.m_angularVelocity -= iA * (rAX * @m_impulse.y - rAY * @m_impulse.x + @m_impulse.z)
      bB.m_linearVelocity.x += mB * @m_impulse.x
      bB.m_linearVelocity.y += mB * @m_impulse.y
      bB.m_angularVelocity += iB * (rBX * @m_impulse.y - rBY * @m_impulse.x + @m_impulse.z)
    else
      @m_impulse.SetZero()
    return

  b2WeldJoint::SolveVelocityConstraints = (step) ->
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    vA = bA.m_linearVelocity
    wA = bA.m_angularVelocity
    vB = bB.m_linearVelocity
    wB = bB.m_angularVelocity
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY
    Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX
    Cdot2 = wB - wA
    impulse = new b2Vec3()
    @m_mass.Solve33 impulse, (-Cdot1X), (-Cdot1Y), (-Cdot2)
    @m_impulse.Add impulse
    vA.x -= mA * impulse.x
    vA.y -= mA * impulse.y
    wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z)
    vB.x += mB * impulse.x
    vB.y += mB * impulse.y
    wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z)
    bA.m_angularVelocity = wA
    bB.m_angularVelocity = wB
    return

  b2WeldJoint::SolvePositionConstraints = (baumgarte) ->
    baumgarte = 0  if baumgarte is `undefined`
    tMat = undefined
    tX = 0
    bA = @m_bodyA
    bB = @m_bodyB
    tMat = bA.m_xf.R
    rAX = @m_localAnchorA.x - bA.m_sweep.localCenter.x
    rAY = @m_localAnchorA.y - bA.m_sweep.localCenter.y
    tX = (tMat.col1.x * rAX + tMat.col2.x * rAY)
    rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY)
    rAX = tX
    tMat = bB.m_xf.R
    rBX = @m_localAnchorB.x - bB.m_sweep.localCenter.x
    rBY = @m_localAnchorB.y - bB.m_sweep.localCenter.y
    tX = (tMat.col1.x * rBX + tMat.col2.x * rBY)
    rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY)
    rBX = tX
    mA = bA.m_invMass
    mB = bB.m_invMass
    iA = bA.m_invI
    iB = bB.m_invI
    C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX
    C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY
    C2 = bB.m_sweep.a - bA.m_sweep.a - @m_referenceAngle
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop
    positionError = Math.sqrt(C1X * C1X + C1Y * C1Y)
    angularError = b2Math.Abs(C2)
    if positionError > k_allowedStretch
      iA *= 1.0
      iB *= 1.0
    @m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB
    @m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB
    @m_mass.col3.x = (-rAY * iA) - rBY * iB
    @m_mass.col1.y = @m_mass.col2.x
    @m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB
    @m_mass.col3.y = rAX * iA + rBX * iB
    @m_mass.col1.z = @m_mass.col3.x
    @m_mass.col2.z = @m_mass.col3.y
    @m_mass.col3.z = iA + iB
    impulse = new b2Vec3()
    @m_mass.Solve33 impulse, (-C1X), (-C1Y), (-C2)
    bA.m_sweep.c.x -= mA * impulse.x
    bA.m_sweep.c.y -= mA * impulse.y
    bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z)
    bB.m_sweep.c.x += mB * impulse.x
    bB.m_sweep.c.y += mB * impulse.y
    bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z)
    bA.SynchronizeTransform()
    bB.SynchronizeTransform()
    positionError <= b2Settings.b2_linearSlop and angularError <= b2Settings.b2_angularSlop

  Box2D.inherit b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef
  b2WeldJointDef::__super = Box2D.Dynamics.Joints.b2JointDef::
  b2WeldJointDef.b2WeldJointDef = ->
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply this, arguments
    @localAnchorA = new b2Vec2()
    @localAnchorB = new b2Vec2()
    return

  b2WeldJointDef::b2WeldJointDef = ->
    @__super.b2JointDef.call this
    @type = b2Joint.e_weldJoint
    @referenceAngle = 0.0
    return

  b2WeldJointDef::Initialize = (bA, bB, anchor) ->
    @bodyA = bA
    @bodyB = bB
    @localAnchorA.SetV @bodyA.GetLocalPoint(anchor)
    @localAnchorB.SetV @bodyB.GetLocalPoint(anchor)
    @referenceAngle = @bodyB.GetAngle() - @bodyA.GetAngle()
    return

  return
)()
(->
  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
  b2DebugDraw.b2DebugDraw = ->
    @m_drawScale = 1.0
    @m_lineThickness = 1.0
    @m_alpha = 1.0
    @m_fillAlpha = 1.0
    @m_xformScale = 1.0
    __this = this
    
    ##WORKAROUND
    @m_sprite = graphics:
      clear: ->
        __this.m_ctx.clearRect 0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height
        return

    return

  b2DebugDraw::_color = (color, alpha) ->
    "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")"

  b2DebugDraw::b2DebugDraw = ->
    @m_drawFlags = 0
    return

  b2DebugDraw::SetFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags = flags
    return

  b2DebugDraw::GetFlags = ->
    @m_drawFlags

  b2DebugDraw::AppendFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags |= flags
    return

  b2DebugDraw::ClearFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags &= ~flags
    return

  b2DebugDraw::SetSprite = (sprite) ->
    @m_ctx = sprite
    return

  b2DebugDraw::GetSprite = ->
    @m_ctx

  b2DebugDraw::SetDrawScale = (drawScale) ->
    drawScale = 0  if drawScale is `undefined`
    @m_drawScale = drawScale
    return

  b2DebugDraw::GetDrawScale = ->
    @m_drawScale

  b2DebugDraw::SetLineThickness = (lineThickness) ->
    lineThickness = 0  if lineThickness is `undefined`
    @m_lineThickness = lineThickness
    @m_ctx.strokeWidth = lineThickness
    return

  b2DebugDraw::GetLineThickness = ->
    @m_lineThickness

  b2DebugDraw::SetAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    @m_alpha = alpha
    return

  b2DebugDraw::GetAlpha = ->
    @m_alpha

  b2DebugDraw::SetFillAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    @m_fillAlpha = alpha
    return

  b2DebugDraw::GetFillAlpha = ->
    @m_fillAlpha

  b2DebugDraw::SetXFormScale = (xformScale) ->
    xformScale = 0  if xformScale is `undefined`
    @m_xformScale = xformScale
    return

  b2DebugDraw::GetXFormScale = ->
    @m_xformScale

  b2DebugDraw::DrawPolygon = (vertices, vertexCount, color) ->
    return  unless vertexCount
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.moveTo vertices[0].x * drawScale, vertices[0].y * drawScale
    i = 1

    while i < vertexCount
      s.lineTo vertices[i].x * drawScale, vertices[i].y * drawScale
      i++
    s.lineTo vertices[0].x * drawScale, vertices[0].y * drawScale
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawSolidPolygon = (vertices, vertexCount, color) ->
    return  unless vertexCount
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.fillStyle = @_color(color.color, @m_fillAlpha)
    s.moveTo vertices[0].x * drawScale, vertices[0].y * drawScale
    i = 1

    while i < vertexCount
      s.lineTo vertices[i].x * drawScale, vertices[i].y * drawScale
      i++
    s.lineTo vertices[0].x * drawScale, vertices[0].y * drawScale
    s.closePath()
    s.fill()
    s.stroke()
    return

  b2DebugDraw::DrawCircle = (center, radius, color) ->
    return  unless radius
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.arc center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawSolidCircle = (center, radius, axis, color) ->
    return  unless radius
    s = @m_ctx
    drawScale = @m_drawScale
    cx = center.x * drawScale
    cy = center.y * drawScale
    s.moveTo 0, 0
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.fillStyle = @_color(color.color, @m_fillAlpha)
    s.arc cx, cy, radius * drawScale, 0, Math.PI * 2, true
    s.moveTo cx, cy
    s.lineTo (center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale
    s.closePath()
    s.fill()
    s.stroke()
    return

  b2DebugDraw::DrawSegment = (p1, p2, color) ->
    s = @m_ctx
    drawScale = @m_drawScale
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.beginPath()
    s.moveTo p1.x * drawScale, p1.y * drawScale
    s.lineTo p2.x * drawScale, p2.y * drawScale
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawTransform = (xf) ->
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(0xff0000, @m_alpha)
    s.moveTo xf.position.x * drawScale, xf.position.y * drawScale
    s.lineTo (xf.position.x + @m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + @m_xformScale * xf.R.col1.y) * drawScale
    s.strokeStyle = @_color(0xff00, @m_alpha)
    s.moveTo xf.position.x * drawScale, xf.position.y * drawScale
    s.lineTo (xf.position.x + @m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + @m_xformScale * xf.R.col2.y) * drawScale
    s.closePath()
    s.stroke()
    return

  return
)() #post-definitions
i = undefined
i = 0
while i < Box2D.postDefs.length
  Box2D.postDefs[i]()
  ++i
delete Box2D.postDefs
