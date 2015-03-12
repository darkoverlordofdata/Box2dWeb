/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *//* Box2dWeb Redux Copyright (c) 2015 by DarkOverlordOfData */

'use strict;'
var Box2D = (function() {
var ClipVertex, Features, b2AABB, b2Body, b2BodyDef, b2Bound, b2BoundValues,
    b2BuoyancyController, b2CircleContact, b2CircleShape, b2Collision, b2Color, b2ConstantAccelController,
    b2ConstantForceController, b2Contact, b2ContactConstraint, b2ContactConstraintPoint, b2ContactEdge,
    b2ContactFactory, b2ContactFilter, b2ContactID, b2ContactImpulse, b2ContactListener, b2ContactManager,
    b2ContactPoint, b2ContactRegister, b2ContactResult, b2ContactSolver, b2Controller, b2ControllerEdge,
    b2DebugDraw, b2DestructionListener, b2Distance, b2DistanceInput, b2DistanceJoint, b2DistanceJointDef,
    b2DistanceOutput, b2DistanceProxy, b2DynamicTree, b2DynamicTreeBroadPhase, b2DynamicTreeNode,
    b2DynamicTreePair, b2EdgeAndCircleContact, b2EdgeChainDef, b2EdgeShape, b2FilterData, b2Fixture,
    b2FixtureDef, b2FrictionJoint, b2FrictionJointDef, b2GearJoint, b2GearJointDef, b2GravityController,
    b2Island, b2Jacobian, b2Joint, b2JointDef, b2JointEdge, b2LineJoint, b2LineJointDef, b2Manifold,
    b2ManifoldPoint, b2MassData, b2Mat22, b2Mat33, b2Math, b2MouseJoint, b2MouseJointDef, b2NullContact,
    b2Point, b2PolyAndCircleContact, b2PolyAndEdgeContact, b2PolygonContact, b2PolygonShape,
    b2PositionSolverManifold, b2PrismaticJoint, b2PrismaticJointDef, b2PulleyJoint, b2PulleyJointDef,
    b2RayCastInput, b2RayCastOutput, b2RevoluteJoint, b2RevoluteJointDef, b2Segment, b2SeparationFunction,
    b2Settings, b2Shape, b2Simplex, b2SimplexCache, b2SimplexVertex, b2Sweep, b2TOIInput,
    b2TensorDampingController, b2TimeOfImpact, b2TimeStep, b2Transform, b2Vec2, b2Vec3, b2WeldJoint,
    b2WeldJointDef, b2World, b2WorldManifold, parseUInt,
    b2internal = 'Box2D.Common.b2internal',
    IBroadPhase = 'Box2D.Collision.IBroadPhase',
    Box2D = {

        is: function(o1, o2) {
            var ref;
            if (o1 === null) {
                return false;
            }
            if ((o2 instanceof Function) && (o1 instanceof o2)) {
                return true;
            }
            if (((ref = o1.constructor.__implements) != null ? ref[o2.name] : void 0)) {
                return true;
            }
            return false;
        },

        generateCallback: function(context, cb) {
            return function() {
                cb.apply(context, arguments);
            };
        },

        NVector: function(length) {
            var a, i;
            if (length == null) {
                length = 0;
            }
            a = new Array(length);
            i = 0;
            while (i < length) {
                a[i] = 0;
                ++i;
            }
            return a;
        },

        parseUInt: function(v) {
            return Math.abs(parseInt(v));
        }
    },
    Vector = Array,
    Vector_a2j_Number = Box2D.NVector;

    Box2D.Common = {
        b2internal: b2internal,
            Math: {}
    };

    Box2D.Collision = {
        IBroadPhase: IBroadPhase,
            Shapes: {}
    };

    Box2D.Dynamics = {
        Contacts: {},
        Controllers: {},
        Joints: {}
    };

    '{% Box2D %}'

    return Box2D;
}).call(this);
if (module !== void 0) {
    module.exports = Box2D;
}