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
# Example
#
###
* Copyright (c) 2006-2007 Erin Catto http:
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
* 2. Altered source versions must be plainly marked, and must not be
* misrepresented the original software.
* 3. This notice may not be removed or altered from any source distribution.
###
'use strict'
module.exports =
class Box2D

class Box2D.Collision

class Box2D.Collision.Shapes
require './collision/shapes/b2CircleShape'
require './collision/shapes/b2PolygonShape'


class Box2D.Dynamics
require './collision/dynamics/b2DebugDraw'
require './collision/dynamics/b2BodyDef'
require './collision/dynamics/b2Fixture'
require './collision/dynamics/b2Body'
require './collision/dynamics/b2FixtureDef'
require './collision/dynamics/b2World'

class Box2D.Dynamics.Contacts
require './collision/dynamics/contacts/b2Contact'
require './collision/dynamics/contacts/b2ContactFilter'
require './collision/dynamics/contacts/b2ContactListener'

class Box2D.Dynamics.Joints
require './collision/dynamics/joints/b2DistanceJointDef'
require './collision/dynamics/joints/b2RevoluteJointDef'

class Box2D.Common

class Box2D.Common.Math
require './collision/common/math/b2Vec2'
require './collision/common/math/b2Mat22'
require './collision/common/math/b2Transform'
require './collision/common/math/b2Math'


