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
###
 *
 *  Cocoon Box2D
 *
 *
 *
###
'use strict'
module.exports =
class Box2D

  class Box2D.Common

class Box2D.Common.Math
require './common/math/b2Vec2'
require './common/math/b2Mat22'
require './common/math/b2Transform'
require './common/math/b2Math'

class Box2D.Collision

class Box2D.Collision.Shapes
require './collision/shapes/b2CircleShape'
require './collision/shapes/b2PolygonShape'

class Box2D.Dynamics

class Box2D.Dynamics.Contacts
require './dynamics/contacts/b2Contact'
require './dynamics/contacts/b2ContactFilter'
require './dynamics/contacts/b2ContactListener'

class Box2D.Dynamics.Joints
require './dynamics/joints/b2Joint'
require './dynamics/joints/b2DistanceJointDef'
require './dynamics/joints/b2RevoluteJointDef'

require './dynamics/b2Fixture'
require './dynamics/b2Body'
require './dynamics/b2DebugDraw'
require './dynamics/b2BodyDef'
require './dynamics/b2FixtureDef'
require './dynamics/b2World'

