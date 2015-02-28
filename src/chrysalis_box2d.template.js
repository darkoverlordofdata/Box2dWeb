/**
 * Copyright (c) 2015 Dark Overlord of Data
 * chrysalis_box2d.js
 *
 * Wrapper for Box2DWeb
 *
 * Mimic cocoon_box2d.js in the browser
 *
 */
var Box2D = (function(){

    {% include '../Box2dWeb-v41.0.0.js' %}
    /**
     *  Modify the interface
     *
     */

    // ***************************************************************************
    //                               b2DebugDraw
    // ***************************************************************************

    var B2DebugDraw = function(){
        this.e_aabbBit = 0x0004;
        this.e_centerOfMassBit = 0x0010;
        this.e_controllerBit = 0x0020;
        this.e_jointBit = 0x0002;
        this.e_pairBit  = 0x0008;
        this.e_shapeBit = 0x000;
    };

    //window.Box2D.Dynamics.b2DebugDraw = B2DebugDraw ;

    B2DebugDraw.prototype.AppendFlags      = function(){};
    B2DebugDraw.prototype.ClearFlags       = function(){};
    B2DebugDraw.prototype.DrawCircle       = function(){};
    B2DebugDraw.prototype.DrawPolygon      = function(){};
    B2DebugDraw.prototype.DrawSegment      = function(){};
    B2DebugDraw.prototype.DrawSolidCircle  = function(){};
    B2DebugDraw.prototype.DrawSolidPolygon = function(){};
    B2DebugDraw.prototype.DrawTransform    = function(){};
    B2DebugDraw.prototype.GetAlpha         = function(){};
    B2DebugDraw.prototype.GetDrawScale     = function(){};
    B2DebugDraw.prototype.GetFillAlpha     = function(){};
    B2DebugDraw.prototype.GetFlags         = function(){};
    B2DebugDraw.prototype.GetLineThickness = function(){};
    B2DebugDraw.prototype.GetSprite        = function(){};
    B2DebugDraw.prototype.GetXFormScale    = function(){};
    B2DebugDraw.prototype.SetAlpha         = function(){};
    B2DebugDraw.prototype.SetDrawScale     = function(){};
    B2DebugDraw.prototype.SetFillAlpha     = function(){};
    B2DebugDraw.prototype.SetFlags         = function(){};
    B2DebugDraw.prototype.SetLineThickness = function(){};
    B2DebugDraw.prototype.SetSprite        = function(){};
    B2DebugDraw.prototype.SetXFormScale    = function(){};



    return { // Remap Box2D to CocoonJS API requirements
        Collision: {
            Shapes: {
                b2CircleShape: Box2D.Collision.Shapes.b2CircleShape,
                b2PolygonShape: Box2D.Collision.Shapes.b2PolygonShape
            }
        },
        Common: {
            Math: {
                b2Mat22: Box2D.Common.Math.b2Mat22,
                b2Math: Box2D.Common.Math.b2Math,
                b2Transform: Box2D.Common.Math.b2Transform,
                b2Vec2: Box2D.Common.Math.b2Vec2
            }
        },
        Dynamics: {
            b2Body: Box2D.Dynamics.b2Body,
            b2BodyDef: Box2D.Dynamics.b2BodyDef,
            b2Contact: Box2D.Dynamics.Contacts.b2Contact,
            b2ContactFilter: Box2D.Dynamics.b2ContactFilter,
            b2ContactListener: Box2D.Dynamics.b2ContactListener,
            b2DebugDraw: B2DebugDraw,
            b2Fixture: Box2D.Dynamics.b2Fixture,
            b2FixtureDef: Box2D.Dynamics.b2FixtureDef,
            b2World: Box2D.Dynamics.b2World,
            Joints: {
                b2DistanceJointDef: Box2D.Dynamics.Joints.b2DistanceJointDef,
                b2Joint: Box2D.Dynamics.Joints.b2Joint,
                b2RevoluteJointDef: Box2D.Dynamics.Joints.b2RevoluteJointDef
            }
        }
    };

})();
