/**
 * Class b2FrictionJointDef
 *
 *
 */
b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef = (function(superClass) {
'use strict;'
   extend(b2FrictionJointDef, superClass);

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2FrictionJointDef(){

      this.localAnchorA = new b2Vec2();
      this.localAnchorB = new b2Vec2();
      b2FrictionJointDef.__super__.constructor.call(this);
      this.type = b2Joint.e_frictionJoint;
      this.maxForce = 0.0;
      this.maxTorque = 0.0;
   }

   /**
    * b2FrictionJointDef
    *
    * @param 
    *
    */
   b2FrictionJointDef.prototype.b2FrictionJointDef = function () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_frictionJoint;
      this.maxForce = 0.0;
      this.maxTorque = 0.0;
   };

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    *
    */
   b2FrictionJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2FrictionJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   return b2FrictionJointDef;
})(b2JointDef);