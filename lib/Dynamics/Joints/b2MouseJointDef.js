/**
 * Class b2MouseJointDef
 *
 *
 */
b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef = (function(superClass) {
'use strict;'
   extend(b2MouseJointDef, superClass);

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2MouseJointDef(){

      this.target = new b2Vec2();
      b2MouseJointDef.__super__.constructor.call(this);
      this.type = b2Joint.e_mouseJoint;
      this.maxForce = 0.0;
      this.frequencyHz = 5.0;
      this.dampingRatio = 0.7;
   }

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2MouseJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   return b2MouseJointDef;
})(b2JointDef);