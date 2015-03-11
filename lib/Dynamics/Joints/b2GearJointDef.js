/**
 * Class b2GearJointDef
 *
 *
 */
b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef = (function(superClass) {
'use strict;'
   extend(b2GearJointDef, superClass);

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2GearJointDef(){

      b2GearJointDef.__super__.constructor.call(this);
      this.type = b2Joint.e_gearJoint;
      this.joint1 = null;
      this.joint2 = null;
      this.ratio = 1.0;
   }

   /**
    * b2GearJointDef
    *
    * @param 
    *
    */
   b2GearJointDef.prototype.b2GearJointDef = function () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_gearJoint;
      this.joint1 = null;
      this.joint2 = null;
      this.ratio = 1.0;
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2GearJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   return b2GearJointDef;
})(b2JointDef);