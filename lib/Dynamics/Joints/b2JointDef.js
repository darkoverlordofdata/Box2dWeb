/**
 * Class b2JointDef
 *
 *
 */
Box2D.Dynamics.Joints.b2JointDef = b2JointDef = (function() {

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2JointDef(){

      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   }

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2JointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   return b2JointDef;
})();