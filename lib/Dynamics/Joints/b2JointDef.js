
   /**
    *  Class b2JointDef
    *
    * @param 
    *
    */
   b2JointDef = Box2D.Dynamics.Joints.b2JointDef = function b2JointDef(){

      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   b2JointDef.constructor = b2JointDef;