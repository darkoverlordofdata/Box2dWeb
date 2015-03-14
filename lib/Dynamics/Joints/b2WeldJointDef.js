
   /**
    *  Class b2WeldJointDef
    *
    * @param 
    *
    */
   b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef = function b2WeldJointDef() {

      this.localAnchorA = new b2Vec2();
      this.localAnchorB = new b2Vec2();
      b2JointDef.call(this);
      this.type = b2Joint.e_weldJoint;
      this.referenceAngle = 0.0;
   };
   b2WeldJointDef.constructor = b2WeldJointDef;
   b2WeldJointDef.prototype = Object.create(b2JointDef.prototype );

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    *
    */
   b2WeldJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2WeldJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };