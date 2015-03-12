
   /**
    *  Class b2FrictionJointDef
    *
    * @param 
    *
    */
   b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef = function b2FrictionJointDef(){

      this.localAnchorA = new b2Vec2();
      this.localAnchorB = new b2Vec2();
      b2JointDef.call(this);
      this.type = b2Joint.e_frictionJoint;
      this.maxForce = 0.0;
      this.maxTorque = 0.0;
   };
   b2FrictionJointDef.constructor = b2FrictionJointDef;
   b2FrictionJointDef.prototype = Object.create(b2JointDef.prototype );

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