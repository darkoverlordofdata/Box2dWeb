
   /**
    *  Class b2LineJointDef
    *
    * @param 
    *
    */
   b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef = function b2LineJointDef() {

      this.localAnchorA = new b2Vec2();
      this.localAnchorB = new b2Vec2();
      this.localAxisA = new b2Vec2();
      b2JointDef.call(this);
      this.type = b2Joint.e_lineJoint;
      this.localAxisA.Set(1.0, 0.0);
      this.enableLimit = false;
      this.lowerTranslation = 0.0;
      this.upperTranslation = 0.0;
      this.enableMotor = false;
      this.maxMotorForce = 0.0;
      this.motorSpeed = 0.0;
   };
   b2LineJointDef.constructor = b2LineJointDef;
   b2LineJointDef.prototype = Object.create(b2JointDef.prototype );

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    * @param axis
    *
    */
   b2LineJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.localAxisA = this.bodyA.GetLocalVector(axis);
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2LineJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };