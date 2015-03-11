/**
 * Class b2RevoluteJointDef
 *
 *
 */
b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef = (function(superClass) {
   extend(b2RevoluteJointDef, superClass);

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2RevoluteJointDef(){

      this.localAnchorA = new b2Vec2();
      this.localAnchorB = new b2Vec2();
      b2RevoluteJointDef.__super__.constructor.call(this);
      this.type = b2Joint.e_revoluteJoint;
      this.localAnchorA.Set(0.0, 0.0);
      this.localAnchorB.Set(0.0, 0.0);
      this.referenceAngle = 0.0;
      this.lowerAngle = 0.0;
      this.upperAngle = 0.0;
      this.maxMotorTorque = 0.0;
      this.motorSpeed = 0.0;
      this.enableLimit = false;
      this.enableMotor = false;
   }

   /**
    * b2RevoluteJointDef
    *
    * @param 
    *
    */
   b2RevoluteJointDef.prototype.b2RevoluteJointDef = function () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_revoluteJoint;
      this.localAnchorA.Set(0.0, 0.0);
      this.localAnchorB.Set(0.0, 0.0);
      this.referenceAngle = 0.0;
      this.lowerAngle = 0.0;
      this.upperAngle = 0.0;
      this.maxMotorTorque = 0.0;
      this.motorSpeed = 0.0;
      this.enableLimit = false;
      this.enableMotor = false;
   };

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    *
    */
   b2RevoluteJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2RevoluteJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };
   return b2RevoluteJointDef;
})(b2JointDef);