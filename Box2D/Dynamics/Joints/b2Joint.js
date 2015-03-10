function b2Joint(def){
      this.m_edgeA = new b2JointEdge();
      this.m_edgeB = new b2JointEdge();
      this.m_localCenterA = new b2Vec2();
      this.m_localCenterB = new b2Vec2();
      b2Settings.b2Assert(def.bodyA != def.bodyB);
      this.m_type = def.type;
      this.m_prev = null;
      this.m_next = null;
      this.m_bodyA = def.bodyA;
      this.m_bodyB = def.bodyB;
      this.m_collideConnected = def.collideConnected;
      this.m_islandFlag = false;
      this.m_userData = def.userData;
}
//GetType
b2Joint.prototype.GetType = function () {
      return this.m_type;
   };
//GetAnchorA
b2Joint.prototype.GetAnchorA = function () {
      return null;
   };
//GetAnchorB
b2Joint.prototype.GetAnchorB = function () {
      return null;
   };
//GetReactionForce
b2Joint.prototype.GetReactionForce = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return null;
   };
//GetReactionTorque
b2Joint.prototype.GetReactionTorque = function (inv_dt) {
      if (inv_dt === undefined) inv_dt = 0;
      return 0.0;
   };
//GetBodyA
b2Joint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };
//GetBodyB
b2Joint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };
//GetNext
b2Joint.prototype.GetNext = function () {
      return this.m_next;
   };
//GetUserData
b2Joint.prototype.GetUserData = function () {
      return this.m_userData;
   };
//SetUserData
b2Joint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };
//IsActive
b2Joint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };
//b2Joint
b2Joint.prototype.b2Joint = function (def) {
      b2Settings.b2Assert(def.bodyA != def.bodyB);
      this.m_type = def.type;
      this.m_prev = null;
      this.m_next = null;
      this.m_bodyA = def.bodyA;
      this.m_bodyB = def.bodyB;
      this.m_collideConnected = def.collideConnected;
      this.m_islandFlag = false;
      this.m_userData = def.userData;
   };
//InitVelocityConstraints
b2Joint.prototype.InitVelocityConstraints = function (step) {};
//SolveVelocityConstraints
b2Joint.prototype.SolveVelocityConstraints = function (step) {};
//FinalizeVelocityConstraints
b2Joint.prototype.FinalizeVelocityConstraints = function () {};
//SolvePositionConstraints
b2Joint.prototype.SolvePositionConstraints = function (baumgarte) {
      if (baumgarte === undefined) baumgarte = 0;
      return false;
   };