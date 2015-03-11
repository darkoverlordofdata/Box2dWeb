/**
 * Class b2ManifoldPoint
 *
 *
 */
Box2D.Collision.b2ManifoldPoint = b2ManifoldPoint = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ManifoldPoint(){
      this.m_localPoint = new b2Vec2();
      this.m_id = new b2ContactID();
      this.Reset();
   }

   /**
    * b2ManifoldPoint
    *
    * @param 
    *
    */
   b2ManifoldPoint.prototype.b2ManifoldPoint = function () {
      this.Reset();
   };

   /**
    * Reset
    *
    * @param 
    *
    */
   b2ManifoldPoint.prototype.Reset = function () {
      this.m_localPoint.SetZero();
      this.m_normalImpulse = 0.0;
      this.m_tangentImpulse = 0.0;
      this.m_id.key = 0;
   };

   /**
    * Set
    *
    * @param m
    *
    */
   b2ManifoldPoint.prototype.Set = function (m) {
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_normalImpulse = m.m_normalImpulse;
      this.m_tangentImpulse = m.m_tangentImpulse;
      this.m_id.Set(m.m_id);
   };
   return b2ManifoldPoint;
})();