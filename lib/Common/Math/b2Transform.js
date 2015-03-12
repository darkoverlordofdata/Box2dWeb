/**
 * Class b2Transform
 *
 *
 */
Box2D.Common.Math.b2Transform = b2Transform = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param pos
    * @param r
    *
    */
   function b2Transform(pos, r){
      this.position = new b2Vec2;
      this.R = new b2Mat22();
      pos = pos || null;
      r = r || null;
      if (pos) {
         this.position.SetV(pos);
         this.R.SetM(r);
      }
   }

   /**
    * Initialize
    *
    * @param pos
    * @param r
    *
    */
   b2Transform.prototype.Initialize = function (pos, r) {
      this.position.SetV(pos);
      this.R.SetM(r);
   };

   /**
    * SetIdentity
    *
    * @param 
    *
    */
   b2Transform.prototype.SetIdentity = function () {
      this.position.SetZero();
      this.R.SetIdentity();
   };

   /**
    * Set
    *
    * @param x
    *
    */
   b2Transform.prototype.Set = function (x) {
      this.position.SetV(x.position);
      this.R.SetM(x.R);
   };

   /**
    * GetAngle
    *
    * @param 
    *
    */
   b2Transform.prototype.GetAngle = function () {
      return Math.atan2(this.R.col1.y, this.R.col1.x);
   };
   return b2Transform;
})();