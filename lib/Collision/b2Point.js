/**
 * Class b2Point
 *
 *
 */
Box2D.Collision.b2Point = b2Point = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2Point(){
      this.p = new b2Vec2();

   }

   /**
    * Support
    *
    * @param xf
    * @param vX
    * @param vY
    *
    */
   b2Point.prototype.Support = function (xf, vX, vY) {
      vX = vX || 0;
      vY = vY || 0;
      return this.p;
   };

   /**
    * GetFirstVertex
    *
    * @param xf
    *
    */
   b2Point.prototype.GetFirstVertex = function (xf) {
      return this.p;
   };
   return b2Point;
})();