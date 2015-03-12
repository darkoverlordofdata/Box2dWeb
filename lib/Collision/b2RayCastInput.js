/**
 * Class b2RayCastInput
 *
 *
 */
Box2D.Collision.b2RayCastInput = b2RayCastInput = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param p1
    * @param p2
    * @param maxFraction
    *
    */
   function b2RayCastInput(p1, p2, maxFraction){
      this.p1 = new b2Vec2();
      this.p2 = new b2Vec2();
      p1 = p1 || null;
      p2 = p2 || null;
      maxFraction = maxFraction || 1;
      if (p1) this.p1.SetV(p1);
      if (p2) this.p2.SetV(p2);
      this.maxFraction = maxFraction;
   }
   return b2RayCastInput;
})();