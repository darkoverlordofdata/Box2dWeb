/**
 * Class b2MassData
 *
 *
 */
Box2D.Collision.Shapes.b2MassData = b2MassData = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2MassData(){
      this.mass = 0.0;
      this.center = new b2Vec2(0, 0);
      this.I = 0.0;

   }
   return b2MassData;
})();