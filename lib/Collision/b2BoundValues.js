/**
 * Class b2BoundValues
 *
 *
 */
Box2D.Collision.b2BoundValues = b2BoundValues = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2BoundValues(){

      this.lowerValues = new Vector_a2j_Number();
      this.lowerValues[0] = 0.0;
      this.lowerValues[1] = 0.0;
      this.upperValues = new Vector_a2j_Number();
      this.upperValues[0] = 0.0;
      this.upperValues[1] = 0.0;
   }
   return b2BoundValues;
})();