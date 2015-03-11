/**
 * Class b2ContactImpulse
 *
 *
 */
Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ContactImpulse(){
      this.normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
      this.tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);

   }
   return b2ContactImpulse;
})();