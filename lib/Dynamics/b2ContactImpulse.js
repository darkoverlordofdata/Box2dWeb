
   /**
    *  Class b2ContactImpulse
    *
    * @param 
    *
    */
   b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse = function b2ContactImpulse(){
      this.normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
      this.tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);

   };
   b2ContactImpulse.constructor = b2ContactImpulse;