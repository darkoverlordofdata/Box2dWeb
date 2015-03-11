/**
 * Class b2TOIInput
 *
 *
 */
Box2D.Collision.b2TOIInput = b2TOIInput = (function() {

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2TOIInput(){
      this.proxyA = new b2DistanceProxy();
      this.proxyB = new b2DistanceProxy();
      this.sweepA = new b2Sweep();
      this.sweepB = new b2Sweep();

   }
   return b2TOIInput;
})();