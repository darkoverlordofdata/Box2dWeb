/**
 * Class b2ContactConstraintPoint
 *
 *
 */
Box2D.Dynamics.Contacts.b2ContactConstraintPoint = b2ContactConstraintPoint = (function() {

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ContactConstraintPoint(){
      this.localPoint = new b2Vec2();
      this.rA = new b2Vec2();
      this.rB = new b2Vec2();

   }
   return b2ContactConstraintPoint;
})();