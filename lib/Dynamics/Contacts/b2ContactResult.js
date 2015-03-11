/**
 * Class b2ContactResult
 *
 *
 */
Box2D.Dynamics.Contacts.b2ContactResult = b2ContactResult = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ContactResult(){
      this.position = new b2Vec2();
      this.normal = new b2Vec2();
      this.id = new b2ContactID();

   }
   return b2ContactResult;
})();