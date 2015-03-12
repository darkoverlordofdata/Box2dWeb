/**
 * Class b2ContactID
 *
 *
 */
Box2D.Collision.b2ContactID = b2ContactID = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ContactID(){
      this.features = new Features();
      this.features._m_id = this;
   }

   /**
    * Set
    *
    * @param id
    *
    */
   b2ContactID.prototype.Set = function (id) {
      this.key = id._key;
   };

   /**
    * Copy
    *
    * @param 
    *
    */
   b2ContactID.prototype.Copy = function () {
      var id = new b2ContactID();
      id.key = this.key;
      return id;
   };
   return b2ContactID;
})();