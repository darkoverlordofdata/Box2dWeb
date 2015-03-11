/**
 * Class ClipVertex
 *
 *
 */
Box2D.Collision.ClipVertex = ClipVertex = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function ClipVertex(){
      this.v = new b2Vec2();
      this.id = new b2ContactID();

   }

   /**
    * Set
    *
    * @param other
    *
    */
   ClipVertex.prototype.Set = function (other) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
   };
   return ClipVertex;
})();