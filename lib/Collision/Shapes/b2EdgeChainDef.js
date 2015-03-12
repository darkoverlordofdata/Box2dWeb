/**
 * Class b2EdgeChainDef
 *
 *
 */
Box2D.Collision.Shapes.b2EdgeChainDef = b2EdgeChainDef = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2EdgeChainDef(){

      this.vertexCount = 0;
      this.isALoop = true;
      this.vertices = [];
   }
   return b2EdgeChainDef;
})();