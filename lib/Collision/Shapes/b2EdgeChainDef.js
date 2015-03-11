/**
 * Class b2EdgeChainDef
 *
 *
 */
Box2D.Collision.Shapes.b2EdgeChainDef = b2EdgeChainDef = (function() {

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

   /**
    * b2EdgeChainDef
    *
    * @param 
    *
    */
   b2EdgeChainDef.prototype.b2EdgeChainDef = function () {
      this.vertexCount = 0;
      this.isALoop = true;
      this.vertices = [];
   };
   return b2EdgeChainDef;
})();