/**
 * Class b2DynamicTreeNode
 *
 *
 */
Box2D.Collision.b2DynamicTreeNode = b2DynamicTreeNode = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2DynamicTreeNode(){
      this.aabb = new b2AABB();

   }

   /**
    * IsLeaf
    *
    * @param 
    *
    */
   b2DynamicTreeNode.prototype.IsLeaf = function () {
      return this.child1 == null;
   };
   return b2DynamicTreeNode;
})();