
   /**
    *  Class b2DynamicTreeNode
    *
    * @param 
    *
    */
   b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode = function b2DynamicTreeNode(){
      this.aabb = new b2AABB();

   };
   b2DynamicTreeNode.constructor = b2DynamicTreeNode;

   /**
    * IsLeaf
    *
    * @param 
    *
    */
   b2DynamicTreeNode.prototype.IsLeaf = function () {
      return this.child1 == null;
   };