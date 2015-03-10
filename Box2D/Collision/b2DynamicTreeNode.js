function b2DynamicTreeNode(){
      this.aabb = new b2AABB();
}
//IsLeaf
b2DynamicTreeNode.prototype.IsLeaf = function () {
      return this.child1 == null;
   };