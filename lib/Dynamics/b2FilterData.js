
   /**
    *  Class b2FilterData
    *
    * @param 
    *
    */
   b2FilterData = Box2D.Dynamics.b2FilterData = function b2FilterData() {
      this.categoryBits = 0x0001;
      this.maskBits = 0xFFFF;
      this.groupIndex = 0;

   };
   b2FilterData.constructor = b2FilterData;

   /**
    * Copy
    *
    * @param 
    *
    */
   b2FilterData.prototype.Copy = function () {
      var copy = new b2FilterData();
      copy.categoryBits = this.categoryBits;
      copy.maskBits = this.maskBits;
      copy.groupIndex = this.groupIndex;
      return copy;
   };