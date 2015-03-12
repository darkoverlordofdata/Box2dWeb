
    b2FixtureDef.prototype.shape           = null;
    b2FixtureDef.prototype.userData        = null;
    b2FixtureDef.prototype.friction        = 0.2;
    b2FixtureDef.prototype.restitution     = 0.0;
    b2FixtureDef.prototype.density         = 0.0;
    b2FixtureDef.prototype.isSensor        = false;
    b2FixtureDef.prototype.filter          = null;
    /**
    * Constructor
    *
    * @param 
    *
    */
   function b2FixtureDef(){
      this.filter = new b2FilterData();
      this.filter.categoryBits = 0x0001;
      this.filter.maskBits = 0xFFFF;
      this.filter.groupIndex = 0;
   }

