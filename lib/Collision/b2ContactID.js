
   /**
    *  Class b2ContactID
    *
    * @param 
    *
    */
   b2ContactID = Box2D.Collision.b2ContactID = function b2ContactID() {
      this.features = new Features();
      this.features._m_id = this;
   };
   b2ContactID.constructor = b2ContactID;

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