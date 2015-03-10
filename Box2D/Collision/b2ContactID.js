function b2ContactID(){
      this.features = new Features();
      this.features._m_id = this;
}
//b2ContactID
b2ContactID.prototype.b2ContactID = function () {
      this.features._m_id = this;
   };
//Set
b2ContactID.prototype.Set = function (id) {
      this.key = id._key;
   };
//Copy
b2ContactID.prototype.Copy = function () {
      var id = new b2ContactID();
      id.key = this.key;
      return id;
   };