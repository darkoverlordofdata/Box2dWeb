function ClipVertex(){
      this.v = new b2Vec2();
      this.id = new b2ContactID();
}
//Set
ClipVertex.prototype.Set = function (other) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
   };