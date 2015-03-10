function b2Point(){
      this.p = new b2Vec2();
}
//Support
b2Point.prototype.Support = function (xf, vX, vY) {
      if (vX === undefined) vX = 0;
      if (vY === undefined) vY = 0;
      return this.p;
   };
//GetFirstVertex
b2Point.prototype.GetFirstVertex = function (xf) {
      return this.p;
   };