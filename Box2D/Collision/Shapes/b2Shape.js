function b2Shape(){
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
}
//Copy
b2Shape.prototype.Copy = function () {
      return null;
   };
//Set
b2Shape.prototype.Set = function (other) {
      this.m_radius = other.m_radius;
   };
//GetType
b2Shape.prototype.GetType = function () {
      return this.m_type;
   };
//TestPoint
b2Shape.prototype.TestPoint = function (xf, p) {
      return false;
   };
//RayCast
b2Shape.prototype.RayCast = function (output, input, transform) {
      return false;
   };
//ComputeAABB
b2Shape.prototype.ComputeAABB = function (aabb, xf) {};
//ComputeMass
b2Shape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
   };
//ComputeSubmergedArea
b2Shape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      return 0;
   };
//b2Shape
b2Shape.prototype.b2Shape = function () {
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
   };