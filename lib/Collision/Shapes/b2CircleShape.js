/**
 * Class b2CircleShape
 *
 *
 */
b2CircleShape = Box2D.Collision.Shapes.b2CircleShape = (function(superClass) {
   extend(b2CircleShape, superClass);

   /**
    * Constructor
    *
    * @param radius
    *
    */
   function b2CircleShape(radius){

      this.m_p = new b2Vec2();
      if (radius === undefined) radius = 0;
      b2CircleShape.__super__.constructor.call(this);
      this.m_type = b2Shape.e_circleShape;
      this.m_radius = radius;
   }

   /**
    * Copy
    *
    * @param 
    *
    */
   b2CircleShape.prototype.Copy = function () {
      var s = new b2CircleShape();
      s.Set(this);
      return s;
   };

   /**
    * Set
    *
    * @param other
    *
    */
   b2CircleShape.prototype.Set = function (other) {
      b2CircleShape.__super__.Set.call(this, other);
      if (Box2D.is(other, b2CircleShape)) {
         var other2 = (other instanceof b2CircleShape ? other : null);
         this.m_p.SetV(other2.m_p);
      }
   };

   /**
    * TestPoint
    *
    * @param transform
    * @param p
    *
    */
   b2CircleShape.prototype.TestPoint = function (transform, p) {
      var tMat = transform.R;
      var dX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var dY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      dX = p.x - dX;
      dY = p.y - dY;
      return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
   };

   /**
    * RayCast
    *
    * @param output
    * @param input
    * @param transform
    *
    */
   b2CircleShape.prototype.RayCast = function (output, input, transform) {
      var tMat = transform.R;
      var positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      var sX = input.p1.x - positionX;
      var sY = input.p1.y - positionY;
      var b = (sX * sX + sY * sY) - this.m_radius * this.m_radius;
      var rX = input.p2.x - input.p1.x;
      var rY = input.p2.y - input.p1.y;
      var c = (sX * rX + sY * rY);
      var rr = (rX * rX + rY * rY);
      var sigma = c * c - rr * b;
      if (sigma < 0.0 || rr < Number.MIN_VALUE) {
         return false;
      }
      var a = (-(c + Math.sqrt(sigma)));
      if (0.0 <= a && a <= input.maxFraction * rr) {
         a /= rr;
         output.fraction = a;
         output.normal.x = sX + a * rX;
         output.normal.y = sY + a * rY;
         output.normal.Normalize();
         return true;
      }
      return false;
   };

   /**
    * ComputeAABB
    *
    * @param aabb
    * @param transform
    *
    */
   b2CircleShape.prototype.ComputeAABB = function (aabb, transform) {
      var tMat = transform.R;
      var pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
      aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
   };

   /**
    * ComputeMass
    *
    * @param massData
    * @param density
    *
    */
   b2CircleShape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
      massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
      massData.center.SetV(this.m_p);
      massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
   };

   /**
    * ComputeSubmergedArea
    *
    * @param normal
    * @param offset
    * @param xf
    * @param c
    *
    */
   b2CircleShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      var p = b2Math.MulX(xf, this.m_p);
      var l = (-(b2Math.Dot(normal, p) - offset));
      if (l < (-this.m_radius) + Number.MIN_VALUE) {
         return 0;
      }
      if (l > this.m_radius) {
         c.SetV(p);
         return Math.PI * this.m_radius * this.m_radius;
      }
      var r2 = this.m_radius * this.m_radius;
      var l2 = l * l;
      var area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
      var com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area);
      c.x = p.x + normal.x * com;
      c.y = p.y + normal.y * com;
      return area;
   };

   /**
    * GetLocalPosition
    *
    * @param 
    *
    */
   b2CircleShape.prototype.GetLocalPosition = function () {
      return this.m_p;
   };

   /**
    * SetLocalPosition
    *
    * @param position
    *
    */
   b2CircleShape.prototype.SetLocalPosition = function (position) {
      this.m_p.SetV(position);
   };

   /**
    * GetRadius
    *
    * @param 
    *
    */
   b2CircleShape.prototype.GetRadius = function () {
      return this.m_radius;
   };

   /**
    * SetRadius
    *
    * @param radius
    *
    */
   b2CircleShape.prototype.SetRadius = function (radius) {
      if (radius === undefined) radius = 0;
      this.m_radius = radius;
   };

   /**
    * b2CircleShape
    *
    * @param radius
    *
    */
   b2CircleShape.prototype.b2CircleShape = function (radius) {
      if (radius === undefined) radius = 0;
      this.__super.b2Shape.call(this);
      this.m_type = b2Shape.e_circleShape;
      this.m_radius = radius;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2CircleShape.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * b2Shape
    *
    * @param 
    *
    */
   b2CircleShape.prototype.b2Shape = function () {
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
   };
   return b2CircleShape;
})(b2Shape);