/**
 * Class b2Shape
 *
 *
 */
Box2D.Collision.Shapes.b2Shape = b2Shape = (function() {

   b2Shape.e_unknownShape = parseInt((-1));
   b2Shape.e_circleShape = 0;
   b2Shape.e_polygonShape = 1;
   b2Shape.e_edgeShape = 2;
   b2Shape.e_shapeTypeCount = 3;
   b2Shape.e_hitCollide = 1;
   b2Shape.e_missCollide = 0;
   b2Shape.e_startsInsideCollide = parseInt((-1));

   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2Shape(){

      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
   }

   /**
    * Static TestOverlap
    *
    * @param shape1
    * @param transform1
    * @param shape2
    * @param transform2
    *
    */
   b2Shape.TestOverlap = function (shape1, transform1, shape2, transform2) {
      var input = new b2DistanceInput();
      input.proxyA = new b2DistanceProxy();
      input.proxyA.Set(shape1);
      input.proxyB = new b2DistanceProxy();
      input.proxyB.Set(shape2);
      input.transformA = transform1;
      input.transformB = transform2;
      input.useRadii = true;
      var simplexCache = new b2SimplexCache();
      simplexCache.count = 0;
      var output = new b2DistanceOutput();
      b2Distance.Distance(output, simplexCache, input);
      return output.distance < 10.0 * Number.MIN_VALUE;
   };

   /**
    * Copy
    *
    * @param 
    *
    */
   b2Shape.prototype.Copy = function () {
      return null;
   };

   /**
    * Set
    *
    * @param other
    *
    */
   b2Shape.prototype.Set = function (other) {
      this.m_radius = other.m_radius;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2Shape.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * TestPoint
    *
    * @param xf
    * @param p
    *
    */
   b2Shape.prototype.TestPoint = function (xf, p) {
      return false;
   };

   /**
    * RayCast
    *
    * @param output
    * @param input
    * @param transform
    *
    */
   b2Shape.prototype.RayCast = function (output, input, transform) {
      return false;
   };

   /**
    * ComputeAABB
    *
    * @param aabb
    * @param xf
    *
    */
   b2Shape.prototype.ComputeAABB = function (aabb, xf) {};

   /**
    * ComputeMass
    *
    * @param massData
    * @param density
    *
    */
   b2Shape.prototype.ComputeMass = function (massData, density) {
      if (density === undefined) density = 0;
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
   b2Shape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      if (offset === undefined) offset = 0;
      return 0;
   };

   /**
    * b2Shape
    *
    * @param 
    *
    */
   b2Shape.prototype.b2Shape = function () {
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
   };
   return b2Shape;
})();