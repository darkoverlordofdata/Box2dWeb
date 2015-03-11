/**
 * Class b2Vec3
 *
 *
 */
Box2D.Common.Math.b2Vec3 = b2Vec3 = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param x
    * @param y
    * @param z
    *
    */
   function b2Vec3(x, y, z){

      if (x === undefined) x = 0;
      if (y === undefined) y = 0;
      if (z === undefined) z = 0;
      this.x = x;
      this.y = y;
      this.z = z;
   }

   /**
    * b2Vec3
    *
    * @param x
    * @param y
    * @param z
    *
    */
   b2Vec3.prototype.b2Vec3 = function (x, y, z) {
      if (x === undefined) x = 0;
      if (y === undefined) y = 0;
      if (z === undefined) z = 0;
      this.x = x;
      this.y = y;
      this.z = z;
   };

   /**
    * SetZero
    *
    * @param 
    *
    */
   b2Vec3.prototype.SetZero = function () {
      this.x = this.y = this.z = 0.0;
   };

   /**
    * Set
    *
    * @param x
    * @param y
    * @param z
    *
    */
   b2Vec3.prototype.Set = function (x, y, z) {
      if (x === undefined) x = 0;
      if (y === undefined) y = 0;
      if (z === undefined) z = 0;
      this.x = x;
      this.y = y;
      this.z = z;
   };

   /**
    * SetV
    *
    * @param v
    *
    */
   b2Vec3.prototype.SetV = function (v) {
      this.x = v.x;
      this.y = v.y;
      this.z = v.z;
   };

   /**
    * GetNegative
    *
    * @param 
    *
    */
   b2Vec3.prototype.GetNegative = function () {
      return new b2Vec3((-this.x), (-this.y), (-this.z));
   };

   /**
    * NegativeSelf
    *
    * @param 
    *
    */
   b2Vec3.prototype.NegativeSelf = function () {
      this.x = (-this.x);
      this.y = (-this.y);
      this.z = (-this.z);
   };

   /**
    * Copy
    *
    * @param 
    *
    */
   b2Vec3.prototype.Copy = function () {
      return new b2Vec3(this.x, this.y, this.z);
   };

   /**
    * Add
    *
    * @param v
    *
    */
   b2Vec3.prototype.Add = function (v) {
      this.x += v.x;
      this.y += v.y;
      this.z += v.z;
   };

   /**
    * Subtract
    *
    * @param v
    *
    */
   b2Vec3.prototype.Subtract = function (v) {
      this.x -= v.x;
      this.y -= v.y;
      this.z -= v.z;
   };

   /**
    * Multiply
    *
    * @param a
    *
    */
   b2Vec3.prototype.Multiply = function (a) {
      if (a === undefined) a = 0;
      this.x *= a;
      this.y *= a;
      this.z *= a;
   };
   return b2Vec3;
})();