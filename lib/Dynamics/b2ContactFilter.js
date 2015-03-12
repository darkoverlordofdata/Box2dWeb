/**
 * Class b2ContactFilter
 *
 *
 */
Box2D.Dynamics.b2ContactFilter = b2ContactFilter = (function() {
'use strict;'


   /**
    * Constructor
    *
    * @param 
    *
    */
   function b2ContactFilter(){


   }

   /**
    * ShouldCollide
    *
    * @param fixtureA
    * @param fixtureB
    *
    */
   b2ContactFilter.prototype.ShouldCollide = function (fixtureA, fixtureB) {
      var filter1 = fixtureA.GetFilterData(),
          filter2 = fixtureB.GetFilterData();
      if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
         return filter1.groupIndex > 0;
      }
      var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
      return collide;
   };

   /**
    * RayCollide
    *
    * @param userData
    * @param fixture
    *
    */
   b2ContactFilter.prototype.RayCollide = function (userData, fixture) {
      if (!userData) return true;
      return this.ShouldCollide((userData instanceof b2Fixture ? userData : null), fixture);
   };
   b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
   return b2ContactFilter;
})();