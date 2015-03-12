/**
 * Class b2Color
 *
 *
 */
Box2D.Common.b2Color = b2Color = (function() {
'use strict;'

   /**
    * Constructor
    *
    * @param rr
    * @param gg
    * @param bb
    *
    */
   function b2Color(rr, gg, bb){
      this._r = 0;
      this._g = 0;
      this._b = 0;
      rr = rr || 0;
      gg = gg || 0;
      bb = bb || 0;
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
   }

   /**
    * Set
    *
    * @param rr
    * @param gg
    * @param bb
    *
    */
   b2Color.prototype.Set = function (rr, gg, bb) {
      rr = rr || 0;
      gg = gg || 0;
      bb = bb || 0;
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
   };
   return b2Color;
})();