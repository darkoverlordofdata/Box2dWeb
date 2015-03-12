
   /**
    *  Class b2BoundValues
    *
    * @param 
    *
    */
   b2BoundValues = Box2D.Collision.b2BoundValues = function b2BoundValues(){

      this.lowerValues = new Vector_a2j_Number();
      this.lowerValues[0] = 0.0;
      this.lowerValues[1] = 0.0;
      this.upperValues = new Vector_a2j_Number();
      this.upperValues[0] = 0.0;
      this.upperValues[1] = 0.0;
   };
   b2BoundValues.constructor = b2BoundValues;