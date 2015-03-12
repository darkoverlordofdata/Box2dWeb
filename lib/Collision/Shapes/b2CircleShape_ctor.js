    b2CircleShape.prototype.m_type          = b2Shape.e_circleShape;
    b2CircleShape.prototype.m_p             = null;
    /**
    * Constructor
    *
    * @param radius
    *
    */
   function b2CircleShape(radius){

      this.m_p = new b2Vec2();
      if (radius === undefined) radius = 0;
      //b2CircleShape.__super__.constructor.call(this);
      this.m_radius = radius;
   }

