
   /**
    *  Class b2ContactResult
    *
    * @param 
    *
    */
   b2ContactResult = Box2D.Dynamics.Contacts.b2ContactResult = function b2ContactResult(){
      this.position = new b2Vec2();
      this.normal = new b2Vec2();
      this.id = new b2ContactID();

   };
   b2ContactResult.constructor = b2ContactResult;