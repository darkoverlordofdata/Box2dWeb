
   /**
    *  Class b2ContactListener
    *
    * @param 
    *
    */
   b2ContactListener = Box2D.Dynamics.b2ContactListener = function b2ContactListener(){


   };
   b2ContactListener.constructor = b2ContactListener;


   /**
    * BeginContact
    *
    * @param contact
    *
    */
   b2ContactListener.prototype.BeginContact = function (contact) {};

   /**
    * EndContact
    *
    * @param contact
    *
    */
   b2ContactListener.prototype.EndContact = function (contact) {};

   /**
    * PreSolve
    *
    * @param contact
    * @param oldManifold
    *
    */
   b2ContactListener.prototype.PreSolve = function (contact, oldManifold) {};

   /**
    * PostSolve
    *
    * @param contact
    * @param impulse
    *
    */
   b2ContactListener.prototype.PostSolve = function (contact, impulse) {};
   b2ContactListener.b2_defaultListener = new b2ContactListener();