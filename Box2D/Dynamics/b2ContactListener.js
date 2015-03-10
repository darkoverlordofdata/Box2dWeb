//BeginContact
b2ContactListener.prototype.BeginContact = function (contact) {};
//EndContact
b2ContactListener.prototype.EndContact = function (contact) {};
//PreSolve
b2ContactListener.prototype.PreSolve = function (contact, oldManifold) {};
//PostSolve
b2ContactListener.prototype.PostSolve = function (contact, impulse) {};