!function(e){if("object"==typeof exports&&"undefined"!=typeof module)module.exports=e();else if("function"==typeof define&&define.amd)define([],e);else{var f;"undefined"!=typeof window?f=window:"undefined"!=typeof global?f=global:"undefined"!=typeof self&&(f=self),f.Box2D=e()}}(function(){var define,module,exports;return (function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Collision.Shapes.b2CircleShape = (function() {
  b2CircleShape.prototype.radius = 0;

  b2CircleShape.prototype.type = '';

  function b2CircleShape(radius) {
    this.radius = radius;
    this.type = "circle";
    return;
  }

  return b2CircleShape;

})();

//# sourceMappingURL=b2CircleShape.js.map

},{"../../index":19}],2:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Collision.Shapes.b2PolygonShape = (function() {
  function b2PolygonShape() {}

  b2PolygonShape.prototype.type = '';

  b2PolygonShape.prototype.width = 0;

  b2PolygonShape.prototype.height = 0;

  b2PolygonShape.prototype.p1x = 0;

  b2PolygonShape.prototype.p1y = 0;

  b2PolygonShape.prototype.p2x = 0;

  b2PolygonShape.prototype.p2y = 0;

  b2PolygonShape.prototype.vertices = null;

  b2PolygonShape.prototype.SetAsBox = function(width, height) {
    this.type = "box";
    this.width = width;
    this.height = height;
  };

  b2PolygonShape.prototype.SetAsEdge = function(v1, v2) {
    this.type = "edge";
    this.p1x = v1.x;
    this.p1y = v1.y;
    this.p2x = v2.x;
    this.p2y = v2.y;
  };

  b2PolygonShape.prototype.SetAsArray = function(vec, length) {
    var i;
    this.type = "polygon";
    this.vertices = [];
    i = 0;
    while (i < length) {
      this.vertices.push(vec[i].x);
      this.vertices.push(vec[i].y);
      i++;
    }
  };

  return b2PolygonShape;

})();

//# sourceMappingURL=b2PolygonShape.js.map

},{"../../index":19}],3:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Mat22 = (function() {
  b2Mat22.prototype.col1 = null;

  b2Mat22.prototype.col2 = null;

  function b2Mat22() {
    this.col1 = new b2Vec2();
    this.col2 = new b2Vec2();
    this.SetIdentity();
  }

  b2Mat22.FromAngle = function(angle) {
    var mat;
    if (angle == null) {
      angle = 0;
    }
    mat = new b2Mat22();
    mat.Set(angle);
    return mat;
  };

  b2Mat22.FromVV = function(c1, c2) {
    var mat;
    mat = new b2Mat22();
    mat.SetVV(c1, c2);
    return mat;
  };

  b2Mat22.prototype.Set = function(angle) {
    var c, s;
    if (angle == null) {
      angle = 0;
    }
    c = Math.cos(angle);
    s = Math.sin(angle);
    this.col1.x = c;
    this.col2.x = -s;
    this.col1.y = s;
    this.col2.y = c;
  };

  b2Mat22.prototype.SetVV = function(c1, c2) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
  };

  b2Mat22.prototype.Copy = function() {
    var mat;
    mat = new b2Mat22();
    mat.SetM(this);
    return mat;
  };

  b2Mat22.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
  };

  b2Mat22.prototype.AddM = function(m) {
    this.col1.x += m.col1.x;
    this.col1.y += m.col1.y;
    this.col2.x += m.col2.x;
    this.col2.y += m.col2.y;
  };

  b2Mat22.prototype.SetIdentity = function() {
    this.col1.x = 1.0;
    this.col2.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 1.0;
  };

  b2Mat22.prototype.SetZero = function() {
    this.col1.x = 0.0;
    this.col2.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 0.0;
  };

  b2Mat22.prototype.GetAngle = function() {
    return Math.atan2(this.col1.y, this.col1.x);
  };

  b2Mat22.prototype.GetInverse = function(out) {
    var a, b, c, d, det;
    a = this.col1.x;
    b = this.col2.x;
    c = this.col1.y;
    d = this.col2.y;
    det = a * d - b * c;
    if (det !== 0.0) {
      det = 1.0 / det;
    }
    out.col1.x = det * d;
    out.col2.x = -det * b;
    out.col1.y = -det * c;
    out.col2.y = det * a;
    return out;
  };

  b2Mat22.prototype.Solve = function(out, bX, bY) {
    var a11, a12, a21, a22, det;
    if (bX == null) {
      bX = 0;
    }
    if (bY == null) {
      bY = 0;
    }
    a11 = this.col1.x;
    a12 = this.col2.x;
    a21 = this.col1.y;
    a22 = this.col2.y;
    det = a11 * a22 - a12 * a21;
    if (det !== 0.0) {
      det = 1.0 / det;
    }
    out.x = det * (a22 * bX - a12 * bY);
    out.y = det * (a11 * bY - a21 * bX);
    return out;
  };

  b2Mat22.prototype.Abs = function() {
    this.col1.Abs();
    this.col2.Abs();
  };

  return b2Mat22;

})();

//# sourceMappingURL=b2Mat22.js.map

},{"../../index":19}],4:[function(require,module,exports){
var Box2D, b2Mat22, b2Math, b2Transform, b2Vec2;

Box2D = require('../../index');

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Math = Box2D.Common.Math.b2Math;

b2Transform = Box2D.Common.Math.b2Transform;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Math = (function() {
  function b2Math() {}

  b2Math.IsValid = function(x) {
    if (x === void 0) {
      x = 0;
    }
    return isFinite(x);
  };

  b2Math.Dot = function(a, b) {
    return a.x * b.x + a.y * b.y;
  };

  b2Math.CrossVV = function(a, b) {
    return a.x * b.y - a.y * b.x;
  };

  b2Math.CrossVF = function(a, s) {
    var v;
    if (s === void 0) {
      s = 0;
    }
    v = new b2Vec2(s * a.y, -s * a.x);
    return v;
  };

  b2Math.CrossFV = function(s, a) {
    var v;
    if (s === void 0) {
      s = 0;
    }
    v = new b2Vec2(-s * a.y, s * a.x);
    return v;
  };

  b2Math.MulMV = function(A, v) {
    var u;
    if (v === void 0) {
      console.error("undefined 'v' in b2Math.MulMV");
    }
    u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
    return u;
  };

  b2Math.MulTMV = function(A, v) {
    var u;
    u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
    return u;
  };

  b2Math.MulX = function(T, v) {
    var a;
    a = b2Math.MulMV(T.R, v);
    a.x += T.position.x;
    a.y += T.position.y;
    return a;
  };

  b2Math.MulXT = function(T, v) {
    var a, tX;
    a = b2Math.SubtractVV(v, T.position);
    tX = a.x * T.R.col1.x + a.y * T.R.col1.y;
    a.y = a.x * T.R.col2.x + a.y * T.R.col2.y;
    a.x = tX;
    return a;
  };

  b2Math.AddVV = function(a, b) {
    var v;
    v = new b2Vec2(a.x + b.x, a.y + b.y);
    return v;
  };

  b2Math.SubtractVV = function(a, b) {
    var v;
    v = new b2Vec2(a.x - b.x, a.y - b.y);
    return v;
  };

  b2Math.Distance = function(a, b) {
    var cX, cY;
    cX = a.x - b.x;
    cY = a.y - b.y;
    return Math.sqrt(cX * cX + cY * cY);
  };

  b2Math.DistanceSquared = function(a, b) {
    var cX, cY;
    cX = a.x - b.x;
    cY = a.y - b.y;
    return cX * cX + cY * cY;
  };

  b2Math.MulFV = function(s, a) {
    var v;
    if (s === void 0) {
      s = 0;
    }
    v = new b2Vec2(s * a.x, s * a.y);
    return v;
  };

  b2Math.AddMM = function(A, B) {
    var C;
    C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
    return C;
  };

  b2Math.MulMM = function(A, B) {
    var C;
    C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
    return C;
  };

  b2Math.MulTMM = function(A, B) {
    var C, c1, c2;
    c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
    c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
    C = b2Mat22.FromVV(c1, c2);
    return C;
  };

  b2Math.Abs = function(a) {
    if (a === void 0) {
      a = 0;
    }
    if (a > 0.0) {
      return a;
    } else {
      return -a;
    }
  };

  b2Math.AbsV = function(a) {
    var b;
    b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
    return b;
  };

  b2Math.AbsM = function(A) {
    var B;
    B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
    return B;
  };

  b2Math.Min = function(a, b) {
    if (a === void 0) {
      a = 0;
    }
    if (b === void 0) {
      b = 0;
    }
    if (a < b) {
      return a;
    } else {
      return b;
    }
  };

  b2Math.MinV = function(a, b) {
    var c;
    c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
    return c;
  };

  b2Math.Max = function(a, b) {
    if (a === void 0) {
      a = 0;
    }
    if (b === void 0) {
      b = 0;
    }
    if (a > b) {
      return a;
    } else {
      return b;
    }
  };

  b2Math.MaxV = function(a, b) {
    var c;
    c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
    return c;
  };

  b2Math.Clamp = function(a, low, high) {
    if (a === void 0) {
      a = 0;
    }
    if (low === void 0) {
      low = 0;
    }
    if (high === void 0) {
      high = 0;
    }
    if (a < low) {
      return low;
    } else {
      if (a > high) {
        return high;
      } else {
        return a;
      }
    }
  };

  b2Math.ClampV = function(a, low, high) {
    return b2Math.MaxV(low, b2Math.MinV(a, high));
  };

  b2Math.Swap = function(a, b) {
    var tmp;
    tmp = a[0];
    a[0] = b[0];
    b[0] = tmp;
  };

  b2Math.Random = function() {
    return Math.random() * 2 - 1;
  };

  b2Math.RandomRange = function(lo, hi) {
    var r;
    if (lo === void 0) {
      lo = 0;
    }
    if (hi === void 0) {
      hi = 0;
    }
    r = Math.random();
    r = (hi - lo) * r + lo;
    return r;
  };

  b2Math.NextPowerOfTwo = function(x) {
    if (x === void 0) {
      x = 0;
    }
    x |= (x >> 1) & 0x7FFFFFFF;
    x |= (x >> 2) & 0x3FFFFFFF;
    x |= (x >> 4) & 0x0FFFFFFF;
    x |= (x >> 8) & 0x00FFFFFF;
    x |= (x >> 16) & 0x0000FFFF;
    return x + 1;
  };

  b2Math.IsPowerOfTwo = function(x) {
    var result;
    if (x === void 0) {
      x = 0;
    }
    result = x > 0 && (x & (x - 1)) === 0;
    return result;
  };

  b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);

  b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));

  b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);

  return b2Math;

})();

//# sourceMappingURL=b2Math.js.map

},{"../../index":19}],5:[function(require,module,exports){
var Box2D, b2Mat22, b2Vec2;

Box2D = require('../../index');

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Transform = (function() {
  b2Transform.prototype.position = null;

  b2Transform.prototype.R = null;

  function b2Transform(pos, r) {
    this.position = new b2Vec2();
    this.R = new b2Mat22();
    if (pos === void 0) {
      pos = null;
    }
    if (r === void 0) {
      r = null;
    }
    if (pos) {
      this.position.SetV(pos);
      this.R.SetM(r);
    }
    return;
  }

  b2Transform.prototype.Initialize = function(pos, r) {
    this.position.SetV(pos);
    this.R.SetM(r);
  };

  b2Transform.prototype.SetIdentity = function() {
    this.position.SetZero();
    this.R.SetIdentity();
  };

  b2Transform.prototype.Set = function(x) {
    this.position.SetV(x.position);
    this.R.SetM(x.R);
  };

  b2Transform.prototype.SetAngle = function() {
    return Math.atan2(this.R.col1.y, this.R.col1.x);
  };

  return b2Transform;

})();

//# sourceMappingURL=b2Transform.js.map

},{"../../index":19}],6:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Common.Math.b2Vec2 = (function() {
  b2Vec2.prototype.x = 0;

  b2Vec2.prototype.y = 0;

  function b2Vec2(x, y) {
    this.x = x != null ? x : 0;
    this.y = y != null ? y : 0;
  }

  b2Vec2.prototype.SetZero = function() {
    this.x = 0.0;
    this.y = 0.0;
  };

  b2Vec2.prototype.Set = function(x, y) {
    if (x == null) {
      x = 0;
    }
    if (y == null) {
      y = 0;
    }
    this.x = x;
    this.y = y;
  };

  b2Vec2.prototype.SetV = function(v) {
    if (v === void 0) {
      console.error("undefined 'v' in b2Vec2.SetV");
    }
    this.x = v.x;
    this.y = v.y;
  };

  b2Vec2.Make = function(x, y) {
    if (x == null) {
      x = 0;
    }
    if (y == null) {
      y = 0;
    }
    return new b2Vec2(x, y);
  };

  if (b2Vec2.Get === void 0) {
    b2Vec2.Get = b2Vec2.Make;
    b2Vec2._freeCache = [];
    b2Vec2.Free = function() {};
  }

  b2Vec2.prototype.Copy = function() {
    return new b2Vec2(this.x, this.y);
  };

  b2Vec2.prototype.Add = function(v) {
    if (v === void 0) {
      console.error("undefined 'v' in b2Vec2.Add");
    }
    this.x += v.x;
    this.y += v.y;
  };

  b2Vec2.prototype.Subtract = function(v) {
    if (v === void 0) {
      console.error("undefined 'v' in b2Vec2.Subtract");
    }
    this.x -= v.x;
    this.y -= v.y;
  };

  b2Vec2.prototype.Multiply = function(a) {
    if (a === void 0) {
      a = 0;
    }
    this.x *= a;
    this.y *= a;
  };

  b2Vec2.prototype.Length = function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  };

  b2Vec2.prototype.LengthSquared = function() {
    return this.x * this.x + this.y * this.y;
  };

  b2Vec2.prototype.Normalize = function() {
    var invLength, length;
    length = Math.sqrt(this.x * this.x + this.y * this.y);
    if (length < Number.MIN_VALUE) {
      return 0.0;
    }
    invLength = 1.0 / length;
    this.x *= invLength;
    this.y *= invLength;
    return length;
  };

  b2Vec2.prototype.NegativeSelf = function() {
    this.x = -this.x;
    this.y = -this.y;
  };

  return b2Vec2;

})();

//# sourceMappingURL=b2Vec2.js.map

},{"../../index":19}],7:[function(require,module,exports){
var Box2D, b2Fixture, b2Mat22, b2Math, b2Transform, b2Vec2;

Box2D = require('../index');

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Math = Box2D.Common.Math.b2Math;

b2Transform = Box2D.Common.Math.b2Transform;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Fixture = Box2D.Dynamics.b2Fixture;

Box2D.Dynamics.b2Body = (function() {
  b2Body.b2_staticBody = 0;

  b2Body.b2_kinematicBody = 1;

  b2Body.b2_dynamicBody = 1;

  b2Body.prototype.m_world = null;

  b2Body.prototype.m_bodyID = null;

  b2Body.prototype.m_userData = null;

  b2Body.prototype.m_xf = null;

  b2Body.prototype.m_fixtures = null;

  b2Body.prototype.m_active = false;

  function b2Body(bd, world) {
    var userData;
    userData = bd.userData;
    bd.userData = null;
    this.m_world = world;
    this.m_xf = new b2Transform(bd.position, b2Mat22.FromAngle(bd.angle));
    this.m_fixtures = [];
    this.m_active = bd.active;
    if (bd.type === b2Body.b2_staticBody) {
      bd.density = 0;
    }
    this.m_bodyID = window.ext.IDTK_SRV_BOX2D.makeCall('createBody', world.m_worldID, bd);
    this.m_userData = userData;
    bd.userData = userData;
  }

  b2Body.prototype.CreateFixture = function(def) {
    var fixture, fixtureID, userData;
    userData = def.userData;
    def.userData = null;
    fixtureID = window.ext.IDTK_SRV_BOX2D.makeCall('createFixture', this.m_world.m_worldID, this.m_bodyID, def);
    def.userData = userData;
    fixture = new b2Fixture(this, userData, fixtureID, def);
    this.m_world.m_fixturesList[fixtureID] = fixture;
    this.m_fixtures.push(fixture);
    return fixture;
  };

  b2Body.prototype.GetFixtureList = function() {
    if (this.m_fixtures.length === 0) {
      return null;
    }
    return this.m_fixtures[0];
  };

  b2Body.prototype.DestroyFixture = function(fixture) {
    window.ext.IDTK_SRV_BOX2D.makeCall('deleteFixture', this.m_world.m_worldID, fixture.m_fixtureID);
    delete this.m_world.m_fixturesList[fixture.m_fixtureID];
  };

  b2Body.prototype.SetPositionAndAngle = function(position, angle) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setBodyTransform', this.m_world.m_worldID, this.m_bodyID, position.x, position.y, angle);
    this.m_xf.R.Set(angle);
    this.m_xf.position.SetV(position);
  };

  b2Body.prototype.GetPosition = function() {
    return this.m_xf.position;
  };

  b2Body.prototype.SetPosition = function(position) {
    this.SetPositionAndAngle(position, this.GetAngle());
  };

  b2Body.prototype.GetLinearVelocity = function() {
    var v;
    v = window.ext.IDTK_SRV_BOX2D.makeCall('getLinearVelocity', this.m_world.m_worldID, this.m_bodyID);
    return new b2Vec2(v[0], v[1]);
  };

  b2Body.prototype.SetLinearVelocity = function(vel) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setLinearVelocity', this.m_world.m_worldID, this.m_bodyID, vel.x, vel.y);
  };

  b2Body.prototype.SetLinearDamping = function(damp) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setLinearDamping', this.m_world.m_worldID, this.m_bodyID, damp);
  };

  b2Body.prototype.GetWorldCenter = function() {
    var p;
    p = window.ext.IDTK_SRV_BOX2D.makeCall('getWorldCenter', this.m_world.m_worldID, this.m_bodyID);
    return new b2Vec2(p[0], p[1]);
  };

  b2Body.prototype.GetLocalCenter = function() {
    var p;
    p = window.ext.IDTK_SRV_BOX2D.makeCall('getLocalCenter', this.m_world.m_worldID, this.m_bodyID);
    return new b2Vec2(p[0], p[1]);
  };

  b2Body.prototype.GetLocalPoint = function(worldPoint) {
    return b2Math.MulXT(this.m_xf, worldPoint);
  };

  b2Body.prototype.GetUserData = function() {
    return this.m_userData;
  };

  b2Body.prototype.SetUserData = function(data) {
    this.m_userData = data;
  };

  b2Body.prototype.GetMass = function() {
    return window.ext.IDTK_SRV_BOX2D.makeCall('getMass', this.m_world.m_worldID, this.m_bodyID);
  };

  b2Body.prototype.IsAwake = function() {
    return window.ext.IDTK_SRV_BOX2D.makeCall('isAwake', this.m_world.m_worldID, this.m_bodyID);
  };

  b2Body.prototype.SetAwake = function(state) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setAwake', this.m_world.m_worldID, this.m_bodyID, state);
  };

  b2Body.prototype.GetAngularVelocity = function() {
    return window.ext.IDTK_SRV_BOX2D.makeCall('getAngularVelocity', this.m_world.m_worldID, this.m_bodyID);
  };

  b2Body.prototype.SetAngularVelocity = function(angvel) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setAngularVelocity', this.m_world.m_worldID, this.m_bodyID, angvel);
  };

  b2Body.prototype.SetFixedRotation = function(fixed) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setFixedRotation', this.m_world.m_worldID, this.m_bodyID, fixed);
  };

  b2Body.prototype.IsActive = function() {
    return this.m_active;
  };

  b2Body.prototype.SetActive = function(state) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setActive', this.m_world.m_worldID, this.m_bodyID, state);
    this.m_active = state;
  };

  b2Body.prototype.GetAngle = function() {
    return this.m_xf.R.GetAngle();
  };

  b2Body.prototype.SetAngle = function(angle) {
    if (angle === void 0) {
      angle = 0;
    }
    this.SetPositionAndAngle(this.GetPosition(), angle);
  };

  b2Body.prototype.SetType = function(type) {
    window.ext.IDTK_SRV_BOX2D.makeCall('setType', this.m_world.m_worldID, this.m_bodyID, type);
  };

  b2Body.prototype.GetContactList = function() {
    var contact, contacts, result, _i, _len;
    contacts = window.ext.IDTK_SRV_BOX2D.makeCall('getObjectContacts', this.m_world.m_worldID, this.m_bodyID);
    result = [];
    for (_i = 0, _len = contacts.length; _i < _len; _i++) {
      contact = contacts[_i];
      result.push(this.m_world.m_bodyList[contact]);
    }
    return result;
  };

  b2Body.prototype.GetWorld = function() {
    return this.m_world;
  };

  b2Body.prototype.ApplyImpulse = function(impulse, point, wake) {
    window.ext.IDTK_SRV_BOX2D.makeCall('applyImpulse', this.m_world.m_worldID, this.m_bodyID, impulse.x, impulse.y, point.x, point.y, wake);
  };

  b2Body.prototype.ApplyForceToCenter = function(force, wake) {
    window.ext.IDTK_SRV_BOX2D.makeCall('applyForceToCenter', this.m_world.m_worldID, this.m_bodyID, force.x, force.y, wake);
  };

  b2Body.prototype.ApplyForce = function(force, point, wake) {
    window.ext.IDTK_SRV_BOX2D.makeCall('applyForce', this.m_world.m_worldID, this.m_bodyID, force.x, force.y, point.x, point.y, wake);
  };

  b2Body.prototype.ApplyTorque = function(torque, wake) {
    window.ext.IDTK_SRV_BOX2D.makeCall('applyTorque', this.m_world.m_worldID, this.m_bodyID, torque, wake);
  };

  return b2Body;

})();

//# sourceMappingURL=b2Body.js.map

},{"../index":19}],8:[function(require,module,exports){
var Box2D, b2Body, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Body = Box2D.Dynamics.b2Body;

Box2D.Dynamics.b2BodyDef = (function() {
  b2BodyDef.prototype.position = null;

  b2BodyDef.prototype.linearVelocity = null;

  b2BodyDef.prototype.userData = null;

  b2BodyDef.prototype.angle = 0.0;

  b2BodyDef.prototype.linearVelocity = null;

  b2BodyDef.prototype.angularVelocity = 0.0;

  b2BodyDef.prototype.angularDamping = 0.0;

  b2BodyDef.prototype.allowSleep = true;

  b2BodyDef.prototype.awake = true;

  b2BodyDef.prototype.fixedRotation = false;

  b2BodyDef.prototype.bullet = false;

  b2BodyDef.prototype.type = 0;

  b2BodyDef.prototype.active = true;

  b2BodyDef.prototype.inertiaScale = 1.0;

  function b2BodyDef() {
    this.position = new b2Vec2(0, 0);
    this.linearVelocity = new b2Vec2();
    this.userData = null;
    this.angle = 0.0;
    this.linearVelocity.Set(0, 0);
    this.angularVelocity = 0.0;
    this.linearDamping = 0.0;
    this.angularDamping = 0.0;
    this.allowSleep = true;
    this.awake = true;
    this.fixedRotation = false;
    this.bullet = false;
    this.type = b2Body.b2_staticBody;
    this.active = true;
    this.inertiaScale = 1.0;
  }

  return b2BodyDef;

})();

//# sourceMappingURL=b2BodyDef.js.map

},{"../index":19}],9:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2DebugDraw = (function() {
  function b2DebugDraw() {}

  b2DebugDraw.e_aabbBit = 0x0004;

  b2DebugDraw.e_centerOfMassBit = 0x0010;

  b2DebugDraw.e_controllerBit = 0x0020;

  b2DebugDraw.e_jointBit = 0x0002;

  b2DebugDraw.e_pairBit = 0x0008;

  b2DebugDraw.e_shapeBit = 0x000;

  b2DebugDraw.prototype.AppendFlags = function() {};

  b2DebugDraw.prototype.ClearFlags = function() {};

  b2DebugDraw.prototype.DrawCircle = function() {};

  b2DebugDraw.prototype.DrawPolygon = function() {};

  b2DebugDraw.prototype.DrawSegment = function() {};

  b2DebugDraw.prototype.DrawSolidCircle = function() {};

  b2DebugDraw.prototype.DrawSolidPolygon = function() {};

  b2DebugDraw.prototype.DrawTransform = function() {};

  b2DebugDraw.prototype.GetAlpha = function() {};

  b2DebugDraw.prototype.GetDrawScale = function() {};

  b2DebugDraw.prototype.GetFillAlpha = function() {};

  b2DebugDraw.prototype.GetFlags = function() {};

  b2DebugDraw.prototype.GetLineThickness = function() {};

  b2DebugDraw.prototype.GetSprite = function() {};

  b2DebugDraw.prototype.GetXFormScale = function() {};

  b2DebugDraw.prototype.SetAlpha = function() {};

  b2DebugDraw.prototype.SetDrawScale = function() {};

  b2DebugDraw.prototype.SetFillAlpha = function() {};

  b2DebugDraw.prototype.SetFlags = function() {};

  b2DebugDraw.prototype.SetLineThickness = function() {};

  b2DebugDraw.prototype.SetSprite = function() {};

  b2DebugDraw.prototype.SetXFormScale = function() {};

  return b2DebugDraw;

})();

//# sourceMappingURL=b2DebugDraw.js.map

},{"../index":19}],10:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.b2Fixture = (function() {
  b2Fixture.prototype.m_body = null;

  b2Fixture.prototype.m_userData = null;

  b2Fixture.prototype.m_fixtureID = null;

  b2Fixture.prototype.m_shape = null;

  b2Fixture.prototype.m_isSensor = false;

  b2Fixture.prototype.m_density = null;

  b2Fixture.prototype.m_friction = null;

  b2Fixture.prototype.m_restitution = null;

  function b2Fixture(body, userData, fixtureID, def) {
    this.m_body = body;
    this.m_userData = userData;
    this.m_fixtureID = fixtureID;
    this.m_shape = {};
    this.m_shape.m_centroid = new b2Vec2();
    this.m_isSensor = false;
    this.m_density = def.density;
    this.m_friction = def.friction;
    this.m_restitution = def.restitution;
    this.m_isSensor = def.isSensor;
    return;
  }

  b2Fixture.prototype.GetBody = function() {
    return this.m_body;
  };

  b2Fixture.prototype.GetShape = function() {
    console.log("fixture.GetShape not yet supported in CocoonJS Box2D binding");
    return null;
  };

  b2Fixture.prototype.GetUserData = function() {
    return this.m_userData;
  };

  b2Fixture.prototype.SetSensor = function(isSensor) {
    this.m_isSensor = isSensor;
    window.ext.IDTK_SRV_BOX2D.makeCall("setSensor", this.m_body.m_world.m_worldID, this.m_fixtureID, this.m_isSensor);
  };

  b2Fixture.prototype.IsSensor = function() {
    return this.m_isSensor;
  };

  b2Fixture.prototype.SetDensity = function(density) {
    window.ext.IDTK_SRV_BOX2D.makeCall("setDensity", this.m_body.m_world.m_worldID, this.m_fixtureID, density);
    this.m_density = density;
  };

  b2Fixture.prototype.SetFriction = function(friction) {
    window.ext.IDTK_SRV_BOX2D.makeCall("setFriction", this.m_body.m_world.m_worldID, this.m_fixtureID, friction);
    this.m_friction = friction;
  };

  b2Fixture.prototype.SetRestitution = function(restitution) {
    window.ext.IDTK_SRV_BOX2D.makeCall("setRestitution", this.m_body.m_world.m_worldID, this.m_fixtureID, restitution);
    this.m_restitution = restitution;
  };

  b2Fixture.prototype.GetDensity = function() {
    return this.m_density;
  };

  b2Fixture.prototype.GetFriction = function() {
    return this.m_friction;
  };

  b2Fixture.prototype.GetRestitution = function() {
    return this.m_restitution;
  };

  return b2Fixture;

})();

//# sourceMappingURL=b2Fixture.js.map

},{"../index":19}],11:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2FixtureDef = (function() {
  b2FixtureDef.prototype.shape = null;

  b2FixtureDef.prototype.userData = null;

  b2FixtureDef.prototype.friction = 0.2;

  b2FixtureDef.prototype.restitution = 0.0;

  b2FixtureDef.prototype.density = 0.0;

  b2FixtureDef.prototype.isSensor = false;

  b2FixtureDef.prototype.filter = null;

  function b2FixtureDef() {
    this.shape = null;
    this.userData = null;
    this.friction = 0.2;
    this.restitution = 0.0;
    this.density = 0.0;
    this.isSensor = false;
    this.filter = {
      categoryBits: 1,
      maskBits: 0xFFFF,
      groupIndex: 0
    };
  }

  return b2FixtureDef;

})();

//# sourceMappingURL=b2FixtureDef.js.map

},{"../index":19}],12:[function(require,module,exports){
var Box2D, b2Body, b2Contact, b2Joint, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Body = Box2D.Dynamics.b2Body;

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Contact = Box2D.Dynamics.b2Contact;

Box2D.Dynamics.b2World = (function() {
  b2World.prototype.m_bodyList = null;

  b2World.prototype.m_jointList = null;

  b2World.prototype.m_fixturesList = null;

  b2World.prototype.m_contactListener = null;

  b2World.prototype.m_jointsList = null;

  b2World.prototype.m_worldID = 0;

  function b2World(gravity, doSleep) {
    this.m_bodyList = [];
    this.m_jointList = [];
    this.m_fixturesList = [];
    this.m_contactListener = null;
    this.m_jointsList = [];
    this.m_worldID = window.ext.IDTK_SRV_BOX2D.makeCall("createWorld", gravity.x, gravity.y, doSleep);
    return;
  }

  b2World.prototype.SetContactListener = function(listener) {
    this.m_contactListener = listener;
  };

  b2World.prototype.SetContactFilter = function(filter) {
    var callbackFunc;
    callbackFunc = function(a, b) {
      var fa, fb;
      fa = world.m_fixturesList[a];
      fb = world.m_fixturesList[b];
      return filter.ShouldCollide(fa, fb);
    };
    window.ext.IDTK_SRV_BOX2D.makeCall("setContactFilter", this.m_worldID, callbackFunc);
  };

  b2World.prototype.CreateBody = function(def) {
    var b;
    b = new b2Body(def, this);
    this.m_bodyList[b.m_bodyID] = b;
    return b;
  };

  b2World.prototype.DestroyBody = function(b) {
    var i;
    window.ext.IDTK_SRV_BOX2D.makeCall("deleteBody", this.m_worldID, b.m_bodyID);
    delete this.m_bodyList[b.m_bodyID];
    return;
    i = 0;
    while (i < b.m_fixtures.length) {
      delete this.m_fixturesList[b.m_fixtures[i].m_fixtureID];
      ++i;
    }
  };

  b2World.prototype.CreateJoint = function(def) {
    var bodyA, bodyB, joint, jointFunc;
    if (def.bodyA.m_bodyID === def.bodyB.m_bodyID) {
      return;
    }
    bodyA = def.bodyA;
    bodyB = def.bodyB;
    def.bodyA = bodyA.m_bodyID;
    def.bodyB = bodyB.m_bodyID;
    jointFunc = "createDistanceJoint";
    if (def.type === b2Joint.e_revoluteJoint) {
      jointFunc = "createRevoluteJoint";
    }
    joint = new b2Joint(def);
    joint.m_jointID = window.ext.IDTK_SRV_BOX2D.makeCall(jointFunc, this.m_worldID, def);
    def.bodyA = bodyA;
    def.bodyB = bodyB;
    this.m_jointsList.push(joint);
    return joint;
  };

  b2World.prototype.DestroyJoint = function(joint) {
    window.ext.IDTK_SRV_BOX2D.makeCall("destroyJoint", this.m_worldID, joint.m_jointID);
  };

  b2World.prototype.GetJointList = function() {
    var i;
    if (this.m_jointsList.length === 0) {
      return null;
    }
    i = 0;
    while (i < this.m_jointsList.length - 1) {
      this.m_jointsList[i].next = this.m_jointsList[i + 1];
      ++i;
    }
    this.m_jointsList[this.m_jointsList.length - 1].next = null;
    return this.m_jointsList[0];
  };

  b2World.prototype.SetContinuousPhysics = function(continuous) {
    window.ext.IDTK_SRV_BOX2D.makeCall("setContinuous", this.m_worldID, continuous);
  };

  b2World.prototype.SetGravity = function(gravity) {
    window.ext.IDTK_SRV_BOX2D.makeCall("setGravity", this.m_worldID, gravity.x, gravity.y);
  };

  b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
    var body, contacts, count, f1, f2, fix1, fix2, i, touching, transforms;
    i = void 0;
    transforms = window.ext.IDTK_SRV_BOX2D.makeCall("step", this.m_worldID, dt, velocityIterations, positionIterations);
    count = transforms[0];
    i = 1;
    while (i <= count * 4) {
      body = this.m_bodyList[transforms[i + 0]];
      if (body === null) {
        break;
      }
      body.m_xf.position.Set(transforms[i + 1], transforms[i + 2]);
      body.m_xf.R.Set(transforms[i + 3]);
      i += 4;
    }
    if (this.m_contactListener !== null) {
      contacts = window.ext.IDTK_SRV_BOX2D.makeCall("getLastContacts", this.m_worldID);
      count = contacts[0];
      i = 1;
      while (i <= count * 3) {
        f1 = contacts[i + 0];
        f2 = contacts[i + 1];
        touching = contacts[i + 2];
        fix1 = this.m_fixturesList[f1];
        fix2 = this.m_fixturesList[f2];
        if ((typeof fix1 === "undefined") || (typeof fix2 === "undefined")) {
          console.log("One of the fixtures in a contact DOESN'T EXIST!!");
          continue;
        }
        this.m_contactListener.BeginContact(new b2Contact(fix1, fix2, touching));
        i += 3;
      }
    }
  };

  b2World.prototype.ClearForces = function() {
    window.ext.IDTK_SRV_BOX2D.makeCall("clearForces", this.m_worldID);
  };

  b2World.prototype.SetDebugDraw = function() {};

  b2World.prototype.DrawDebugData = function() {};

  return b2World;

})();

//# sourceMappingURL=b2World.js.map

},{"../index":19}],13:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.b2Contact = (function() {
  b2Contact.prototype.m_fixtureA = null;

  b2Contact.prototype.m_fixtureB = null;

  b2Contact.prototype.m_touching = false;

  function b2Contact(fixtureA, fixtureB, touching) {
    this.m_fixtureA = fixtureA;
    this.m_fixtureB = fixtureB;
    this.m_touching = touching;
    return;
  }

  b2Contact.prototype.GetFixtureA = function() {
    return this.m_fixtureA;
  };

  b2Contact.prototype.GetFixtureB = function() {
    return this.m_fixtureB;
  };

  b2Contact.prototype.IsTouching = function() {
    return this.m_touching;
  };

  return b2Contact;

})();

//# sourceMappingURL=b2Contact.js.map

},{"../../index":19}],14:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.b2ContactFilter = (function() {
  function b2ContactFilter() {}

  return b2ContactFilter;

})();

//# sourceMappingURL=b2ContactFilter.js.map

},{"../../index":19}],15:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.b2ContactListener = (function() {
  function b2ContactListener() {}

  b2ContactListener.prototype.BeginContact = function() {};

  b2ContactListener.prototype.EndContact = function() {};

  b2ContactListener.prototype.PreSolve = function() {};

  b2ContactListener.prototype.PostSolve = function() {};

  b2ContactListener.b2_defaultListener = new b2ContactListener();

  return b2ContactListener;

})();

//# sourceMappingURL=b2ContactListener.js.map

},{"../../index":19}],16:[function(require,module,exports){
var Box2D, b2Joint, b2Vec2;

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Joints.b2DistanceJointDef = (function() {
  b2DistanceJointDef.prototype.type = 0;

  b2DistanceJointDef.prototype.localAnchorA = null;

  b2DistanceJointDef.prototype.localAnchorB = null;

  b2DistanceJointDef.prototype.userData = null;

  b2DistanceJointDef.prototype.bodyA = null;

  b2DistanceJointDef.prototype.bodyB = null;

  b2DistanceJointDef.prototype.length = 0;

  b2DistanceJointDef.prototype.frequencyHz = 0.0;

  b2DistanceJointDef.prototype.dampingRatio = 0.0;

  function b2DistanceJointDef(bA, bB, anchorA, anchorB) {
    var dX, dY;
    this.type = b2Joint.e_distanceJoint;
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.userData = null;
    if (bA !== void 0) {
      this.bodyA = bA;
    }
    if (bB !== void 0) {
      this.bodyB = bB;
    }
    if (anchorA !== void 0) {
      this.localAnchorA.SetV(anchorA);
    }
    if (anchorB !== void 0) {
      this.localAnchorB.SetV(anchorB);
    }
    if (anchorA !== void 0 && anchorB !== void 0) {
      dX = anchorB.x - anchorA.x;
      dY = anchorB.y - anchorA.y;
      this.length = Math.sqrt(dX * dX + dY * dY);
    }
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;
    return;
  }

  return b2DistanceJointDef;

})();

//# sourceMappingURL=b2DistanceJointDef.js.map

},{"../../index":19}],17:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.Joints.b2Joint = (function() {
  b2Joint.e_distanceJoint = 0;

  b2Joint.e_revoluteJoint = 1;

  b2Joint.prototype.userData = null;

  b2Joint.prototype.bodyA = null;

  b2Joint.prototype.bodyB = null;

  b2Joint.prototype.length = 0;

  b2Joint.prototype.next = null;

  function b2Joint(def) {
    this.bodyA = def.bodyA;
    this.bodyB = def.bodyB;
    this.userData = def.userData;
    this.type = def.type;
    this.next = null;
    return;
  }

  b2Joint.prototype.GetBodyA = function() {
    return this.bodyA;
  };

  b2Joint.prototype.GetBodyB = function() {
    return this.bodyB;
  };

  b2Joint.prototype.GetUserData = function() {
    return this.userData;
  };

  b2Joint.prototype.GetType = function() {
    return this.type;
  };

  b2Joint.prototype.GetNext = function() {
    return this.next;
  };

  return b2Joint;

})();

//# sourceMappingURL=b2Joint.js.map

},{"../../index":19}],18:[function(require,module,exports){
var Box2D, b2Joint, b2Vec2;

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Joints.b2RevoluteJointDef = (function() {
  b2RevoluteJointDef.prototype.type = 0;

  b2RevoluteJointDef.prototype.localAnchorA = null;

  b2RevoluteJointDef.prototype.localAnchorB = null;

  b2RevoluteJointDef.prototype.userData = null;

  b2RevoluteJointDef.prototype.bodyA = null;

  b2RevoluteJointDef.prototype.bodyB = null;

  b2RevoluteJointDef.prototype.referenceAngle = 0.0;

  b2RevoluteJointDef.prototype.lowerAngle = 0.0;

  b2RevoluteJointDef.prototype.upperAngle = 0.0;

  b2RevoluteJointDef.prototype.maxMotorTorque = 0.0;

  b2RevoluteJointDef.prototype.motorSpeed = 0.0;

  b2RevoluteJointDef.prototype.enableLimit = false;

  b2RevoluteJointDef.prototype.enableMotor = false;

  function b2RevoluteJointDef(bA, bB, anchorA, anchorB) {
    this.type = b2Joint.e_revoluteJoint;
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.userData = null;
    if (bA !== void 0) {
      this.bodyA = bA;
    }
    if (bB !== void 0) {
      this.bodyB = bB;
    }
    if (anchorA !== void 0) {
      this.localAnchorA.SetV(anchorA);
    }
    if (anchorB !== void 0) {
      this.localAnchorB.SetV(anchorB);
    }
    this.referenceAngle = 0.0;
    this.lowerAngle = 0.0;
    this.upperAngle = 0.0;
    this.maxMotorTorque = 0.0;
    this.motorSpeed = 0.0;
    this.enableLimit = false;
    this.enableMotor = false;
    return;
  }

  b2RevoluteJointDef.prototype.Initialize = function(bA, bB, anchor) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };

  return b2RevoluteJointDef;

})();

//# sourceMappingURL=b2RevoluteJointDef.js.map

},{"../../index":19}],19:[function(require,module,exports){

/*
 */
'use strict';
var Box2D;

module.exports = Box2D = (function() {
  function Box2D() {}

  Box2D.Common = (function() {
    function Common() {}

    return Common;

  })();

  return Box2D;

})();

Box2D.Common.Math = (function() {
  function Math() {}

  return Math;

})();

require('./common/math/b2Vec2');

require('./common/math/b2Mat22');

require('./common/math/b2Transform');

require('./common/math/b2Math');

Box2D.Collision = (function() {
  function Collision() {}

  return Collision;

})();

Box2D.Collision.Shapes = (function() {
  function Shapes() {}

  return Shapes;

})();

require('./collision/shapes/b2CircleShape');

require('./collision/shapes/b2PolygonShape');

Box2D.Dynamics = (function() {
  function Dynamics() {}

  return Dynamics;

})();

Box2D.Dynamics.Contacts = (function() {
  function Contacts() {}

  return Contacts;

})();

require('./dynamics/contacts/b2Contact');

require('./dynamics/contacts/b2ContactFilter');

require('./dynamics/contacts/b2ContactListener');

Box2D.Dynamics.Joints = (function() {
  function Joints() {}

  return Joints;

})();

require('./dynamics/joints/b2Joint');

require('./dynamics/joints/b2DistanceJointDef');

require('./dynamics/joints/b2RevoluteJointDef');

require('./dynamics/b2Fixture');

require('./dynamics/b2Body');

require('./dynamics/b2DebugDraw');

require('./dynamics/b2BodyDef');

require('./dynamics/b2FixtureDef');

require('./dynamics/b2World');

//# sourceMappingURL=index.js.map

},{"./collision/shapes/b2CircleShape":1,"./collision/shapes/b2PolygonShape":2,"./common/math/b2Mat22":3,"./common/math/b2Math":4,"./common/math/b2Transform":5,"./common/math/b2Vec2":6,"./dynamics/b2Body":7,"./dynamics/b2BodyDef":8,"./dynamics/b2DebugDraw":9,"./dynamics/b2Fixture":10,"./dynamics/b2FixtureDef":11,"./dynamics/b2World":12,"./dynamics/contacts/b2Contact":13,"./dynamics/contacts/b2ContactFilter":14,"./dynamics/contacts/b2ContactListener":15,"./dynamics/joints/b2DistanceJointDef":16,"./dynamics/joints/b2Joint":17,"./dynamics/joints/b2RevoluteJointDef":18}]},{},[19])(19)
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJ0bXAvbGliL2NvbGxpc2lvbi9zaGFwZXMvYjJDaXJjbGVTaGFwZS5qcyIsInRtcC9saWIvY29sbGlzaW9uL3NoYXBlcy9iMlBvbHlnb25TaGFwZS5qcyIsInRtcC9saWIvY29tbW9uL21hdGgvYjJNYXQyMi5qcyIsInRtcC9saWIvY29tbW9uL21hdGgvYjJNYXRoLmpzIiwidG1wL2xpYi9jb21tb24vbWF0aC9iMlRyYW5zZm9ybS5qcyIsInRtcC9saWIvY29tbW9uL21hdGgvYjJWZWMyLmpzIiwidG1wL2xpYi9keW5hbWljcy9iMkJvZHkuanMiLCJ0bXAvbGliL2R5bmFtaWNzL2IyQm9keURlZi5qcyIsInRtcC9saWIvZHluYW1pY3MvYjJEZWJ1Z0RyYXcuanMiLCJ0bXAvbGliL2R5bmFtaWNzL2IyRml4dHVyZS5qcyIsInRtcC9saWIvZHluYW1pY3MvYjJGaXh0dXJlRGVmLmpzIiwidG1wL2xpYi9keW5hbWljcy9iMldvcmxkLmpzIiwidG1wL2xpYi9keW5hbWljcy9jb250YWN0cy9iMkNvbnRhY3QuanMiLCJ0bXAvbGliL2R5bmFtaWNzL2NvbnRhY3RzL2IyQ29udGFjdEZpbHRlci5qcyIsInRtcC9saWIvZHluYW1pY3MvY29udGFjdHMvYjJDb250YWN0TGlzdGVuZXIuanMiLCJ0bXAvbGliL2R5bmFtaWNzL2pvaW50cy9iMkRpc3RhbmNlSm9pbnREZWYuanMiLCJ0bXAvbGliL2R5bmFtaWNzL2pvaW50cy9iMkpvaW50LmpzIiwidG1wL2xpYi9keW5hbWljcy9qb2ludHMvYjJSZXZvbHV0ZUpvaW50RGVmLmpzIiwidG1wL2xpYi9pbmRleC5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNwQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdERBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN6SUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdlJBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNyREE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNoSEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2pOQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM1REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3BFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzNGQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdENBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNwS0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ25DQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNaQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3RCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM1REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JEQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMzRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbiBlKHQsbixyKXtmdW5jdGlvbiBzKG8sdSl7aWYoIW5bb10pe2lmKCF0W29dKXt2YXIgYT10eXBlb2YgcmVxdWlyZT09XCJmdW5jdGlvblwiJiZyZXF1aXJlO2lmKCF1JiZhKXJldHVybiBhKG8sITApO2lmKGkpcmV0dXJuIGkobywhMCk7dmFyIGY9bmV3IEVycm9yKFwiQ2Fubm90IGZpbmQgbW9kdWxlICdcIitvK1wiJ1wiKTt0aHJvdyBmLmNvZGU9XCJNT0RVTEVfTk9UX0ZPVU5EXCIsZn12YXIgbD1uW29dPXtleHBvcnRzOnt9fTt0W29dWzBdLmNhbGwobC5leHBvcnRzLGZ1bmN0aW9uKGUpe3ZhciBuPXRbb11bMV1bZV07cmV0dXJuIHMobj9uOmUpfSxsLGwuZXhwb3J0cyxlLHQsbixyKX1yZXR1cm4gbltvXS5leHBvcnRzfXZhciBpPXR5cGVvZiByZXF1aXJlPT1cImZ1bmN0aW9uXCImJnJlcXVpcmU7Zm9yKHZhciBvPTA7bzxyLmxlbmd0aDtvKyspcyhyW29dKTtyZXR1cm4gc30pIiwidmFyIEJveDJEO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uLy4uL2luZGV4Jyk7XG5cbkJveDJELkNvbGxpc2lvbi5TaGFwZXMuYjJDaXJjbGVTaGFwZSA9IChmdW5jdGlvbigpIHtcbiAgYjJDaXJjbGVTaGFwZS5wcm90b3R5cGUucmFkaXVzID0gMDtcblxuICBiMkNpcmNsZVNoYXBlLnByb3RvdHlwZS50eXBlID0gJyc7XG5cbiAgZnVuY3Rpb24gYjJDaXJjbGVTaGFwZShyYWRpdXMpIHtcbiAgICB0aGlzLnJhZGl1cyA9IHJhZGl1cztcbiAgICB0aGlzLnR5cGUgPSBcImNpcmNsZVwiO1xuICAgIHJldHVybjtcbiAgfVxuXG4gIHJldHVybiBiMkNpcmNsZVNoYXBlO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMkNpcmNsZVNoYXBlLmpzLm1hcFxuIiwidmFyIEJveDJEO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uLy4uL2luZGV4Jyk7XG5cbkJveDJELkNvbGxpc2lvbi5TaGFwZXMuYjJQb2x5Z29uU2hhcGUgPSAoZnVuY3Rpb24oKSB7XG4gIGZ1bmN0aW9uIGIyUG9seWdvblNoYXBlKCkge31cblxuICBiMlBvbHlnb25TaGFwZS5wcm90b3R5cGUudHlwZSA9ICcnO1xuXG4gIGIyUG9seWdvblNoYXBlLnByb3RvdHlwZS53aWR0aCA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLmhlaWdodCA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLnAxeCA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLnAxeSA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLnAyeCA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLnAyeSA9IDA7XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLnZlcnRpY2VzID0gbnVsbDtcblxuICBiMlBvbHlnb25TaGFwZS5wcm90b3R5cGUuU2V0QXNCb3ggPSBmdW5jdGlvbih3aWR0aCwgaGVpZ2h0KSB7XG4gICAgdGhpcy50eXBlID0gXCJib3hcIjtcbiAgICB0aGlzLndpZHRoID0gd2lkdGg7XG4gICAgdGhpcy5oZWlnaHQgPSBoZWlnaHQ7XG4gIH07XG5cbiAgYjJQb2x5Z29uU2hhcGUucHJvdG90eXBlLlNldEFzRWRnZSA9IGZ1bmN0aW9uKHYxLCB2Mikge1xuICAgIHRoaXMudHlwZSA9IFwiZWRnZVwiO1xuICAgIHRoaXMucDF4ID0gdjEueDtcbiAgICB0aGlzLnAxeSA9IHYxLnk7XG4gICAgdGhpcy5wMnggPSB2Mi54O1xuICAgIHRoaXMucDJ5ID0gdjIueTtcbiAgfTtcblxuICBiMlBvbHlnb25TaGFwZS5wcm90b3R5cGUuU2V0QXNBcnJheSA9IGZ1bmN0aW9uKHZlYywgbGVuZ3RoKSB7XG4gICAgdmFyIGk7XG4gICAgdGhpcy50eXBlID0gXCJwb2x5Z29uXCI7XG4gICAgdGhpcy52ZXJ0aWNlcyA9IFtdO1xuICAgIGkgPSAwO1xuICAgIHdoaWxlIChpIDwgbGVuZ3RoKSB7XG4gICAgICB0aGlzLnZlcnRpY2VzLnB1c2godmVjW2ldLngpO1xuICAgICAgdGhpcy52ZXJ0aWNlcy5wdXNoKHZlY1tpXS55KTtcbiAgICAgIGkrKztcbiAgICB9XG4gIH07XG5cbiAgcmV0dXJuIGIyUG9seWdvblNoYXBlO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMlBvbHlnb25TaGFwZS5qcy5tYXBcbiIsInZhciBCb3gyRCwgYjJWZWMyO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uLy4uL2luZGV4Jyk7XG5cbmIyVmVjMiA9IEJveDJELkNvbW1vbi5NYXRoLmIyVmVjMjtcblxuQm94MkQuQ29tbW9uLk1hdGguYjJNYXQyMiA9IChmdW5jdGlvbigpIHtcbiAgYjJNYXQyMi5wcm90b3R5cGUuY29sMSA9IG51bGw7XG5cbiAgYjJNYXQyMi5wcm90b3R5cGUuY29sMiA9IG51bGw7XG5cbiAgZnVuY3Rpb24gYjJNYXQyMigpIHtcbiAgICB0aGlzLmNvbDEgPSBuZXcgYjJWZWMyKCk7XG4gICAgdGhpcy5jb2wyID0gbmV3IGIyVmVjMigpO1xuICAgIHRoaXMuU2V0SWRlbnRpdHkoKTtcbiAgfVxuXG4gIGIyTWF0MjIuRnJvbUFuZ2xlID0gZnVuY3Rpb24oYW5nbGUpIHtcbiAgICB2YXIgbWF0O1xuICAgIGlmIChhbmdsZSA9PSBudWxsKSB7XG4gICAgICBhbmdsZSA9IDA7XG4gICAgfVxuICAgIG1hdCA9IG5ldyBiMk1hdDIyKCk7XG4gICAgbWF0LlNldChhbmdsZSk7XG4gICAgcmV0dXJuIG1hdDtcbiAgfTtcblxuICBiMk1hdDIyLkZyb21WViA9IGZ1bmN0aW9uKGMxLCBjMikge1xuICAgIHZhciBtYXQ7XG4gICAgbWF0ID0gbmV3IGIyTWF0MjIoKTtcbiAgICBtYXQuU2V0VlYoYzEsIGMyKTtcbiAgICByZXR1cm4gbWF0O1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLlNldCA9IGZ1bmN0aW9uKGFuZ2xlKSB7XG4gICAgdmFyIGMsIHM7XG4gICAgaWYgKGFuZ2xlID09IG51bGwpIHtcbiAgICAgIGFuZ2xlID0gMDtcbiAgICB9XG4gICAgYyA9IE1hdGguY29zKGFuZ2xlKTtcbiAgICBzID0gTWF0aC5zaW4oYW5nbGUpO1xuICAgIHRoaXMuY29sMS54ID0gYztcbiAgICB0aGlzLmNvbDIueCA9IC1zO1xuICAgIHRoaXMuY29sMS55ID0gcztcbiAgICB0aGlzLmNvbDIueSA9IGM7XG4gIH07XG5cbiAgYjJNYXQyMi5wcm90b3R5cGUuU2V0VlYgPSBmdW5jdGlvbihjMSwgYzIpIHtcbiAgICB0aGlzLmNvbDEuU2V0VihjMSk7XG4gICAgdGhpcy5jb2wyLlNldFYoYzIpO1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLkNvcHkgPSBmdW5jdGlvbigpIHtcbiAgICB2YXIgbWF0O1xuICAgIG1hdCA9IG5ldyBiMk1hdDIyKCk7XG4gICAgbWF0LlNldE0odGhpcyk7XG4gICAgcmV0dXJuIG1hdDtcbiAgfTtcblxuICBiMk1hdDIyLnByb3RvdHlwZS5TZXRNID0gZnVuY3Rpb24obSkge1xuICAgIHRoaXMuY29sMS5TZXRWKG0uY29sMSk7XG4gICAgdGhpcy5jb2wyLlNldFYobS5jb2wyKTtcbiAgfTtcblxuICBiMk1hdDIyLnByb3RvdHlwZS5BZGRNID0gZnVuY3Rpb24obSkge1xuICAgIHRoaXMuY29sMS54ICs9IG0uY29sMS54O1xuICAgIHRoaXMuY29sMS55ICs9IG0uY29sMS55O1xuICAgIHRoaXMuY29sMi54ICs9IG0uY29sMi54O1xuICAgIHRoaXMuY29sMi55ICs9IG0uY29sMi55O1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLlNldElkZW50aXR5ID0gZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5jb2wxLnggPSAxLjA7XG4gICAgdGhpcy5jb2wyLnggPSAwLjA7XG4gICAgdGhpcy5jb2wxLnkgPSAwLjA7XG4gICAgdGhpcy5jb2wyLnkgPSAxLjA7XG4gIH07XG5cbiAgYjJNYXQyMi5wcm90b3R5cGUuU2V0WmVybyA9IGZ1bmN0aW9uKCkge1xuICAgIHRoaXMuY29sMS54ID0gMC4wO1xuICAgIHRoaXMuY29sMi54ID0gMC4wO1xuICAgIHRoaXMuY29sMS55ID0gMC4wO1xuICAgIHRoaXMuY29sMi55ID0gMC4wO1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLkdldEFuZ2xlID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIE1hdGguYXRhbjIodGhpcy5jb2wxLnksIHRoaXMuY29sMS54KTtcbiAgfTtcblxuICBiMk1hdDIyLnByb3RvdHlwZS5HZXRJbnZlcnNlID0gZnVuY3Rpb24ob3V0KSB7XG4gICAgdmFyIGEsIGIsIGMsIGQsIGRldDtcbiAgICBhID0gdGhpcy5jb2wxLng7XG4gICAgYiA9IHRoaXMuY29sMi54O1xuICAgIGMgPSB0aGlzLmNvbDEueTtcbiAgICBkID0gdGhpcy5jb2wyLnk7XG4gICAgZGV0ID0gYSAqIGQgLSBiICogYztcbiAgICBpZiAoZGV0ICE9PSAwLjApIHtcbiAgICAgIGRldCA9IDEuMCAvIGRldDtcbiAgICB9XG4gICAgb3V0LmNvbDEueCA9IGRldCAqIGQ7XG4gICAgb3V0LmNvbDIueCA9IC1kZXQgKiBiO1xuICAgIG91dC5jb2wxLnkgPSAtZGV0ICogYztcbiAgICBvdXQuY29sMi55ID0gZGV0ICogYTtcbiAgICByZXR1cm4gb3V0O1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLlNvbHZlID0gZnVuY3Rpb24ob3V0LCBiWCwgYlkpIHtcbiAgICB2YXIgYTExLCBhMTIsIGEyMSwgYTIyLCBkZXQ7XG4gICAgaWYgKGJYID09IG51bGwpIHtcbiAgICAgIGJYID0gMDtcbiAgICB9XG4gICAgaWYgKGJZID09IG51bGwpIHtcbiAgICAgIGJZID0gMDtcbiAgICB9XG4gICAgYTExID0gdGhpcy5jb2wxLng7XG4gICAgYTEyID0gdGhpcy5jb2wyLng7XG4gICAgYTIxID0gdGhpcy5jb2wxLnk7XG4gICAgYTIyID0gdGhpcy5jb2wyLnk7XG4gICAgZGV0ID0gYTExICogYTIyIC0gYTEyICogYTIxO1xuICAgIGlmIChkZXQgIT09IDAuMCkge1xuICAgICAgZGV0ID0gMS4wIC8gZGV0O1xuICAgIH1cbiAgICBvdXQueCA9IGRldCAqIChhMjIgKiBiWCAtIGExMiAqIGJZKTtcbiAgICBvdXQueSA9IGRldCAqIChhMTEgKiBiWSAtIGEyMSAqIGJYKTtcbiAgICByZXR1cm4gb3V0O1xuICB9O1xuXG4gIGIyTWF0MjIucHJvdG90eXBlLkFicyA9IGZ1bmN0aW9uKCkge1xuICAgIHRoaXMuY29sMS5BYnMoKTtcbiAgICB0aGlzLmNvbDIuQWJzKCk7XG4gIH07XG5cbiAgcmV0dXJuIGIyTWF0MjI7XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIyTWF0MjIuanMubWFwXG4iLCJ2YXIgQm94MkQsIGIyTWF0MjIsIGIyTWF0aCwgYjJUcmFuc2Zvcm0sIGIyVmVjMjtcblxuQm94MkQgPSByZXF1aXJlKCcuLi8uLi9pbmRleCcpO1xuXG5iMk1hdDIyID0gQm94MkQuQ29tbW9uLk1hdGguYjJNYXQyMjtcblxuYjJNYXRoID0gQm94MkQuQ29tbW9uLk1hdGguYjJNYXRoO1xuXG5iMlRyYW5zZm9ybSA9IEJveDJELkNvbW1vbi5NYXRoLmIyVHJhbnNmb3JtO1xuXG5iMlZlYzIgPSBCb3gyRC5Db21tb24uTWF0aC5iMlZlYzI7XG5cbkJveDJELkNvbW1vbi5NYXRoLmIyTWF0aCA9IChmdW5jdGlvbigpIHtcbiAgZnVuY3Rpb24gYjJNYXRoKCkge31cblxuICBiMk1hdGguSXNWYWxpZCA9IGZ1bmN0aW9uKHgpIHtcbiAgICBpZiAoeCA9PT0gdm9pZCAwKSB7XG4gICAgICB4ID0gMDtcbiAgICB9XG4gICAgcmV0dXJuIGlzRmluaXRlKHgpO1xuICB9O1xuXG4gIGIyTWF0aC5Eb3QgPSBmdW5jdGlvbihhLCBiKSB7XG4gICAgcmV0dXJuIGEueCAqIGIueCArIGEueSAqIGIueTtcbiAgfTtcblxuICBiMk1hdGguQ3Jvc3NWViA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICByZXR1cm4gYS54ICogYi55IC0gYS55ICogYi54O1xuICB9O1xuXG4gIGIyTWF0aC5Dcm9zc1ZGID0gZnVuY3Rpb24oYSwgcykge1xuICAgIHZhciB2O1xuICAgIGlmIChzID09PSB2b2lkIDApIHtcbiAgICAgIHMgPSAwO1xuICAgIH1cbiAgICB2ID0gbmV3IGIyVmVjMihzICogYS55LCAtcyAqIGEueCk7XG4gICAgcmV0dXJuIHY7XG4gIH07XG5cbiAgYjJNYXRoLkNyb3NzRlYgPSBmdW5jdGlvbihzLCBhKSB7XG4gICAgdmFyIHY7XG4gICAgaWYgKHMgPT09IHZvaWQgMCkge1xuICAgICAgcyA9IDA7XG4gICAgfVxuICAgIHYgPSBuZXcgYjJWZWMyKC1zICogYS55LCBzICogYS54KTtcbiAgICByZXR1cm4gdjtcbiAgfTtcblxuICBiMk1hdGguTXVsTVYgPSBmdW5jdGlvbihBLCB2KSB7XG4gICAgdmFyIHU7XG4gICAgaWYgKHYgPT09IHZvaWQgMCkge1xuICAgICAgY29uc29sZS5lcnJvcihcInVuZGVmaW5lZCAndicgaW4gYjJNYXRoLk11bE1WXCIpO1xuICAgIH1cbiAgICB1ID0gbmV3IGIyVmVjMihBLmNvbDEueCAqIHYueCArIEEuY29sMi54ICogdi55LCBBLmNvbDEueSAqIHYueCArIEEuY29sMi55ICogdi55KTtcbiAgICByZXR1cm4gdTtcbiAgfTtcblxuICBiMk1hdGguTXVsVE1WID0gZnVuY3Rpb24oQSwgdikge1xuICAgIHZhciB1O1xuICAgIHUgPSBuZXcgYjJWZWMyKGIyTWF0aC5Eb3QodiwgQS5jb2wxKSwgYjJNYXRoLkRvdCh2LCBBLmNvbDIpKTtcbiAgICByZXR1cm4gdTtcbiAgfTtcblxuICBiMk1hdGguTXVsWCA9IGZ1bmN0aW9uKFQsIHYpIHtcbiAgICB2YXIgYTtcbiAgICBhID0gYjJNYXRoLk11bE1WKFQuUiwgdik7XG4gICAgYS54ICs9IFQucG9zaXRpb24ueDtcbiAgICBhLnkgKz0gVC5wb3NpdGlvbi55O1xuICAgIHJldHVybiBhO1xuICB9O1xuXG4gIGIyTWF0aC5NdWxYVCA9IGZ1bmN0aW9uKFQsIHYpIHtcbiAgICB2YXIgYSwgdFg7XG4gICAgYSA9IGIyTWF0aC5TdWJ0cmFjdFZWKHYsIFQucG9zaXRpb24pO1xuICAgIHRYID0gYS54ICogVC5SLmNvbDEueCArIGEueSAqIFQuUi5jb2wxLnk7XG4gICAgYS55ID0gYS54ICogVC5SLmNvbDIueCArIGEueSAqIFQuUi5jb2wyLnk7XG4gICAgYS54ID0gdFg7XG4gICAgcmV0dXJuIGE7XG4gIH07XG5cbiAgYjJNYXRoLkFkZFZWID0gZnVuY3Rpb24oYSwgYikge1xuICAgIHZhciB2O1xuICAgIHYgPSBuZXcgYjJWZWMyKGEueCArIGIueCwgYS55ICsgYi55KTtcbiAgICByZXR1cm4gdjtcbiAgfTtcblxuICBiMk1hdGguU3VidHJhY3RWViA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgdjtcbiAgICB2ID0gbmV3IGIyVmVjMihhLnggLSBiLngsIGEueSAtIGIueSk7XG4gICAgcmV0dXJuIHY7XG4gIH07XG5cbiAgYjJNYXRoLkRpc3RhbmNlID0gZnVuY3Rpb24oYSwgYikge1xuICAgIHZhciBjWCwgY1k7XG4gICAgY1ggPSBhLnggLSBiLng7XG4gICAgY1kgPSBhLnkgLSBiLnk7XG4gICAgcmV0dXJuIE1hdGguc3FydChjWCAqIGNYICsgY1kgKiBjWSk7XG4gIH07XG5cbiAgYjJNYXRoLkRpc3RhbmNlU3F1YXJlZCA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgY1gsIGNZO1xuICAgIGNYID0gYS54IC0gYi54O1xuICAgIGNZID0gYS55IC0gYi55O1xuICAgIHJldHVybiBjWCAqIGNYICsgY1kgKiBjWTtcbiAgfTtcblxuICBiMk1hdGguTXVsRlYgPSBmdW5jdGlvbihzLCBhKSB7XG4gICAgdmFyIHY7XG4gICAgaWYgKHMgPT09IHZvaWQgMCkge1xuICAgICAgcyA9IDA7XG4gICAgfVxuICAgIHYgPSBuZXcgYjJWZWMyKHMgKiBhLngsIHMgKiBhLnkpO1xuICAgIHJldHVybiB2O1xuICB9O1xuXG4gIGIyTWF0aC5BZGRNTSA9IGZ1bmN0aW9uKEEsIEIpIHtcbiAgICB2YXIgQztcbiAgICBDID0gYjJNYXQyMi5Gcm9tVlYoYjJNYXRoLkFkZFZWKEEuY29sMSwgQi5jb2wxKSwgYjJNYXRoLkFkZFZWKEEuY29sMiwgQi5jb2wyKSk7XG4gICAgcmV0dXJuIEM7XG4gIH07XG5cbiAgYjJNYXRoLk11bE1NID0gZnVuY3Rpb24oQSwgQikge1xuICAgIHZhciBDO1xuICAgIEMgPSBiMk1hdDIyLkZyb21WVihiMk1hdGguTXVsTVYoQSwgQi5jb2wxKSwgYjJNYXRoLk11bE1WKEEsIEIuY29sMikpO1xuICAgIHJldHVybiBDO1xuICB9O1xuXG4gIGIyTWF0aC5NdWxUTU0gPSBmdW5jdGlvbihBLCBCKSB7XG4gICAgdmFyIEMsIGMxLCBjMjtcbiAgICBjMSA9IG5ldyBiMlZlYzIoYjJNYXRoLkRvdChBLmNvbDEsIEIuY29sMSksIGIyTWF0aC5Eb3QoQS5jb2wyLCBCLmNvbDEpKTtcbiAgICBjMiA9IG5ldyBiMlZlYzIoYjJNYXRoLkRvdChBLmNvbDEsIEIuY29sMiksIGIyTWF0aC5Eb3QoQS5jb2wyLCBCLmNvbDIpKTtcbiAgICBDID0gYjJNYXQyMi5Gcm9tVlYoYzEsIGMyKTtcbiAgICByZXR1cm4gQztcbiAgfTtcblxuICBiMk1hdGguQWJzID0gZnVuY3Rpb24oYSkge1xuICAgIGlmIChhID09PSB2b2lkIDApIHtcbiAgICAgIGEgPSAwO1xuICAgIH1cbiAgICBpZiAoYSA+IDAuMCkge1xuICAgICAgcmV0dXJuIGE7XG4gICAgfSBlbHNlIHtcbiAgICAgIHJldHVybiAtYTtcbiAgICB9XG4gIH07XG5cbiAgYjJNYXRoLkFic1YgPSBmdW5jdGlvbihhKSB7XG4gICAgdmFyIGI7XG4gICAgYiA9IG5ldyBiMlZlYzIoYjJNYXRoLkFicyhhLngpLCBiMk1hdGguQWJzKGEueSkpO1xuICAgIHJldHVybiBiO1xuICB9O1xuXG4gIGIyTWF0aC5BYnNNID0gZnVuY3Rpb24oQSkge1xuICAgIHZhciBCO1xuICAgIEIgPSBiMk1hdDIyLkZyb21WVihiMk1hdGguQWJzVihBLmNvbDEpLCBiMk1hdGguQWJzVihBLmNvbDIpKTtcbiAgICByZXR1cm4gQjtcbiAgfTtcblxuICBiMk1hdGguTWluID0gZnVuY3Rpb24oYSwgYikge1xuICAgIGlmIChhID09PSB2b2lkIDApIHtcbiAgICAgIGEgPSAwO1xuICAgIH1cbiAgICBpZiAoYiA9PT0gdm9pZCAwKSB7XG4gICAgICBiID0gMDtcbiAgICB9XG4gICAgaWYgKGEgPCBiKSB7XG4gICAgICByZXR1cm4gYTtcbiAgICB9IGVsc2Uge1xuICAgICAgcmV0dXJuIGI7XG4gICAgfVxuICB9O1xuXG4gIGIyTWF0aC5NaW5WID0gZnVuY3Rpb24oYSwgYikge1xuICAgIHZhciBjO1xuICAgIGMgPSBuZXcgYjJWZWMyKGIyTWF0aC5NaW4oYS54LCBiLngpLCBiMk1hdGguTWluKGEueSwgYi55KSk7XG4gICAgcmV0dXJuIGM7XG4gIH07XG5cbiAgYjJNYXRoLk1heCA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICBpZiAoYSA9PT0gdm9pZCAwKSB7XG4gICAgICBhID0gMDtcbiAgICB9XG4gICAgaWYgKGIgPT09IHZvaWQgMCkge1xuICAgICAgYiA9IDA7XG4gICAgfVxuICAgIGlmIChhID4gYikge1xuICAgICAgcmV0dXJuIGE7XG4gICAgfSBlbHNlIHtcbiAgICAgIHJldHVybiBiO1xuICAgIH1cbiAgfTtcblxuICBiMk1hdGguTWF4ViA9IGZ1bmN0aW9uKGEsIGIpIHtcbiAgICB2YXIgYztcbiAgICBjID0gbmV3IGIyVmVjMihiMk1hdGguTWF4KGEueCwgYi54KSwgYjJNYXRoLk1heChhLnksIGIueSkpO1xuICAgIHJldHVybiBjO1xuICB9O1xuXG4gIGIyTWF0aC5DbGFtcCA9IGZ1bmN0aW9uKGEsIGxvdywgaGlnaCkge1xuICAgIGlmIChhID09PSB2b2lkIDApIHtcbiAgICAgIGEgPSAwO1xuICAgIH1cbiAgICBpZiAobG93ID09PSB2b2lkIDApIHtcbiAgICAgIGxvdyA9IDA7XG4gICAgfVxuICAgIGlmIChoaWdoID09PSB2b2lkIDApIHtcbiAgICAgIGhpZ2ggPSAwO1xuICAgIH1cbiAgICBpZiAoYSA8IGxvdykge1xuICAgICAgcmV0dXJuIGxvdztcbiAgICB9IGVsc2Uge1xuICAgICAgaWYgKGEgPiBoaWdoKSB7XG4gICAgICAgIHJldHVybiBoaWdoO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0dXJuIGE7XG4gICAgICB9XG4gICAgfVxuICB9O1xuXG4gIGIyTWF0aC5DbGFtcFYgPSBmdW5jdGlvbihhLCBsb3csIGhpZ2gpIHtcbiAgICByZXR1cm4gYjJNYXRoLk1heFYobG93LCBiMk1hdGguTWluVihhLCBoaWdoKSk7XG4gIH07XG5cbiAgYjJNYXRoLlN3YXAgPSBmdW5jdGlvbihhLCBiKSB7XG4gICAgdmFyIHRtcDtcbiAgICB0bXAgPSBhWzBdO1xuICAgIGFbMF0gPSBiWzBdO1xuICAgIGJbMF0gPSB0bXA7XG4gIH07XG5cbiAgYjJNYXRoLlJhbmRvbSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiBNYXRoLnJhbmRvbSgpICogMiAtIDE7XG4gIH07XG5cbiAgYjJNYXRoLlJhbmRvbVJhbmdlID0gZnVuY3Rpb24obG8sIGhpKSB7XG4gICAgdmFyIHI7XG4gICAgaWYgKGxvID09PSB2b2lkIDApIHtcbiAgICAgIGxvID0gMDtcbiAgICB9XG4gICAgaWYgKGhpID09PSB2b2lkIDApIHtcbiAgICAgIGhpID0gMDtcbiAgICB9XG4gICAgciA9IE1hdGgucmFuZG9tKCk7XG4gICAgciA9IChoaSAtIGxvKSAqIHIgKyBsbztcbiAgICByZXR1cm4gcjtcbiAgfTtcblxuICBiMk1hdGguTmV4dFBvd2VyT2ZUd28gPSBmdW5jdGlvbih4KSB7XG4gICAgaWYgKHggPT09IHZvaWQgMCkge1xuICAgICAgeCA9IDA7XG4gICAgfVxuICAgIHggfD0gKHggPj4gMSkgJiAweDdGRkZGRkZGO1xuICAgIHggfD0gKHggPj4gMikgJiAweDNGRkZGRkZGO1xuICAgIHggfD0gKHggPj4gNCkgJiAweDBGRkZGRkZGO1xuICAgIHggfD0gKHggPj4gOCkgJiAweDAwRkZGRkZGO1xuICAgIHggfD0gKHggPj4gMTYpICYgMHgwMDAwRkZGRjtcbiAgICByZXR1cm4geCArIDE7XG4gIH07XG5cbiAgYjJNYXRoLklzUG93ZXJPZlR3byA9IGZ1bmN0aW9uKHgpIHtcbiAgICB2YXIgcmVzdWx0O1xuICAgIGlmICh4ID09PSB2b2lkIDApIHtcbiAgICAgIHggPSAwO1xuICAgIH1cbiAgICByZXN1bHQgPSB4ID4gMCAmJiAoeCAmICh4IC0gMSkpID09PSAwO1xuICAgIHJldHVybiByZXN1bHQ7XG4gIH07XG5cbiAgYjJNYXRoLmIyVmVjMl96ZXJvID0gbmV3IGIyVmVjMigwLjAsIDAuMCk7XG5cbiAgYjJNYXRoLmIyTWF0MjJfaWRlbnRpdHkgPSBiMk1hdDIyLkZyb21WVihuZXcgYjJWZWMyKDEuMCwgMC4wKSwgbmV3IGIyVmVjMigwLjAsIDEuMCkpO1xuXG4gIGIyTWF0aC5iMlRyYW5zZm9ybV9pZGVudGl0eSA9IG5ldyBiMlRyYW5zZm9ybShiMk1hdGguYjJWZWMyX3plcm8sIGIyTWF0aC5iMk1hdDIyX2lkZW50aXR5KTtcblxuICByZXR1cm4gYjJNYXRoO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMk1hdGguanMubWFwXG4iLCJ2YXIgQm94MkQsIGIyTWF0MjIsIGIyVmVjMjtcblxuQm94MkQgPSByZXF1aXJlKCcuLi8uLi9pbmRleCcpO1xuXG5iMk1hdDIyID0gQm94MkQuQ29tbW9uLk1hdGguYjJNYXQyMjtcblxuYjJWZWMyID0gQm94MkQuQ29tbW9uLk1hdGguYjJWZWMyO1xuXG5Cb3gyRC5Db21tb24uTWF0aC5iMlRyYW5zZm9ybSA9IChmdW5jdGlvbigpIHtcbiAgYjJUcmFuc2Zvcm0ucHJvdG90eXBlLnBvc2l0aW9uID0gbnVsbDtcblxuICBiMlRyYW5zZm9ybS5wcm90b3R5cGUuUiA9IG51bGw7XG5cbiAgZnVuY3Rpb24gYjJUcmFuc2Zvcm0ocG9zLCByKSB7XG4gICAgdGhpcy5wb3NpdGlvbiA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLlIgPSBuZXcgYjJNYXQyMigpO1xuICAgIGlmIChwb3MgPT09IHZvaWQgMCkge1xuICAgICAgcG9zID0gbnVsbDtcbiAgICB9XG4gICAgaWYgKHIgPT09IHZvaWQgMCkge1xuICAgICAgciA9IG51bGw7XG4gICAgfVxuICAgIGlmIChwb3MpIHtcbiAgICAgIHRoaXMucG9zaXRpb24uU2V0Vihwb3MpO1xuICAgICAgdGhpcy5SLlNldE0ocik7XG4gICAgfVxuICAgIHJldHVybjtcbiAgfVxuXG4gIGIyVHJhbnNmb3JtLnByb3RvdHlwZS5Jbml0aWFsaXplID0gZnVuY3Rpb24ocG9zLCByKSB7XG4gICAgdGhpcy5wb3NpdGlvbi5TZXRWKHBvcyk7XG4gICAgdGhpcy5SLlNldE0ocik7XG4gIH07XG5cbiAgYjJUcmFuc2Zvcm0ucHJvdG90eXBlLlNldElkZW50aXR5ID0gZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5wb3NpdGlvbi5TZXRaZXJvKCk7XG4gICAgdGhpcy5SLlNldElkZW50aXR5KCk7XG4gIH07XG5cbiAgYjJUcmFuc2Zvcm0ucHJvdG90eXBlLlNldCA9IGZ1bmN0aW9uKHgpIHtcbiAgICB0aGlzLnBvc2l0aW9uLlNldFYoeC5wb3NpdGlvbik7XG4gICAgdGhpcy5SLlNldE0oeC5SKTtcbiAgfTtcblxuICBiMlRyYW5zZm9ybS5wcm90b3R5cGUuU2V0QW5nbGUgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gTWF0aC5hdGFuMih0aGlzLlIuY29sMS55LCB0aGlzLlIuY29sMS54KTtcbiAgfTtcblxuICByZXR1cm4gYjJUcmFuc2Zvcm07XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIyVHJhbnNmb3JtLmpzLm1hcFxuIiwidmFyIEJveDJEO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uLy4uL2luZGV4Jyk7XG5cbkJveDJELkNvbW1vbi5NYXRoLmIyVmVjMiA9IChmdW5jdGlvbigpIHtcbiAgYjJWZWMyLnByb3RvdHlwZS54ID0gMDtcblxuICBiMlZlYzIucHJvdG90eXBlLnkgPSAwO1xuXG4gIGZ1bmN0aW9uIGIyVmVjMih4LCB5KSB7XG4gICAgdGhpcy54ID0geCAhPSBudWxsID8geCA6IDA7XG4gICAgdGhpcy55ID0geSAhPSBudWxsID8geSA6IDA7XG4gIH1cblxuICBiMlZlYzIucHJvdG90eXBlLlNldFplcm8gPSBmdW5jdGlvbigpIHtcbiAgICB0aGlzLnggPSAwLjA7XG4gICAgdGhpcy55ID0gMC4wO1xuICB9O1xuXG4gIGIyVmVjMi5wcm90b3R5cGUuU2V0ID0gZnVuY3Rpb24oeCwgeSkge1xuICAgIGlmICh4ID09IG51bGwpIHtcbiAgICAgIHggPSAwO1xuICAgIH1cbiAgICBpZiAoeSA9PSBudWxsKSB7XG4gICAgICB5ID0gMDtcbiAgICB9XG4gICAgdGhpcy54ID0geDtcbiAgICB0aGlzLnkgPSB5O1xuICB9O1xuXG4gIGIyVmVjMi5wcm90b3R5cGUuU2V0ViA9IGZ1bmN0aW9uKHYpIHtcbiAgICBpZiAodiA9PT0gdm9pZCAwKSB7XG4gICAgICBjb25zb2xlLmVycm9yKFwidW5kZWZpbmVkICd2JyBpbiBiMlZlYzIuU2V0VlwiKTtcbiAgICB9XG4gICAgdGhpcy54ID0gdi54O1xuICAgIHRoaXMueSA9IHYueTtcbiAgfTtcblxuICBiMlZlYzIuTWFrZSA9IGZ1bmN0aW9uKHgsIHkpIHtcbiAgICBpZiAoeCA9PSBudWxsKSB7XG4gICAgICB4ID0gMDtcbiAgICB9XG4gICAgaWYgKHkgPT0gbnVsbCkge1xuICAgICAgeSA9IDA7XG4gICAgfVxuICAgIHJldHVybiBuZXcgYjJWZWMyKHgsIHkpO1xuICB9O1xuXG4gIGlmIChiMlZlYzIuR2V0ID09PSB2b2lkIDApIHtcbiAgICBiMlZlYzIuR2V0ID0gYjJWZWMyLk1ha2U7XG4gICAgYjJWZWMyLl9mcmVlQ2FjaGUgPSBbXTtcbiAgICBiMlZlYzIuRnJlZSA9IGZ1bmN0aW9uKCkge307XG4gIH1cblxuICBiMlZlYzIucHJvdG90eXBlLkNvcHkgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gbmV3IGIyVmVjMih0aGlzLngsIHRoaXMueSk7XG4gIH07XG5cbiAgYjJWZWMyLnByb3RvdHlwZS5BZGQgPSBmdW5jdGlvbih2KSB7XG4gICAgaWYgKHYgPT09IHZvaWQgMCkge1xuICAgICAgY29uc29sZS5lcnJvcihcInVuZGVmaW5lZCAndicgaW4gYjJWZWMyLkFkZFwiKTtcbiAgICB9XG4gICAgdGhpcy54ICs9IHYueDtcbiAgICB0aGlzLnkgKz0gdi55O1xuICB9O1xuXG4gIGIyVmVjMi5wcm90b3R5cGUuU3VidHJhY3QgPSBmdW5jdGlvbih2KSB7XG4gICAgaWYgKHYgPT09IHZvaWQgMCkge1xuICAgICAgY29uc29sZS5lcnJvcihcInVuZGVmaW5lZCAndicgaW4gYjJWZWMyLlN1YnRyYWN0XCIpO1xuICAgIH1cbiAgICB0aGlzLnggLT0gdi54O1xuICAgIHRoaXMueSAtPSB2Lnk7XG4gIH07XG5cbiAgYjJWZWMyLnByb3RvdHlwZS5NdWx0aXBseSA9IGZ1bmN0aW9uKGEpIHtcbiAgICBpZiAoYSA9PT0gdm9pZCAwKSB7XG4gICAgICBhID0gMDtcbiAgICB9XG4gICAgdGhpcy54ICo9IGE7XG4gICAgdGhpcy55ICo9IGE7XG4gIH07XG5cbiAgYjJWZWMyLnByb3RvdHlwZS5MZW5ndGggPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gTWF0aC5zcXJ0KHRoaXMueCAqIHRoaXMueCArIHRoaXMueSAqIHRoaXMueSk7XG4gIH07XG5cbiAgYjJWZWMyLnByb3RvdHlwZS5MZW5ndGhTcXVhcmVkID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMueCAqIHRoaXMueCArIHRoaXMueSAqIHRoaXMueTtcbiAgfTtcblxuICBiMlZlYzIucHJvdG90eXBlLk5vcm1hbGl6ZSA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBpbnZMZW5ndGgsIGxlbmd0aDtcbiAgICBsZW5ndGggPSBNYXRoLnNxcnQodGhpcy54ICogdGhpcy54ICsgdGhpcy55ICogdGhpcy55KTtcbiAgICBpZiAobGVuZ3RoIDwgTnVtYmVyLk1JTl9WQUxVRSkge1xuICAgICAgcmV0dXJuIDAuMDtcbiAgICB9XG4gICAgaW52TGVuZ3RoID0gMS4wIC8gbGVuZ3RoO1xuICAgIHRoaXMueCAqPSBpbnZMZW5ndGg7XG4gICAgdGhpcy55ICo9IGludkxlbmd0aDtcbiAgICByZXR1cm4gbGVuZ3RoO1xuICB9O1xuXG4gIGIyVmVjMi5wcm90b3R5cGUuTmVnYXRpdmVTZWxmID0gZnVuY3Rpb24oKSB7XG4gICAgdGhpcy54ID0gLXRoaXMueDtcbiAgICB0aGlzLnkgPSAtdGhpcy55O1xuICB9O1xuXG4gIHJldHVybiBiMlZlYzI7XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIyVmVjMi5qcy5tYXBcbiIsInZhciBCb3gyRCwgYjJGaXh0dXJlLCBiMk1hdDIyLCBiMk1hdGgsIGIyVHJhbnNmb3JtLCBiMlZlYzI7XG5cbkJveDJEID0gcmVxdWlyZSgnLi4vaW5kZXgnKTtcblxuYjJNYXQyMiA9IEJveDJELkNvbW1vbi5NYXRoLmIyTWF0MjI7XG5cbmIyTWF0aCA9IEJveDJELkNvbW1vbi5NYXRoLmIyTWF0aDtcblxuYjJUcmFuc2Zvcm0gPSBCb3gyRC5Db21tb24uTWF0aC5iMlRyYW5zZm9ybTtcblxuYjJWZWMyID0gQm94MkQuQ29tbW9uLk1hdGguYjJWZWMyO1xuXG5iMkZpeHR1cmUgPSBCb3gyRC5EeW5hbWljcy5iMkZpeHR1cmU7XG5cbkJveDJELkR5bmFtaWNzLmIyQm9keSA9IChmdW5jdGlvbigpIHtcbiAgYjJCb2R5LmIyX3N0YXRpY0JvZHkgPSAwO1xuXG4gIGIyQm9keS5iMl9raW5lbWF0aWNCb2R5ID0gMTtcblxuICBiMkJvZHkuYjJfZHluYW1pY0JvZHkgPSAxO1xuXG4gIGIyQm9keS5wcm90b3R5cGUubV93b3JsZCA9IG51bGw7XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5tX2JvZHlJRCA9IG51bGw7XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5tX3VzZXJEYXRhID0gbnVsbDtcblxuICBiMkJvZHkucHJvdG90eXBlLm1feGYgPSBudWxsO1xuXG4gIGIyQm9keS5wcm90b3R5cGUubV9maXh0dXJlcyA9IG51bGw7XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5tX2FjdGl2ZSA9IGZhbHNlO1xuXG4gIGZ1bmN0aW9uIGIyQm9keShiZCwgd29ybGQpIHtcbiAgICB2YXIgdXNlckRhdGE7XG4gICAgdXNlckRhdGEgPSBiZC51c2VyRGF0YTtcbiAgICBiZC51c2VyRGF0YSA9IG51bGw7XG4gICAgdGhpcy5tX3dvcmxkID0gd29ybGQ7XG4gICAgdGhpcy5tX3hmID0gbmV3IGIyVHJhbnNmb3JtKGJkLnBvc2l0aW9uLCBiMk1hdDIyLkZyb21BbmdsZShiZC5hbmdsZSkpO1xuICAgIHRoaXMubV9maXh0dXJlcyA9IFtdO1xuICAgIHRoaXMubV9hY3RpdmUgPSBiZC5hY3RpdmU7XG4gICAgaWYgKGJkLnR5cGUgPT09IGIyQm9keS5iMl9zdGF0aWNCb2R5KSB7XG4gICAgICBiZC5kZW5zaXR5ID0gMDtcbiAgICB9XG4gICAgdGhpcy5tX2JvZHlJRCA9IHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ2NyZWF0ZUJvZHknLCB3b3JsZC5tX3dvcmxkSUQsIGJkKTtcbiAgICB0aGlzLm1fdXNlckRhdGEgPSB1c2VyRGF0YTtcbiAgICBiZC51c2VyRGF0YSA9IHVzZXJEYXRhO1xuICB9XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5DcmVhdGVGaXh0dXJlID0gZnVuY3Rpb24oZGVmKSB7XG4gICAgdmFyIGZpeHR1cmUsIGZpeHR1cmVJRCwgdXNlckRhdGE7XG4gICAgdXNlckRhdGEgPSBkZWYudXNlckRhdGE7XG4gICAgZGVmLnVzZXJEYXRhID0gbnVsbDtcbiAgICBmaXh0dXJlSUQgPSB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdjcmVhdGVGaXh0dXJlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgZGVmKTtcbiAgICBkZWYudXNlckRhdGEgPSB1c2VyRGF0YTtcbiAgICBmaXh0dXJlID0gbmV3IGIyRml4dHVyZSh0aGlzLCB1c2VyRGF0YSwgZml4dHVyZUlELCBkZWYpO1xuICAgIHRoaXMubV93b3JsZC5tX2ZpeHR1cmVzTGlzdFtmaXh0dXJlSURdID0gZml4dHVyZTtcbiAgICB0aGlzLm1fZml4dHVyZXMucHVzaChmaXh0dXJlKTtcbiAgICByZXR1cm4gZml4dHVyZTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldEZpeHR1cmVMaXN0ID0gZnVuY3Rpb24oKSB7XG4gICAgaWYgKHRoaXMubV9maXh0dXJlcy5sZW5ndGggPT09IDApIHtcbiAgICAgIHJldHVybiBudWxsO1xuICAgIH1cbiAgICByZXR1cm4gdGhpcy5tX2ZpeHR1cmVzWzBdO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuRGVzdHJveUZpeHR1cmUgPSBmdW5jdGlvbihmaXh0dXJlKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbCgnZGVsZXRlRml4dHVyZScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIGZpeHR1cmUubV9maXh0dXJlSUQpO1xuICAgIGRlbGV0ZSB0aGlzLm1fd29ybGQubV9maXh0dXJlc0xpc3RbZml4dHVyZS5tX2ZpeHR1cmVJRF07XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5TZXRQb3NpdGlvbkFuZEFuZ2xlID0gZnVuY3Rpb24ocG9zaXRpb24sIGFuZ2xlKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbCgnc2V0Qm9keVRyYW5zZm9ybScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQsIHBvc2l0aW9uLngsIHBvc2l0aW9uLnksIGFuZ2xlKTtcbiAgICB0aGlzLm1feGYuUi5TZXQoYW5nbGUpO1xuICAgIHRoaXMubV94Zi5wb3NpdGlvbi5TZXRWKHBvc2l0aW9uKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldFBvc2l0aW9uID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMubV94Zi5wb3NpdGlvbjtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLlNldFBvc2l0aW9uID0gZnVuY3Rpb24ocG9zaXRpb24pIHtcbiAgICB0aGlzLlNldFBvc2l0aW9uQW5kQW5nbGUocG9zaXRpb24sIHRoaXMuR2V0QW5nbGUoKSk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5HZXRMaW5lYXJWZWxvY2l0eSA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciB2O1xuICAgIHYgPSB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdnZXRMaW5lYXJWZWxvY2l0eScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQpO1xuICAgIHJldHVybiBuZXcgYjJWZWMyKHZbMF0sIHZbMV0pO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuU2V0TGluZWFyVmVsb2NpdHkgPSBmdW5jdGlvbih2ZWwpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdzZXRMaW5lYXJWZWxvY2l0eScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQsIHZlbC54LCB2ZWwueSk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5TZXRMaW5lYXJEYW1waW5nID0gZnVuY3Rpb24oZGFtcCkge1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ3NldExpbmVhckRhbXBpbmcnLCB0aGlzLm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fYm9keUlELCBkYW1wKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldFdvcmxkQ2VudGVyID0gZnVuY3Rpb24oKSB7XG4gICAgdmFyIHA7XG4gICAgcCA9IHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ2dldFdvcmxkQ2VudGVyJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCk7XG4gICAgcmV0dXJuIG5ldyBiMlZlYzIocFswXSwgcFsxXSk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5HZXRMb2NhbENlbnRlciA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBwO1xuICAgIHAgPSB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdnZXRMb2NhbENlbnRlcicsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQpO1xuICAgIHJldHVybiBuZXcgYjJWZWMyKHBbMF0sIHBbMV0pO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuR2V0TG9jYWxQb2ludCA9IGZ1bmN0aW9uKHdvcmxkUG9pbnQpIHtcbiAgICByZXR1cm4gYjJNYXRoLk11bFhUKHRoaXMubV94Ziwgd29ybGRQb2ludCk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5HZXRVc2VyRGF0YSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm1fdXNlckRhdGE7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5TZXRVc2VyRGF0YSA9IGZ1bmN0aW9uKGRhdGEpIHtcbiAgICB0aGlzLm1fdXNlckRhdGEgPSBkYXRhO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuR2V0TWFzcyA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdnZXRNYXNzJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5Jc0F3YWtlID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ2lzQXdha2UnLCB0aGlzLm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fYm9keUlEKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLlNldEF3YWtlID0gZnVuY3Rpb24oc3RhdGUpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdzZXRBd2FrZScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQsIHN0YXRlKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldEFuZ3VsYXJWZWxvY2l0eSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdnZXRBbmd1bGFyVmVsb2NpdHknLCB0aGlzLm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fYm9keUlEKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLlNldEFuZ3VsYXJWZWxvY2l0eSA9IGZ1bmN0aW9uKGFuZ3ZlbCkge1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ3NldEFuZ3VsYXJWZWxvY2l0eScsIHRoaXMubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9ib2R5SUQsIGFuZ3ZlbCk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5TZXRGaXhlZFJvdGF0aW9uID0gZnVuY3Rpb24oZml4ZWQpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdzZXRGaXhlZFJvdGF0aW9uJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgZml4ZWQpO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuSXNBY3RpdmUgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5tX2FjdGl2ZTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLlNldEFjdGl2ZSA9IGZ1bmN0aW9uKHN0YXRlKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbCgnc2V0QWN0aXZlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgc3RhdGUpO1xuICAgIHRoaXMubV9hY3RpdmUgPSBzdGF0ZTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldEFuZ2xlID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMubV94Zi5SLkdldEFuZ2xlKCk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5TZXRBbmdsZSA9IGZ1bmN0aW9uKGFuZ2xlKSB7XG4gICAgaWYgKGFuZ2xlID09PSB2b2lkIDApIHtcbiAgICAgIGFuZ2xlID0gMDtcbiAgICB9XG4gICAgdGhpcy5TZXRQb3NpdGlvbkFuZEFuZ2xlKHRoaXMuR2V0UG9zaXRpb24oKSwgYW5nbGUpO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuU2V0VHlwZSA9IGZ1bmN0aW9uKHR5cGUpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdzZXRUeXBlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgdHlwZSk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5HZXRDb250YWN0TGlzdCA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBjb250YWN0LCBjb250YWN0cywgcmVzdWx0LCBfaSwgX2xlbjtcbiAgICBjb250YWN0cyA9IHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ2dldE9iamVjdENvbnRhY3RzJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCk7XG4gICAgcmVzdWx0ID0gW107XG4gICAgZm9yIChfaSA9IDAsIF9sZW4gPSBjb250YWN0cy5sZW5ndGg7IF9pIDwgX2xlbjsgX2krKykge1xuICAgICAgY29udGFjdCA9IGNvbnRhY3RzW19pXTtcbiAgICAgIHJlc3VsdC5wdXNoKHRoaXMubV93b3JsZC5tX2JvZHlMaXN0W2NvbnRhY3RdKTtcbiAgICB9XG4gICAgcmV0dXJuIHJlc3VsdDtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkdldFdvcmxkID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMubV93b3JsZDtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkFwcGx5SW1wdWxzZSA9IGZ1bmN0aW9uKGltcHVsc2UsIHBvaW50LCB3YWtlKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbCgnYXBwbHlJbXB1bHNlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgaW1wdWxzZS54LCBpbXB1bHNlLnksIHBvaW50LngsIHBvaW50LnksIHdha2UpO1xuICB9O1xuXG4gIGIyQm9keS5wcm90b3R5cGUuQXBwbHlGb3JjZVRvQ2VudGVyID0gZnVuY3Rpb24oZm9yY2UsIHdha2UpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdhcHBseUZvcmNlVG9DZW50ZXInLCB0aGlzLm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fYm9keUlELCBmb3JjZS54LCBmb3JjZS55LCB3YWtlKTtcbiAgfTtcblxuICBiMkJvZHkucHJvdG90eXBlLkFwcGx5Rm9yY2UgPSBmdW5jdGlvbihmb3JjZSwgcG9pbnQsIHdha2UpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKCdhcHBseUZvcmNlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgZm9yY2UueCwgZm9yY2UueSwgcG9pbnQueCwgcG9pbnQueSwgd2FrZSk7XG4gIH07XG5cbiAgYjJCb2R5LnByb3RvdHlwZS5BcHBseVRvcnF1ZSA9IGZ1bmN0aW9uKHRvcnF1ZSwgd2FrZSkge1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoJ2FwcGx5VG9ycXVlJywgdGhpcy5tX3dvcmxkLm1fd29ybGRJRCwgdGhpcy5tX2JvZHlJRCwgdG9ycXVlLCB3YWtlKTtcbiAgfTtcblxuICByZXR1cm4gYjJCb2R5O1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMkJvZHkuanMubWFwXG4iLCJ2YXIgQm94MkQsIGIyQm9keSwgYjJWZWMyO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uL2luZGV4Jyk7XG5cbmIyVmVjMiA9IEJveDJELkNvbW1vbi5NYXRoLmIyVmVjMjtcblxuYjJCb2R5ID0gQm94MkQuRHluYW1pY3MuYjJCb2R5O1xuXG5Cb3gyRC5EeW5hbWljcy5iMkJvZHlEZWYgPSAoZnVuY3Rpb24oKSB7XG4gIGIyQm9keURlZi5wcm90b3R5cGUucG9zaXRpb24gPSBudWxsO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUubGluZWFyVmVsb2NpdHkgPSBudWxsO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUudXNlckRhdGEgPSBudWxsO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUuYW5nbGUgPSAwLjA7XG5cbiAgYjJCb2R5RGVmLnByb3RvdHlwZS5saW5lYXJWZWxvY2l0eSA9IG51bGw7XG5cbiAgYjJCb2R5RGVmLnByb3RvdHlwZS5hbmd1bGFyVmVsb2NpdHkgPSAwLjA7XG5cbiAgYjJCb2R5RGVmLnByb3RvdHlwZS5hbmd1bGFyRGFtcGluZyA9IDAuMDtcblxuICBiMkJvZHlEZWYucHJvdG90eXBlLmFsbG93U2xlZXAgPSB0cnVlO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUuYXdha2UgPSB0cnVlO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUuZml4ZWRSb3RhdGlvbiA9IGZhbHNlO1xuXG4gIGIyQm9keURlZi5wcm90b3R5cGUuYnVsbGV0ID0gZmFsc2U7XG5cbiAgYjJCb2R5RGVmLnByb3RvdHlwZS50eXBlID0gMDtcblxuICBiMkJvZHlEZWYucHJvdG90eXBlLmFjdGl2ZSA9IHRydWU7XG5cbiAgYjJCb2R5RGVmLnByb3RvdHlwZS5pbmVydGlhU2NhbGUgPSAxLjA7XG5cbiAgZnVuY3Rpb24gYjJCb2R5RGVmKCkge1xuICAgIHRoaXMucG9zaXRpb24gPSBuZXcgYjJWZWMyKDAsIDApO1xuICAgIHRoaXMubGluZWFyVmVsb2NpdHkgPSBuZXcgYjJWZWMyKCk7XG4gICAgdGhpcy51c2VyRGF0YSA9IG51bGw7XG4gICAgdGhpcy5hbmdsZSA9IDAuMDtcbiAgICB0aGlzLmxpbmVhclZlbG9jaXR5LlNldCgwLCAwKTtcbiAgICB0aGlzLmFuZ3VsYXJWZWxvY2l0eSA9IDAuMDtcbiAgICB0aGlzLmxpbmVhckRhbXBpbmcgPSAwLjA7XG4gICAgdGhpcy5hbmd1bGFyRGFtcGluZyA9IDAuMDtcbiAgICB0aGlzLmFsbG93U2xlZXAgPSB0cnVlO1xuICAgIHRoaXMuYXdha2UgPSB0cnVlO1xuICAgIHRoaXMuZml4ZWRSb3RhdGlvbiA9IGZhbHNlO1xuICAgIHRoaXMuYnVsbGV0ID0gZmFsc2U7XG4gICAgdGhpcy50eXBlID0gYjJCb2R5LmIyX3N0YXRpY0JvZHk7XG4gICAgdGhpcy5hY3RpdmUgPSB0cnVlO1xuICAgIHRoaXMuaW5lcnRpYVNjYWxlID0gMS4wO1xuICB9XG5cbiAgcmV0dXJuIGIyQm9keURlZjtcblxufSkoKTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YjJCb2R5RGVmLmpzLm1hcFxuIiwidmFyIEJveDJEO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uL2luZGV4Jyk7XG5cbkJveDJELkR5bmFtaWNzLmIyRGVidWdEcmF3ID0gKGZ1bmN0aW9uKCkge1xuICBmdW5jdGlvbiBiMkRlYnVnRHJhdygpIHt9XG5cbiAgYjJEZWJ1Z0RyYXcuZV9hYWJiQml0ID0gMHgwMDA0O1xuXG4gIGIyRGVidWdEcmF3LmVfY2VudGVyT2ZNYXNzQml0ID0gMHgwMDEwO1xuXG4gIGIyRGVidWdEcmF3LmVfY29udHJvbGxlckJpdCA9IDB4MDAyMDtcblxuICBiMkRlYnVnRHJhdy5lX2pvaW50Qml0ID0gMHgwMDAyO1xuXG4gIGIyRGVidWdEcmF3LmVfcGFpckJpdCA9IDB4MDAwODtcblxuICBiMkRlYnVnRHJhdy5lX3NoYXBlQml0ID0gMHgwMDA7XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkFwcGVuZEZsYWdzID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkRlYnVnRHJhdy5wcm90b3R5cGUuQ2xlYXJGbGFncyA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkRyYXdDaXJjbGUgPSBmdW5jdGlvbigpIHt9O1xuXG4gIGIyRGVidWdEcmF3LnByb3RvdHlwZS5EcmF3UG9seWdvbiA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkRyYXdTZWdtZW50ID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkRlYnVnRHJhdy5wcm90b3R5cGUuRHJhd1NvbGlkQ2lyY2xlID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkRlYnVnRHJhdy5wcm90b3R5cGUuRHJhd1NvbGlkUG9seWdvbiA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkRyYXdUcmFuc2Zvcm0gPSBmdW5jdGlvbigpIHt9O1xuXG4gIGIyRGVidWdEcmF3LnByb3RvdHlwZS5HZXRBbHBoYSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkdldERyYXdTY2FsZSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkdldEZpbGxBbHBoYSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkdldEZsYWdzID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkRlYnVnRHJhdy5wcm90b3R5cGUuR2V0TGluZVRoaWNrbmVzcyA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkdldFNwcml0ZSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLkdldFhGb3JtU2NhbGUgPSBmdW5jdGlvbigpIHt9O1xuXG4gIGIyRGVidWdEcmF3LnByb3RvdHlwZS5TZXRBbHBoYSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLlNldERyYXdTY2FsZSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLlNldEZpbGxBbHBoYSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLlNldEZsYWdzID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkRlYnVnRHJhdy5wcm90b3R5cGUuU2V0TGluZVRoaWNrbmVzcyA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLlNldFNwcml0ZSA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJEZWJ1Z0RyYXcucHJvdG90eXBlLlNldFhGb3JtU2NhbGUgPSBmdW5jdGlvbigpIHt9O1xuXG4gIHJldHVybiBiMkRlYnVnRHJhdztcblxufSkoKTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YjJEZWJ1Z0RyYXcuanMubWFwXG4iLCJ2YXIgQm94MkQsIGIyVmVjMjtcblxuQm94MkQgPSByZXF1aXJlKCcuLi9pbmRleCcpO1xuXG5iMlZlYzIgPSBCb3gyRC5Db21tb24uTWF0aC5iMlZlYzI7XG5cbkJveDJELkR5bmFtaWNzLmIyRml4dHVyZSA9IChmdW5jdGlvbigpIHtcbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5tX2JvZHkgPSBudWxsO1xuXG4gIGIyRml4dHVyZS5wcm90b3R5cGUubV91c2VyRGF0YSA9IG51bGw7XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5tX2ZpeHR1cmVJRCA9IG51bGw7XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5tX3NoYXBlID0gbnVsbDtcblxuICBiMkZpeHR1cmUucHJvdG90eXBlLm1faXNTZW5zb3IgPSBmYWxzZTtcblxuICBiMkZpeHR1cmUucHJvdG90eXBlLm1fZGVuc2l0eSA9IG51bGw7XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5tX2ZyaWN0aW9uID0gbnVsbDtcblxuICBiMkZpeHR1cmUucHJvdG90eXBlLm1fcmVzdGl0dXRpb24gPSBudWxsO1xuXG4gIGZ1bmN0aW9uIGIyRml4dHVyZShib2R5LCB1c2VyRGF0YSwgZml4dHVyZUlELCBkZWYpIHtcbiAgICB0aGlzLm1fYm9keSA9IGJvZHk7XG4gICAgdGhpcy5tX3VzZXJEYXRhID0gdXNlckRhdGE7XG4gICAgdGhpcy5tX2ZpeHR1cmVJRCA9IGZpeHR1cmVJRDtcbiAgICB0aGlzLm1fc2hhcGUgPSB7fTtcbiAgICB0aGlzLm1fc2hhcGUubV9jZW50cm9pZCA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLm1faXNTZW5zb3IgPSBmYWxzZTtcbiAgICB0aGlzLm1fZGVuc2l0eSA9IGRlZi5kZW5zaXR5O1xuICAgIHRoaXMubV9mcmljdGlvbiA9IGRlZi5mcmljdGlvbjtcbiAgICB0aGlzLm1fcmVzdGl0dXRpb24gPSBkZWYucmVzdGl0dXRpb247XG4gICAgdGhpcy5tX2lzU2Vuc29yID0gZGVmLmlzU2Vuc29yO1xuICAgIHJldHVybjtcbiAgfVxuXG4gIGIyRml4dHVyZS5wcm90b3R5cGUuR2V0Qm9keSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm1fYm9keTtcbiAgfTtcblxuICBiMkZpeHR1cmUucHJvdG90eXBlLkdldFNoYXBlID0gZnVuY3Rpb24oKSB7XG4gICAgY29uc29sZS5sb2coXCJmaXh0dXJlLkdldFNoYXBlIG5vdCB5ZXQgc3VwcG9ydGVkIGluIENvY29vbkpTIEJveDJEIGJpbmRpbmdcIik7XG4gICAgcmV0dXJuIG51bGw7XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5HZXRVc2VyRGF0YSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm1fdXNlckRhdGE7XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5TZXRTZW5zb3IgPSBmdW5jdGlvbihpc1NlbnNvcikge1xuICAgIHRoaXMubV9pc1NlbnNvciA9IGlzU2Vuc29yO1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoXCJzZXRTZW5zb3JcIiwgdGhpcy5tX2JvZHkubV93b3JsZC5tX3dvcmxkSUQsIHRoaXMubV9maXh0dXJlSUQsIHRoaXMubV9pc1NlbnNvcik7XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5Jc1NlbnNvciA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm1faXNTZW5zb3I7XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5TZXREZW5zaXR5ID0gZnVuY3Rpb24oZGVuc2l0eSkge1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoXCJzZXREZW5zaXR5XCIsIHRoaXMubV9ib2R5Lm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fZml4dHVyZUlELCBkZW5zaXR5KTtcbiAgICB0aGlzLm1fZGVuc2l0eSA9IGRlbnNpdHk7XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5TZXRGcmljdGlvbiA9IGZ1bmN0aW9uKGZyaWN0aW9uKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbChcInNldEZyaWN0aW9uXCIsIHRoaXMubV9ib2R5Lm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fZml4dHVyZUlELCBmcmljdGlvbik7XG4gICAgdGhpcy5tX2ZyaWN0aW9uID0gZnJpY3Rpb247XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5TZXRSZXN0aXR1dGlvbiA9IGZ1bmN0aW9uKHJlc3RpdHV0aW9uKSB7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbChcInNldFJlc3RpdHV0aW9uXCIsIHRoaXMubV9ib2R5Lm1fd29ybGQubV93b3JsZElELCB0aGlzLm1fZml4dHVyZUlELCByZXN0aXR1dGlvbik7XG4gICAgdGhpcy5tX3Jlc3RpdHV0aW9uID0gcmVzdGl0dXRpb247XG4gIH07XG5cbiAgYjJGaXh0dXJlLnByb3RvdHlwZS5HZXREZW5zaXR5ID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMubV9kZW5zaXR5O1xuICB9O1xuXG4gIGIyRml4dHVyZS5wcm90b3R5cGUuR2V0RnJpY3Rpb24gPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5tX2ZyaWN0aW9uO1xuICB9O1xuXG4gIGIyRml4dHVyZS5wcm90b3R5cGUuR2V0UmVzdGl0dXRpb24gPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5tX3Jlc3RpdHV0aW9uO1xuICB9O1xuXG4gIHJldHVybiBiMkZpeHR1cmU7XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIyRml4dHVyZS5qcy5tYXBcbiIsInZhciBCb3gyRDtcblxuQm94MkQgPSByZXF1aXJlKCcuLi9pbmRleCcpO1xuXG5Cb3gyRC5EeW5hbWljcy5iMkZpeHR1cmVEZWYgPSAoZnVuY3Rpb24oKSB7XG4gIGIyRml4dHVyZURlZi5wcm90b3R5cGUuc2hhcGUgPSBudWxsO1xuXG4gIGIyRml4dHVyZURlZi5wcm90b3R5cGUudXNlckRhdGEgPSBudWxsO1xuXG4gIGIyRml4dHVyZURlZi5wcm90b3R5cGUuZnJpY3Rpb24gPSAwLjI7XG5cbiAgYjJGaXh0dXJlRGVmLnByb3RvdHlwZS5yZXN0aXR1dGlvbiA9IDAuMDtcblxuICBiMkZpeHR1cmVEZWYucHJvdG90eXBlLmRlbnNpdHkgPSAwLjA7XG5cbiAgYjJGaXh0dXJlRGVmLnByb3RvdHlwZS5pc1NlbnNvciA9IGZhbHNlO1xuXG4gIGIyRml4dHVyZURlZi5wcm90b3R5cGUuZmlsdGVyID0gbnVsbDtcblxuICBmdW5jdGlvbiBiMkZpeHR1cmVEZWYoKSB7XG4gICAgdGhpcy5zaGFwZSA9IG51bGw7XG4gICAgdGhpcy51c2VyRGF0YSA9IG51bGw7XG4gICAgdGhpcy5mcmljdGlvbiA9IDAuMjtcbiAgICB0aGlzLnJlc3RpdHV0aW9uID0gMC4wO1xuICAgIHRoaXMuZGVuc2l0eSA9IDAuMDtcbiAgICB0aGlzLmlzU2Vuc29yID0gZmFsc2U7XG4gICAgdGhpcy5maWx0ZXIgPSB7XG4gICAgICBjYXRlZ29yeUJpdHM6IDEsXG4gICAgICBtYXNrQml0czogMHhGRkZGLFxuICAgICAgZ3JvdXBJbmRleDogMFxuICAgIH07XG4gIH1cblxuICByZXR1cm4gYjJGaXh0dXJlRGVmO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMkZpeHR1cmVEZWYuanMubWFwXG4iLCJ2YXIgQm94MkQsIGIyQm9keSwgYjJDb250YWN0LCBiMkpvaW50LCBiMlZlYzI7XG5cbkJveDJEID0gcmVxdWlyZSgnLi4vaW5kZXgnKTtcblxuYjJWZWMyID0gQm94MkQuQ29tbW9uLk1hdGguYjJWZWMyO1xuXG5iMkJvZHkgPSBCb3gyRC5EeW5hbWljcy5iMkJvZHk7XG5cbmIySm9pbnQgPSBCb3gyRC5EeW5hbWljcy5Kb2ludHMuYjJKb2ludDtcblxuYjJDb250YWN0ID0gQm94MkQuRHluYW1pY3MuYjJDb250YWN0O1xuXG5Cb3gyRC5EeW5hbWljcy5iMldvcmxkID0gKGZ1bmN0aW9uKCkge1xuICBiMldvcmxkLnByb3RvdHlwZS5tX2JvZHlMaXN0ID0gbnVsbDtcblxuICBiMldvcmxkLnByb3RvdHlwZS5tX2pvaW50TGlzdCA9IG51bGw7XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUubV9maXh0dXJlc0xpc3QgPSBudWxsO1xuXG4gIGIyV29ybGQucHJvdG90eXBlLm1fY29udGFjdExpc3RlbmVyID0gbnVsbDtcblxuICBiMldvcmxkLnByb3RvdHlwZS5tX2pvaW50c0xpc3QgPSBudWxsO1xuXG4gIGIyV29ybGQucHJvdG90eXBlLm1fd29ybGRJRCA9IDA7XG5cbiAgZnVuY3Rpb24gYjJXb3JsZChncmF2aXR5LCBkb1NsZWVwKSB7XG4gICAgdGhpcy5tX2JvZHlMaXN0ID0gW107XG4gICAgdGhpcy5tX2pvaW50TGlzdCA9IFtdO1xuICAgIHRoaXMubV9maXh0dXJlc0xpc3QgPSBbXTtcbiAgICB0aGlzLm1fY29udGFjdExpc3RlbmVyID0gbnVsbDtcbiAgICB0aGlzLm1fam9pbnRzTGlzdCA9IFtdO1xuICAgIHRoaXMubV93b3JsZElEID0gd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbChcImNyZWF0ZVdvcmxkXCIsIGdyYXZpdHkueCwgZ3Jhdml0eS55LCBkb1NsZWVwKTtcbiAgICByZXR1cm47XG4gIH1cblxuICBiMldvcmxkLnByb3RvdHlwZS5TZXRDb250YWN0TGlzdGVuZXIgPSBmdW5jdGlvbihsaXN0ZW5lcikge1xuICAgIHRoaXMubV9jb250YWN0TGlzdGVuZXIgPSBsaXN0ZW5lcjtcbiAgfTtcblxuICBiMldvcmxkLnByb3RvdHlwZS5TZXRDb250YWN0RmlsdGVyID0gZnVuY3Rpb24oZmlsdGVyKSB7XG4gICAgdmFyIGNhbGxiYWNrRnVuYztcbiAgICBjYWxsYmFja0Z1bmMgPSBmdW5jdGlvbihhLCBiKSB7XG4gICAgICB2YXIgZmEsIGZiO1xuICAgICAgZmEgPSB3b3JsZC5tX2ZpeHR1cmVzTGlzdFthXTtcbiAgICAgIGZiID0gd29ybGQubV9maXh0dXJlc0xpc3RbYl07XG4gICAgICByZXR1cm4gZmlsdGVyLlNob3VsZENvbGxpZGUoZmEsIGZiKTtcbiAgICB9O1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoXCJzZXRDb250YWN0RmlsdGVyXCIsIHRoaXMubV93b3JsZElELCBjYWxsYmFja0Z1bmMpO1xuICB9O1xuXG4gIGIyV29ybGQucHJvdG90eXBlLkNyZWF0ZUJvZHkgPSBmdW5jdGlvbihkZWYpIHtcbiAgICB2YXIgYjtcbiAgICBiID0gbmV3IGIyQm9keShkZWYsIHRoaXMpO1xuICAgIHRoaXMubV9ib2R5TGlzdFtiLm1fYm9keUlEXSA9IGI7XG4gICAgcmV0dXJuIGI7XG4gIH07XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUuRGVzdHJveUJvZHkgPSBmdW5jdGlvbihiKSB7XG4gICAgdmFyIGk7XG4gICAgd2luZG93LmV4dC5JRFRLX1NSVl9CT1gyRC5tYWtlQ2FsbChcImRlbGV0ZUJvZHlcIiwgdGhpcy5tX3dvcmxkSUQsIGIubV9ib2R5SUQpO1xuICAgIGRlbGV0ZSB0aGlzLm1fYm9keUxpc3RbYi5tX2JvZHlJRF07XG4gICAgcmV0dXJuO1xuICAgIGkgPSAwO1xuICAgIHdoaWxlIChpIDwgYi5tX2ZpeHR1cmVzLmxlbmd0aCkge1xuICAgICAgZGVsZXRlIHRoaXMubV9maXh0dXJlc0xpc3RbYi5tX2ZpeHR1cmVzW2ldLm1fZml4dHVyZUlEXTtcbiAgICAgICsraTtcbiAgICB9XG4gIH07XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUuQ3JlYXRlSm9pbnQgPSBmdW5jdGlvbihkZWYpIHtcbiAgICB2YXIgYm9keUEsIGJvZHlCLCBqb2ludCwgam9pbnRGdW5jO1xuICAgIGlmIChkZWYuYm9keUEubV9ib2R5SUQgPT09IGRlZi5ib2R5Qi5tX2JvZHlJRCkge1xuICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICBib2R5QSA9IGRlZi5ib2R5QTtcbiAgICBib2R5QiA9IGRlZi5ib2R5QjtcbiAgICBkZWYuYm9keUEgPSBib2R5QS5tX2JvZHlJRDtcbiAgICBkZWYuYm9keUIgPSBib2R5Qi5tX2JvZHlJRDtcbiAgICBqb2ludEZ1bmMgPSBcImNyZWF0ZURpc3RhbmNlSm9pbnRcIjtcbiAgICBpZiAoZGVmLnR5cGUgPT09IGIySm9pbnQuZV9yZXZvbHV0ZUpvaW50KSB7XG4gICAgICBqb2ludEZ1bmMgPSBcImNyZWF0ZVJldm9sdXRlSm9pbnRcIjtcbiAgICB9XG4gICAgam9pbnQgPSBuZXcgYjJKb2ludChkZWYpO1xuICAgIGpvaW50Lm1fam9pbnRJRCA9IHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoam9pbnRGdW5jLCB0aGlzLm1fd29ybGRJRCwgZGVmKTtcbiAgICBkZWYuYm9keUEgPSBib2R5QTtcbiAgICBkZWYuYm9keUIgPSBib2R5QjtcbiAgICB0aGlzLm1fam9pbnRzTGlzdC5wdXNoKGpvaW50KTtcbiAgICByZXR1cm4gam9pbnQ7XG4gIH07XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUuRGVzdHJveUpvaW50ID0gZnVuY3Rpb24oam9pbnQpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKFwiZGVzdHJveUpvaW50XCIsIHRoaXMubV93b3JsZElELCBqb2ludC5tX2pvaW50SUQpO1xuICB9O1xuXG4gIGIyV29ybGQucHJvdG90eXBlLkdldEpvaW50TGlzdCA9IGZ1bmN0aW9uKCkge1xuICAgIHZhciBpO1xuICAgIGlmICh0aGlzLm1fam9pbnRzTGlzdC5sZW5ndGggPT09IDApIHtcbiAgICAgIHJldHVybiBudWxsO1xuICAgIH1cbiAgICBpID0gMDtcbiAgICB3aGlsZSAoaSA8IHRoaXMubV9qb2ludHNMaXN0Lmxlbmd0aCAtIDEpIHtcbiAgICAgIHRoaXMubV9qb2ludHNMaXN0W2ldLm5leHQgPSB0aGlzLm1fam9pbnRzTGlzdFtpICsgMV07XG4gICAgICArK2k7XG4gICAgfVxuICAgIHRoaXMubV9qb2ludHNMaXN0W3RoaXMubV9qb2ludHNMaXN0Lmxlbmd0aCAtIDFdLm5leHQgPSBudWxsO1xuICAgIHJldHVybiB0aGlzLm1fam9pbnRzTGlzdFswXTtcbiAgfTtcblxuICBiMldvcmxkLnByb3RvdHlwZS5TZXRDb250aW51b3VzUGh5c2ljcyA9IGZ1bmN0aW9uKGNvbnRpbnVvdXMpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKFwic2V0Q29udGludW91c1wiLCB0aGlzLm1fd29ybGRJRCwgY29udGludW91cyk7XG4gIH07XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUuU2V0R3Jhdml0eSA9IGZ1bmN0aW9uKGdyYXZpdHkpIHtcbiAgICB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKFwic2V0R3Jhdml0eVwiLCB0aGlzLm1fd29ybGRJRCwgZ3Jhdml0eS54LCBncmF2aXR5LnkpO1xuICB9O1xuXG4gIGIyV29ybGQucHJvdG90eXBlLlN0ZXAgPSBmdW5jdGlvbihkdCwgdmVsb2NpdHlJdGVyYXRpb25zLCBwb3NpdGlvbkl0ZXJhdGlvbnMpIHtcbiAgICB2YXIgYm9keSwgY29udGFjdHMsIGNvdW50LCBmMSwgZjIsIGZpeDEsIGZpeDIsIGksIHRvdWNoaW5nLCB0cmFuc2Zvcm1zO1xuICAgIGkgPSB2b2lkIDA7XG4gICAgdHJhbnNmb3JtcyA9IHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoXCJzdGVwXCIsIHRoaXMubV93b3JsZElELCBkdCwgdmVsb2NpdHlJdGVyYXRpb25zLCBwb3NpdGlvbkl0ZXJhdGlvbnMpO1xuICAgIGNvdW50ID0gdHJhbnNmb3Jtc1swXTtcbiAgICBpID0gMTtcbiAgICB3aGlsZSAoaSA8PSBjb3VudCAqIDQpIHtcbiAgICAgIGJvZHkgPSB0aGlzLm1fYm9keUxpc3RbdHJhbnNmb3Jtc1tpICsgMF1dO1xuICAgICAgaWYgKGJvZHkgPT09IG51bGwpIHtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9XG4gICAgICBib2R5Lm1feGYucG9zaXRpb24uU2V0KHRyYW5zZm9ybXNbaSArIDFdLCB0cmFuc2Zvcm1zW2kgKyAyXSk7XG4gICAgICBib2R5Lm1feGYuUi5TZXQodHJhbnNmb3Jtc1tpICsgM10pO1xuICAgICAgaSArPSA0O1xuICAgIH1cbiAgICBpZiAodGhpcy5tX2NvbnRhY3RMaXN0ZW5lciAhPT0gbnVsbCkge1xuICAgICAgY29udGFjdHMgPSB3aW5kb3cuZXh0LklEVEtfU1JWX0JPWDJELm1ha2VDYWxsKFwiZ2V0TGFzdENvbnRhY3RzXCIsIHRoaXMubV93b3JsZElEKTtcbiAgICAgIGNvdW50ID0gY29udGFjdHNbMF07XG4gICAgICBpID0gMTtcbiAgICAgIHdoaWxlIChpIDw9IGNvdW50ICogMykge1xuICAgICAgICBmMSA9IGNvbnRhY3RzW2kgKyAwXTtcbiAgICAgICAgZjIgPSBjb250YWN0c1tpICsgMV07XG4gICAgICAgIHRvdWNoaW5nID0gY29udGFjdHNbaSArIDJdO1xuICAgICAgICBmaXgxID0gdGhpcy5tX2ZpeHR1cmVzTGlzdFtmMV07XG4gICAgICAgIGZpeDIgPSB0aGlzLm1fZml4dHVyZXNMaXN0W2YyXTtcbiAgICAgICAgaWYgKCh0eXBlb2YgZml4MSA9PT0gXCJ1bmRlZmluZWRcIikgfHwgKHR5cGVvZiBmaXgyID09PSBcInVuZGVmaW5lZFwiKSkge1xuICAgICAgICAgIGNvbnNvbGUubG9nKFwiT25lIG9mIHRoZSBmaXh0dXJlcyBpbiBhIGNvbnRhY3QgRE9FU04nVCBFWElTVCEhXCIpO1xuICAgICAgICAgIGNvbnRpbnVlO1xuICAgICAgICB9XG4gICAgICAgIHRoaXMubV9jb250YWN0TGlzdGVuZXIuQmVnaW5Db250YWN0KG5ldyBiMkNvbnRhY3QoZml4MSwgZml4MiwgdG91Y2hpbmcpKTtcbiAgICAgICAgaSArPSAzO1xuICAgICAgfVxuICAgIH1cbiAgfTtcblxuICBiMldvcmxkLnByb3RvdHlwZS5DbGVhckZvcmNlcyA9IGZ1bmN0aW9uKCkge1xuICAgIHdpbmRvdy5leHQuSURUS19TUlZfQk9YMkQubWFrZUNhbGwoXCJjbGVhckZvcmNlc1wiLCB0aGlzLm1fd29ybGRJRCk7XG4gIH07XG5cbiAgYjJXb3JsZC5wcm90b3R5cGUuU2V0RGVidWdEcmF3ID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMldvcmxkLnByb3RvdHlwZS5EcmF3RGVidWdEYXRhID0gZnVuY3Rpb24oKSB7fTtcblxuICByZXR1cm4gYjJXb3JsZDtcblxufSkoKTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YjJXb3JsZC5qcy5tYXBcbiIsInZhciBCb3gyRDtcblxuQm94MkQgPSByZXF1aXJlKCcuLi8uLi9pbmRleCcpO1xuXG5Cb3gyRC5EeW5hbWljcy5iMkNvbnRhY3QgPSAoZnVuY3Rpb24oKSB7XG4gIGIyQ29udGFjdC5wcm90b3R5cGUubV9maXh0dXJlQSA9IG51bGw7XG5cbiAgYjJDb250YWN0LnByb3RvdHlwZS5tX2ZpeHR1cmVCID0gbnVsbDtcblxuICBiMkNvbnRhY3QucHJvdG90eXBlLm1fdG91Y2hpbmcgPSBmYWxzZTtcblxuICBmdW5jdGlvbiBiMkNvbnRhY3QoZml4dHVyZUEsIGZpeHR1cmVCLCB0b3VjaGluZykge1xuICAgIHRoaXMubV9maXh0dXJlQSA9IGZpeHR1cmVBO1xuICAgIHRoaXMubV9maXh0dXJlQiA9IGZpeHR1cmVCO1xuICAgIHRoaXMubV90b3VjaGluZyA9IHRvdWNoaW5nO1xuICAgIHJldHVybjtcbiAgfVxuXG4gIGIyQ29udGFjdC5wcm90b3R5cGUuR2V0Rml4dHVyZUEgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5tX2ZpeHR1cmVBO1xuICB9O1xuXG4gIGIyQ29udGFjdC5wcm90b3R5cGUuR2V0Rml4dHVyZUIgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5tX2ZpeHR1cmVCO1xuICB9O1xuXG4gIGIyQ29udGFjdC5wcm90b3R5cGUuSXNUb3VjaGluZyA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm1fdG91Y2hpbmc7XG4gIH07XG5cbiAgcmV0dXJuIGIyQ29udGFjdDtcblxufSkoKTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YjJDb250YWN0LmpzLm1hcFxuIiwidmFyIEJveDJEO1xuXG5Cb3gyRCA9IHJlcXVpcmUoJy4uLy4uL2luZGV4Jyk7XG5cbkJveDJELkR5bmFtaWNzLmIyQ29udGFjdEZpbHRlciA9IChmdW5jdGlvbigpIHtcbiAgZnVuY3Rpb24gYjJDb250YWN0RmlsdGVyKCkge31cblxuICByZXR1cm4gYjJDb250YWN0RmlsdGVyO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMkNvbnRhY3RGaWx0ZXIuanMubWFwXG4iLCJ2YXIgQm94MkQ7XG5cbkJveDJEID0gcmVxdWlyZSgnLi4vLi4vaW5kZXgnKTtcblxuQm94MkQuRHluYW1pY3MuYjJDb250YWN0TGlzdGVuZXIgPSAoZnVuY3Rpb24oKSB7XG4gIGZ1bmN0aW9uIGIyQ29udGFjdExpc3RlbmVyKCkge31cblxuICBiMkNvbnRhY3RMaXN0ZW5lci5wcm90b3R5cGUuQmVnaW5Db250YWN0ID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkNvbnRhY3RMaXN0ZW5lci5wcm90b3R5cGUuRW5kQ29udGFjdCA9IGZ1bmN0aW9uKCkge307XG5cbiAgYjJDb250YWN0TGlzdGVuZXIucHJvdG90eXBlLlByZVNvbHZlID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkNvbnRhY3RMaXN0ZW5lci5wcm90b3R5cGUuUG9zdFNvbHZlID0gZnVuY3Rpb24oKSB7fTtcblxuICBiMkNvbnRhY3RMaXN0ZW5lci5iMl9kZWZhdWx0TGlzdGVuZXIgPSBuZXcgYjJDb250YWN0TGlzdGVuZXIoKTtcblxuICByZXR1cm4gYjJDb250YWN0TGlzdGVuZXI7XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIyQ29udGFjdExpc3RlbmVyLmpzLm1hcFxuIiwidmFyIEJveDJELCBiMkpvaW50LCBiMlZlYzI7XG5cbkJveDJEID0gcmVxdWlyZSgnLi4vLi4vaW5kZXgnKTtcblxuYjJKb2ludCA9IEJveDJELkR5bmFtaWNzLkpvaW50cy5iMkpvaW50O1xuXG5iMlZlYzIgPSBCb3gyRC5Db21tb24uTWF0aC5iMlZlYzI7XG5cbkJveDJELkR5bmFtaWNzLkpvaW50cy5iMkRpc3RhbmNlSm9pbnREZWYgPSAoZnVuY3Rpb24oKSB7XG4gIGIyRGlzdGFuY2VKb2ludERlZi5wcm90b3R5cGUudHlwZSA9IDA7XG5cbiAgYjJEaXN0YW5jZUpvaW50RGVmLnByb3RvdHlwZS5sb2NhbEFuY2hvckEgPSBudWxsO1xuXG4gIGIyRGlzdGFuY2VKb2ludERlZi5wcm90b3R5cGUubG9jYWxBbmNob3JCID0gbnVsbDtcblxuICBiMkRpc3RhbmNlSm9pbnREZWYucHJvdG90eXBlLnVzZXJEYXRhID0gbnVsbDtcblxuICBiMkRpc3RhbmNlSm9pbnREZWYucHJvdG90eXBlLmJvZHlBID0gbnVsbDtcblxuICBiMkRpc3RhbmNlSm9pbnREZWYucHJvdG90eXBlLmJvZHlCID0gbnVsbDtcblxuICBiMkRpc3RhbmNlSm9pbnREZWYucHJvdG90eXBlLmxlbmd0aCA9IDA7XG5cbiAgYjJEaXN0YW5jZUpvaW50RGVmLnByb3RvdHlwZS5mcmVxdWVuY3lIeiA9IDAuMDtcblxuICBiMkRpc3RhbmNlSm9pbnREZWYucHJvdG90eXBlLmRhbXBpbmdSYXRpbyA9IDAuMDtcblxuICBmdW5jdGlvbiBiMkRpc3RhbmNlSm9pbnREZWYoYkEsIGJCLCBhbmNob3JBLCBhbmNob3JCKSB7XG4gICAgdmFyIGRYLCBkWTtcbiAgICB0aGlzLnR5cGUgPSBiMkpvaW50LmVfZGlzdGFuY2VKb2ludDtcbiAgICB0aGlzLmxvY2FsQW5jaG9yQSA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLmxvY2FsQW5jaG9yQiA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLnVzZXJEYXRhID0gbnVsbDtcbiAgICBpZiAoYkEgIT09IHZvaWQgMCkge1xuICAgICAgdGhpcy5ib2R5QSA9IGJBO1xuICAgIH1cbiAgICBpZiAoYkIgIT09IHZvaWQgMCkge1xuICAgICAgdGhpcy5ib2R5QiA9IGJCO1xuICAgIH1cbiAgICBpZiAoYW5jaG9yQSAhPT0gdm9pZCAwKSB7XG4gICAgICB0aGlzLmxvY2FsQW5jaG9yQS5TZXRWKGFuY2hvckEpO1xuICAgIH1cbiAgICBpZiAoYW5jaG9yQiAhPT0gdm9pZCAwKSB7XG4gICAgICB0aGlzLmxvY2FsQW5jaG9yQi5TZXRWKGFuY2hvckIpO1xuICAgIH1cbiAgICBpZiAoYW5jaG9yQSAhPT0gdm9pZCAwICYmIGFuY2hvckIgIT09IHZvaWQgMCkge1xuICAgICAgZFggPSBhbmNob3JCLnggLSBhbmNob3JBLng7XG4gICAgICBkWSA9IGFuY2hvckIueSAtIGFuY2hvckEueTtcbiAgICAgIHRoaXMubGVuZ3RoID0gTWF0aC5zcXJ0KGRYICogZFggKyBkWSAqIGRZKTtcbiAgICB9XG4gICAgdGhpcy5mcmVxdWVuY3lIeiA9IDAuMDtcbiAgICB0aGlzLmRhbXBpbmdSYXRpbyA9IDAuMDtcbiAgICByZXR1cm47XG4gIH1cblxuICByZXR1cm4gYjJEaXN0YW5jZUpvaW50RGVmO1xuXG59KSgpO1xuXG4vLyMgc291cmNlTWFwcGluZ1VSTD1iMkRpc3RhbmNlSm9pbnREZWYuanMubWFwXG4iLCJ2YXIgQm94MkQ7XG5cbkJveDJEID0gcmVxdWlyZSgnLi4vLi4vaW5kZXgnKTtcblxuQm94MkQuRHluYW1pY3MuSm9pbnRzLmIySm9pbnQgPSAoZnVuY3Rpb24oKSB7XG4gIGIySm9pbnQuZV9kaXN0YW5jZUpvaW50ID0gMDtcblxuICBiMkpvaW50LmVfcmV2b2x1dGVKb2ludCA9IDE7XG5cbiAgYjJKb2ludC5wcm90b3R5cGUudXNlckRhdGEgPSBudWxsO1xuXG4gIGIySm9pbnQucHJvdG90eXBlLmJvZHlBID0gbnVsbDtcblxuICBiMkpvaW50LnByb3RvdHlwZS5ib2R5QiA9IG51bGw7XG5cbiAgYjJKb2ludC5wcm90b3R5cGUubGVuZ3RoID0gMDtcblxuICBiMkpvaW50LnByb3RvdHlwZS5uZXh0ID0gbnVsbDtcblxuICBmdW5jdGlvbiBiMkpvaW50KGRlZikge1xuICAgIHRoaXMuYm9keUEgPSBkZWYuYm9keUE7XG4gICAgdGhpcy5ib2R5QiA9IGRlZi5ib2R5QjtcbiAgICB0aGlzLnVzZXJEYXRhID0gZGVmLnVzZXJEYXRhO1xuICAgIHRoaXMudHlwZSA9IGRlZi50eXBlO1xuICAgIHRoaXMubmV4dCA9IG51bGw7XG4gICAgcmV0dXJuO1xuICB9XG5cbiAgYjJKb2ludC5wcm90b3R5cGUuR2V0Qm9keUEgPSBmdW5jdGlvbigpIHtcbiAgICByZXR1cm4gdGhpcy5ib2R5QTtcbiAgfTtcblxuICBiMkpvaW50LnByb3RvdHlwZS5HZXRCb2R5QiA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLmJvZHlCO1xuICB9O1xuXG4gIGIySm9pbnQucHJvdG90eXBlLkdldFVzZXJEYXRhID0gZnVuY3Rpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMudXNlckRhdGE7XG4gIH07XG5cbiAgYjJKb2ludC5wcm90b3R5cGUuR2V0VHlwZSA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLnR5cGU7XG4gIH07XG5cbiAgYjJKb2ludC5wcm90b3R5cGUuR2V0TmV4dCA9IGZ1bmN0aW9uKCkge1xuICAgIHJldHVybiB0aGlzLm5leHQ7XG4gIH07XG5cbiAgcmV0dXJuIGIySm9pbnQ7XG5cbn0pKCk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWIySm9pbnQuanMubWFwXG4iLCJ2YXIgQm94MkQsIGIySm9pbnQsIGIyVmVjMjtcblxuQm94MkQgPSByZXF1aXJlKCcuLi8uLi9pbmRleCcpO1xuXG5iMkpvaW50ID0gQm94MkQuRHluYW1pY3MuSm9pbnRzLmIySm9pbnQ7XG5cbmIyVmVjMiA9IEJveDJELkNvbW1vbi5NYXRoLmIyVmVjMjtcblxuQm94MkQuRHluYW1pY3MuSm9pbnRzLmIyUmV2b2x1dGVKb2ludERlZiA9IChmdW5jdGlvbigpIHtcbiAgYjJSZXZvbHV0ZUpvaW50RGVmLnByb3RvdHlwZS50eXBlID0gMDtcblxuICBiMlJldm9sdXRlSm9pbnREZWYucHJvdG90eXBlLmxvY2FsQW5jaG9yQSA9IG51bGw7XG5cbiAgYjJSZXZvbHV0ZUpvaW50RGVmLnByb3RvdHlwZS5sb2NhbEFuY2hvckIgPSBudWxsO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUudXNlckRhdGEgPSBudWxsO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUuYm9keUEgPSBudWxsO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUuYm9keUIgPSBudWxsO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUucmVmZXJlbmNlQW5nbGUgPSAwLjA7XG5cbiAgYjJSZXZvbHV0ZUpvaW50RGVmLnByb3RvdHlwZS5sb3dlckFuZ2xlID0gMC4wO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUudXBwZXJBbmdsZSA9IDAuMDtcblxuICBiMlJldm9sdXRlSm9pbnREZWYucHJvdG90eXBlLm1heE1vdG9yVG9ycXVlID0gMC4wO1xuXG4gIGIyUmV2b2x1dGVKb2ludERlZi5wcm90b3R5cGUubW90b3JTcGVlZCA9IDAuMDtcblxuICBiMlJldm9sdXRlSm9pbnREZWYucHJvdG90eXBlLmVuYWJsZUxpbWl0ID0gZmFsc2U7XG5cbiAgYjJSZXZvbHV0ZUpvaW50RGVmLnByb3RvdHlwZS5lbmFibGVNb3RvciA9IGZhbHNlO1xuXG4gIGZ1bmN0aW9uIGIyUmV2b2x1dGVKb2ludERlZihiQSwgYkIsIGFuY2hvckEsIGFuY2hvckIpIHtcbiAgICB0aGlzLnR5cGUgPSBiMkpvaW50LmVfcmV2b2x1dGVKb2ludDtcbiAgICB0aGlzLmxvY2FsQW5jaG9yQSA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLmxvY2FsQW5jaG9yQiA9IG5ldyBiMlZlYzIoKTtcbiAgICB0aGlzLnVzZXJEYXRhID0gbnVsbDtcbiAgICBpZiAoYkEgIT09IHZvaWQgMCkge1xuICAgICAgdGhpcy5ib2R5QSA9IGJBO1xuICAgIH1cbiAgICBpZiAoYkIgIT09IHZvaWQgMCkge1xuICAgICAgdGhpcy5ib2R5QiA9IGJCO1xuICAgIH1cbiAgICBpZiAoYW5jaG9yQSAhPT0gdm9pZCAwKSB7XG4gICAgICB0aGlzLmxvY2FsQW5jaG9yQS5TZXRWKGFuY2hvckEpO1xuICAgIH1cbiAgICBpZiAoYW5jaG9yQiAhPT0gdm9pZCAwKSB7XG4gICAgICB0aGlzLmxvY2FsQW5jaG9yQi5TZXRWKGFuY2hvckIpO1xuICAgIH1cbiAgICB0aGlzLnJlZmVyZW5jZUFuZ2xlID0gMC4wO1xuICAgIHRoaXMubG93ZXJBbmdsZSA9IDAuMDtcbiAgICB0aGlzLnVwcGVyQW5nbGUgPSAwLjA7XG4gICAgdGhpcy5tYXhNb3RvclRvcnF1ZSA9IDAuMDtcbiAgICB0aGlzLm1vdG9yU3BlZWQgPSAwLjA7XG4gICAgdGhpcy5lbmFibGVMaW1pdCA9IGZhbHNlO1xuICAgIHRoaXMuZW5hYmxlTW90b3IgPSBmYWxzZTtcbiAgICByZXR1cm47XG4gIH1cblxuICBiMlJldm9sdXRlSm9pbnREZWYucHJvdG90eXBlLkluaXRpYWxpemUgPSBmdW5jdGlvbihiQSwgYkIsIGFuY2hvcikge1xuICAgIHRoaXMuYm9keUEgPSBiQTtcbiAgICB0aGlzLmJvZHlCID0gYkI7XG4gICAgdGhpcy5sb2NhbEFuY2hvckEgPSB0aGlzLmJvZHlBLkdldExvY2FsUG9pbnQoYW5jaG9yKTtcbiAgICB0aGlzLmxvY2FsQW5jaG9yQiA9IHRoaXMuYm9keUIuR2V0TG9jYWxQb2ludChhbmNob3IpO1xuICAgIHRoaXMucmVmZXJlbmNlQW5nbGUgPSB0aGlzLmJvZHlCLkdldEFuZ2xlKCkgLSB0aGlzLmJvZHlBLkdldEFuZ2xlKCk7XG4gIH07XG5cbiAgcmV0dXJuIGIyUmV2b2x1dGVKb2ludERlZjtcblxufSkoKTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YjJSZXZvbHV0ZUpvaW50RGVmLmpzLm1hcFxuIiwiXG4vKlxuICovXG4ndXNlIHN0cmljdCc7XG52YXIgQm94MkQ7XG5cbm1vZHVsZS5leHBvcnRzID0gQm94MkQgPSAoZnVuY3Rpb24oKSB7XG4gIGZ1bmN0aW9uIEJveDJEKCkge31cblxuICBCb3gyRC5Db21tb24gPSAoZnVuY3Rpb24oKSB7XG4gICAgZnVuY3Rpb24gQ29tbW9uKCkge31cblxuICAgIHJldHVybiBDb21tb247XG5cbiAgfSkoKTtcblxuICByZXR1cm4gQm94MkQ7XG5cbn0pKCk7XG5cbkJveDJELkNvbW1vbi5NYXRoID0gKGZ1bmN0aW9uKCkge1xuICBmdW5jdGlvbiBNYXRoKCkge31cblxuICByZXR1cm4gTWF0aDtcblxufSkoKTtcblxucmVxdWlyZSgnLi9jb21tb24vbWF0aC9iMlZlYzInKTtcblxucmVxdWlyZSgnLi9jb21tb24vbWF0aC9iMk1hdDIyJyk7XG5cbnJlcXVpcmUoJy4vY29tbW9uL21hdGgvYjJUcmFuc2Zvcm0nKTtcblxucmVxdWlyZSgnLi9jb21tb24vbWF0aC9iMk1hdGgnKTtcblxuQm94MkQuQ29sbGlzaW9uID0gKGZ1bmN0aW9uKCkge1xuICBmdW5jdGlvbiBDb2xsaXNpb24oKSB7fVxuXG4gIHJldHVybiBDb2xsaXNpb247XG5cbn0pKCk7XG5cbkJveDJELkNvbGxpc2lvbi5TaGFwZXMgPSAoZnVuY3Rpb24oKSB7XG4gIGZ1bmN0aW9uIFNoYXBlcygpIHt9XG5cbiAgcmV0dXJuIFNoYXBlcztcblxufSkoKTtcblxucmVxdWlyZSgnLi9jb2xsaXNpb24vc2hhcGVzL2IyQ2lyY2xlU2hhcGUnKTtcblxucmVxdWlyZSgnLi9jb2xsaXNpb24vc2hhcGVzL2IyUG9seWdvblNoYXBlJyk7XG5cbkJveDJELkR5bmFtaWNzID0gKGZ1bmN0aW9uKCkge1xuICBmdW5jdGlvbiBEeW5hbWljcygpIHt9XG5cbiAgcmV0dXJuIER5bmFtaWNzO1xuXG59KSgpO1xuXG5Cb3gyRC5EeW5hbWljcy5Db250YWN0cyA9IChmdW5jdGlvbigpIHtcbiAgZnVuY3Rpb24gQ29udGFjdHMoKSB7fVxuXG4gIHJldHVybiBDb250YWN0cztcblxufSkoKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9jb250YWN0cy9iMkNvbnRhY3QnKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9jb250YWN0cy9iMkNvbnRhY3RGaWx0ZXInKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9jb250YWN0cy9iMkNvbnRhY3RMaXN0ZW5lcicpO1xuXG5Cb3gyRC5EeW5hbWljcy5Kb2ludHMgPSAoZnVuY3Rpb24oKSB7XG4gIGZ1bmN0aW9uIEpvaW50cygpIHt9XG5cbiAgcmV0dXJuIEpvaW50cztcblxufSkoKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9qb2ludHMvYjJKb2ludCcpO1xuXG5yZXF1aXJlKCcuL2R5bmFtaWNzL2pvaW50cy9iMkRpc3RhbmNlSm9pbnREZWYnKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9qb2ludHMvYjJSZXZvbHV0ZUpvaW50RGVmJyk7XG5cbnJlcXVpcmUoJy4vZHluYW1pY3MvYjJGaXh0dXJlJyk7XG5cbnJlcXVpcmUoJy4vZHluYW1pY3MvYjJCb2R5Jyk7XG5cbnJlcXVpcmUoJy4vZHluYW1pY3MvYjJEZWJ1Z0RyYXcnKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9iMkJvZHlEZWYnKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9iMkZpeHR1cmVEZWYnKTtcblxucmVxdWlyZSgnLi9keW5hbWljcy9iMldvcmxkJyk7XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWluZGV4LmpzLm1hcFxuIl19
