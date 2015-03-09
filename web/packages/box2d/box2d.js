!function(e){if("object"==typeof exports&&"undefined"!=typeof module)module.exports=e();else if("function"==typeof define&&define.amd)define([],e);else{var f;"undefined"!=typeof window?f=window:"undefined"!=typeof global?f=global:"undefined"!=typeof self&&(f=self),f.Box2D=e()}}(function(){var define,module,exports;return (function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
var Box2D, b2ContactID, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2ContactID = Box2D.Collision.b2ContactID;

Box2D.Collision.ClipVertex = (function() {
  ClipVertex.prototype.v = null;

  ClipVertex.prototype.id = null;

  function ClipVertex() {
    this.v = new b2Vec2();
    this.id = new b2ContactID();
    return;
  }

  ClipVertex.prototype.Set = function(other) {
    this.v.SetV(other.v);
    this.id.Set(other.id);
  };

  return ClipVertex;

})();

//# sourceMappingURL=ClipVertex.js.map

},{"../index":103}],2:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Collision.Features = (function() {
  function Features() {}

  Object.defineProperties(Features.prototype, {
    referenceEdge: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._referenceEdge;
      },
      set: function(value) {
        if (value === void 0) {
          value = 0;
        }
        this._referenceEdge = value;
        this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
      }
    },
    incidentEdge: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._incidentEdge;
      },
      set: function(value) {
        if (value === void 0) {
          value = 0;
        }
        this._incidentEdge = value;
        this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
      }
    },
    incidentVertex: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._incidentVertex;
      },
      set: function(value) {
        if (value === void 0) {
          value = 0;
        }
        this._incidentVertex = value;
        this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
      }
    },
    flip: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._flip;
      },
      set: function(value) {
        if (value === void 0) {
          value = 0;
        }
        this._flip = value;
        this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
      }
    }
  });

  return Features;

})();

//# sourceMappingURL=Features.js.map

},{"../index":103}],3:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2AABB = (function() {
  b2AABB.prototype.lowerBound = null;

  b2AABB.prototype.upperBound = null;

  function b2AABB() {
    this.lowerBound = new b2Vec2();
    this.upperBound = new b2Vec2();
    return;
  }

  b2AABB.prototype.IsValid = function() {
    var dX, dY, valid;
    dX = this.upperBound.x - this.lowerBound.x;
    dY = this.upperBound.y - this.lowerBound.y;
    valid = dX >= 0.0 && dY >= 0.0;
    valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
    return valid;
  };

  b2AABB.prototype.GetCenter = function() {
    return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
  };

  b2AABB.prototype.GetExtents = function() {
    return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
  };

  b2AABB.prototype.Contains = function(aabb) {
    var result;
    result = true;
    result = result && this.lowerBound.x <= aabb.lowerBound.x;
    result = result && this.lowerBound.y <= aabb.lowerBound.y;
    result = result && aabb.upperBound.x <= this.upperBound.x;
    result = result && aabb.upperBound.y <= this.upperBound.y;
    return result;
  };

  b2AABB.prototype.RayCast = function(output, input) {
    var absDX, absDY, dX, dY, inv_d, normal, pX, pY, s, t1, t2, t3, tmax, tmin;
    tmin = -Number.MAX_VALUE;
    tmax = Number.MAX_VALUE;
    pX = input.p1.x;
    pY = input.p1.y;
    dX = input.p2.x - input.p1.x;
    dY = input.p2.y - input.p1.y;
    absDX = Math.abs(dX);
    absDY = Math.abs(dY);
    normal = output.normal;
    inv_d = 0;
    t1 = 0;
    t2 = 0;
    t3 = 0;
    s = 0;
    if (absDX < Number.MIN_VALUE) {
      if (pX < this.lowerBound.x || this.upperBound.x < pX) {
        return false;
      }
    } else {
      inv_d = 1.0 / dX;
      t1 = (this.lowerBound.x - pX) * inv_d;
      t2 = (this.upperBound.x - pX) * inv_d;
      s = -1.0;
      if (t1 > t2) {
        t3 = t1;
        t1 = t2;
        t2 = t3;
        s = 1.0;
      }
      if (t1 > tmin) {
        normal.x = s;
        normal.y = 0;
        tmin = t1;
      }
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) {
        return false;
      }
    }
    if (absDY < Number.MIN_VALUE) {
      if (pY < this.lowerBound.y || this.upperBound.y < pY) {
        return false;
      }
    } else {
      inv_d = 1.0 / dY;
      t1 = (this.lowerBound.y - pY) * inv_d;
      t2 = (this.upperBound.y - pY) * inv_d;
      s = -1.0;
      if (t1 > t2) {
        t3 = t1;
        t1 = t2;
        t2 = t3;
        s = 1.0;
      }
      if (t1 > tmin) {
        normal.y = s;
        normal.x = 0;
        tmin = t1;
      }
      tmax = Math.min(tmax, t2);
      if (tmin > tmax) {
        return false;
      }
    }
    output.fraction = tmin;
    return true;
  };

  b2AABB.prototype.TestOverlap = function(other) {
    var d1X, d1Y, d2X, d2Y;
    d1X = other.lowerBound.x - this.upperBound.x;
    d1Y = other.lowerBound.y - this.upperBound.y;
    d2X = this.lowerBound.x - other.upperBound.x;
    d2Y = this.lowerBound.y - other.upperBound.y;
    if (d1X > 0.0 || d1Y > 0.0) {
      return false;
    }
    if (d2X > 0.0 || d2Y > 0.0) {
      return false;
    }
    return true;
  };

  b2AABB.Combine = function(aabb1, aabb2) {
    var aabb;
    aabb = new b2AABB();
    aabb.Combine(aabb1, aabb2);
    return aabb;
  };

  b2AABB.prototype.Combine = function(aabb1, aabb2) {
    this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
    this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
    this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
    this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
  };

  return b2AABB;

})();

//# sourceMappingURL=b2AABB.js.map

},{"../index":103}],4:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Collision.b2Bound = (function() {
  function b2Bound() {}

  b2Bound.prototype.value = 0;

  b2Bound.prototype.proxy = null;

  b2Bound.prototype.stabbingCount = 0;

  b2Bound.prototype.IsLower = function() {
    return (this.value & 1) === 0;
  };

  b2Bound.prototype.IsUpper = function() {
    return (this.value & 1) === 1;
  };

  b2Bound.prototype.Swap = function(b) {
    var tempProxy, tempStabbingCount, tempValue;
    tempValue = this.value;
    tempProxy = this.proxy;
    tempStabbingCount = this.stabbingCount;
    this.value = b.value;
    this.proxy = b.proxy;
    this.stabbingCount = b.stabbingCount;
    b.value = tempValue;
    b.proxy = tempProxy;
    b.stabbingCount = tempStabbingCount;
  };

  return b2Bound;

})();

//# sourceMappingURL=b2Bound.js.map

},{"../index":103}],5:[function(require,module,exports){
var Box2D, Vector;

Box2D = require('../index');

Vector = Box2D.Vector;

Box2D.Collision.b2BoundValues = (function() {
  b2BoundValues.prototype.lowerValues = null;

  b2BoundValues.prototype.upperValues = null;

  function b2BoundValues() {
    this.lowerValues = new Vector();
    this.lowerValues[0] = 0.0;
    this.lowerValues[1] = 0.0;
    this.upperValues = new Vector();
    this.upperValues[0] = 0.0;
    this.upperValues[1] = 0.0;
    return;
  }

  return b2BoundValues;

})();

//# sourceMappingURL=b2BoundValues.js.map

},{"../index":103}],6:[function(require,module,exports){
var Box2D, ClipVertex, Vector, b2Manifold, b2Settings, b2Vec2;

Box2D = require('../index');

Vector = Box2D.Vector;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Settings = Box2D.Common.b2Settings;

ClipVertex = Box2D.Collision.ClipVertex;

b2Manifold = Box2D.Collision.b2Manifold;

Box2D.Collision.b2Collision = (function() {
  function b2Collision() {}

  b2Collision.b2_nullFeature = 0x000000ff;

  b2Collision.s_edgeAO = new Vector(1);

  b2Collision.s_edgeBO = new Vector(1);

  b2Collision.s_localTangent = new b2Vec2();

  b2Collision.s_localNormal = new b2Vec2();

  b2Collision.s_planePoint = new b2Vec2();

  b2Collision.s_normal = new b2Vec2();

  b2Collision.s_tangent = new b2Vec2();

  b2Collision.s_tangent2 = new b2Vec2();

  b2Collision.s_v11 = new b2Vec2();

  b2Collision.s_v12 = new b2Vec2();

  b2Collision.b2CollidePolyTempVec = new b2Vec2();

  b2Collision.ClipSegmentToLine = function(vOut, vIn, normal, offset) {
    var cv, cv2, distance0, distance1, interp, numOut, tVec, vIn0, vIn1;
    if (offset === void 0) {
      offset = 0;
    }
    cv = void 0;
    numOut = 0;
    cv = vIn[0];
    vIn0 = cv.v;
    cv = vIn[1];
    vIn1 = cv.v;
    distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
    distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
    if (distance0 <= 0.0) {
      vOut[numOut++].Set(vIn[0]);
    }
    if (distance1 <= 0.0) {
      vOut[numOut++].Set(vIn[1]);
    }
    if (distance0 * distance1 < 0.0) {
      interp = distance0 / (distance0 - distance1);
      cv = vOut[numOut];
      tVec = cv.v;
      tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
      tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
      cv = vOut[numOut];
      cv2 = void 0;
      if (distance0 > 0.0) {
        cv2 = vIn[0];
        cv.id = cv2.id;
      } else {
        cv2 = vIn[1];
        cv.id = cv2.id;
      }
      ++numOut;
    }
    return numOut;
  };

  b2Collision.EdgeSeparation = function(poly1, xf1, edge1, poly2, xf2) {
    var count1, count2, dot, i, index, minDot, normal1WorldX, normal1WorldY, normal1X, normal1Y, normals1, separation, tMat, tVec, v1X, v1Y, v2X, v2Y, vertices1, vertices2;
    if (edge1 === void 0) {
      edge1 = 0;
    }
    count1 = parseInt(poly1.m_vertexCount);
    vertices1 = poly1.m_vertices;
    normals1 = poly1.m_normals;
    count2 = parseInt(poly2.m_vertexCount);
    vertices2 = poly2.m_vertices;
    tMat = void 0;
    tVec = void 0;
    tMat = xf1.R;
    tVec = normals1[edge1];
    normal1WorldX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
    normal1WorldY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
    tMat = xf2.R;
    normal1X = tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY;
    normal1Y = tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY;
    index = 0;
    minDot = Number.MAX_VALUE;
    i = 0;
    while (i < count2) {
      tVec = vertices2[i];
      dot = tVec.x * normal1X + tVec.y * normal1Y;
      if (dot < minDot) {
        minDot = dot;
        index = i;
      }
      ++i;
    }
    tVec = vertices1[edge1];
    tMat = xf1.R;
    v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tVec = vertices2[index];
    tMat = xf2.R;
    v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    v2X -= v1X;
    v2Y -= v1Y;
    separation = v2X * normal1WorldX + v2Y * normal1WorldY;
    return separation;
  };

  b2Collision.FindMaxSeparation = function(edgeIndex, poly1, xf1, poly2, xf2) {
    var bestEdge, bestSeparation, count1, dLocal1X, dLocal1Y, dX, dY, dot, edge, i, increment, maxDot, nextEdge, normals1, prevEdge, s, sNext, sPrev, tMat, tVec;
    count1 = parseInt(poly1.m_vertexCount);
    normals1 = poly1.m_normals;
    tVec = void 0;
    tMat = void 0;
    tMat = xf2.R;
    tVec = poly2.m_centroid;
    dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tMat = xf1.R;
    tVec = poly1.m_centroid;
    dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    dLocal1X = dX * xf1.R.col1.x + dY * xf1.R.col1.y;
    dLocal1Y = dX * xf1.R.col2.x + dY * xf1.R.col2.y;
    edge = 0;
    maxDot = -Number.MAX_VALUE;
    i = 0;
    while (i < count1) {
      tVec = normals1[i];
      dot = tVec.x * dLocal1X + tVec.y * dLocal1Y;
      if (dot > maxDot) {
        maxDot = dot;
        edge = i;
      }
      ++i;
    }
    s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
    prevEdge = parseInt((edge - 1 >= 0 ? edge - 1 : count1 - 1));
    sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
    nextEdge = parseInt((edge + 1 < count1 ? edge + 1 : 0));
    sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
    bestEdge = 0;
    bestSeparation = 0;
    increment = 0;
    if (sPrev > s && sPrev > sNext) {
      increment = -1.;
      bestEdge = prevEdge;
      bestSeparation = sPrev;
    } else if (sNext > s) {
      increment = 1;
      bestEdge = nextEdge;
      bestSeparation = sNext;
    } else {
      edgeIndex[0] = edge;
      return s;
    }
    while (true) {
      if (increment === (-1)) {
        edge = (bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1);
      } else {
        edge = (bestEdge + 1 < count1 ? bestEdge + 1 : 0);
      }
      s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
      if (s > bestSeparation) {
        bestEdge = edge;
        bestSeparation = s;
      } else {
        break;
      }
    }
    edgeIndex[0] = bestEdge;
    return bestSeparation;
  };

  b2Collision.FindIncidentEdge = function(c, poly1, xf1, edge1, poly2, xf2) {
    var count1, count2, dot, i, i1, i2, index, minDot, normal1X, normal1Y, normals1, normals2, tClip, tMat, tVec, tX, vertices2;
    if (edge1 === void 0) {
      edge1 = 0;
    }
    count1 = parseInt(poly1.m_vertexCount);
    normals1 = poly1.m_normals;
    count2 = parseInt(poly2.m_vertexCount);
    vertices2 = poly2.m_vertices;
    normals2 = poly2.m_normals;
    tMat = void 0;
    tVec = void 0;
    tMat = xf1.R;
    tVec = normals1[edge1];
    normal1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
    normal1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
    tMat = xf2.R;
    tX = tMat.col1.x * normal1X + tMat.col1.y * normal1Y;
    normal1Y = tMat.col2.x * normal1X + tMat.col2.y * normal1Y;
    normal1X = tX;
    index = 0;
    minDot = Number.MAX_VALUE;
    i = 0;
    while (i < count2) {
      tVec = normals2[i];
      dot = normal1X * tVec.x + normal1Y * tVec.y;
      if (dot < minDot) {
        minDot = dot;
        index = i;
      }
      ++i;
    }
    tClip = void 0;
    i1 = parseInt(index);
    i2 = parseInt((i1 + 1 < count2 ? i1 + 1 : 0));
    tClip = c[0];
    tVec = vertices2[i1];
    tMat = xf2.R;
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tClip.id.features.referenceEdge = edge1;
    tClip.id.features.incidentEdge = i1;
    tClip.id.features.incidentVertex = 0;
    tClip = c[1];
    tVec = vertices2[i2];
    tMat = xf2.R;
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tClip.id.features.referenceEdge = edge1;
    tClip.id.features.incidentEdge = i2;
    tClip.id.features.incidentVertex = 1;
  };

  b2Collision.MakeClipPointVector = function() {
    var r;
    r = new Array(2);
    r[0] = new ClipVertex();
    r[1] = new ClipVertex();
    return r;
  };

  b2Collision.CollidePolygons = function(manifold, polyA, xfA, polyB, xfB) {
    var clipPoints1, clipPoints2, count1, cp, cv, edge1, edgeA, edgeB, flip, frontOffset, i, incidentEdge, k_absoluteTol, k_relativeTol, localNormal, localTangent, local_v11, local_v12, normal, np, planePoint, pointCount, poly1, poly2, separation, separationA, separationB, sideOffset1, sideOffset2, tMat, tX, tY, tangent, tangent2, totalRadius, v11, v12, vertices1, xf1, xf2;
    cv = void 0;
    manifold.m_pointCount = 0;
    totalRadius = polyA.m_radius + polyB.m_radius;
    edgeA = 0;
    b2Collision.s_edgeAO[0] = edgeA;
    separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
    edgeA = b2Collision.s_edgeAO[0];
    if (separationA > totalRadius) {
      return;
    }
    edgeB = 0;
    b2Collision.s_edgeBO[0] = edgeB;
    separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
    edgeB = b2Collision.s_edgeBO[0];
    if (separationB > totalRadius) {
      return;
    }
    poly1 = void 0;
    poly2 = void 0;
    xf1 = void 0;
    xf2 = void 0;
    edge1 = 0;
    flip = 0;
    k_relativeTol = 0.98;
    k_absoluteTol = 0.001;
    tMat = void 0;
    if (separationB > k_relativeTol * separationA + k_absoluteTol) {
      poly1 = polyB;
      poly2 = polyA;
      xf1 = xfB;
      xf2 = xfA;
      edge1 = edgeB;
      manifold.m_type = b2Manifold.e_faceB;
      flip = 1;
    } else {
      poly1 = polyA;
      poly2 = polyB;
      xf1 = xfA;
      xf2 = xfB;
      edge1 = edgeA;
      manifold.m_type = b2Manifold.e_faceA;
      flip = 0;
    }
    incidentEdge = b2Collision.s_incidentEdge;
    b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
    count1 = parseInt(poly1.m_vertexCount);
    vertices1 = poly1.m_vertices;
    local_v11 = vertices1[edge1];
    local_v12 = void 0;
    if (edge1 + 1 < count1) {
      local_v12 = vertices1[parseInt(edge1 + 1)];
    } else {
      local_v12 = vertices1[0];
    }
    localTangent = b2Collision.s_localTangent;
    localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
    localTangent.Normalize();
    localNormal = b2Collision.s_localNormal;
    localNormal.x = localTangent.y;
    localNormal.y = -localTangent.x;
    planePoint = b2Collision.s_planePoint;
    planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
    tangent = b2Collision.s_tangent;
    tMat = xf1.R;
    tangent.x = tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y;
    tangent.y = tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y;
    tangent2 = b2Collision.s_tangent2;
    tangent2.x = -tangent.x;
    tangent2.y = -tangent.y;
    normal = b2Collision.s_normal;
    normal.x = tangent.y;
    normal.y = -tangent.x;
    v11 = b2Collision.s_v11;
    v12 = b2Collision.s_v12;
    v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
    v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
    v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
    v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);
    frontOffset = normal.x * v11.x + normal.y * v11.y;
    sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius;
    sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
    clipPoints1 = b2Collision.s_clipPoints1;
    clipPoints2 = b2Collision.s_clipPoints2;
    np = 0;
    np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);
    if (np < 2) {
      return;
    }
    np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
    if (np < 2) {
      return;
    }
    manifold.m_localPlaneNormal.SetV(localNormal);
    manifold.m_localPoint.SetV(planePoint);
    pointCount = 0;
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      cv = clipPoints2[i];
      separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
      if (separation <= totalRadius) {
        cp = manifold.m_points[pointCount];
        tMat = xf2.R;
        tX = cv.v.x - xf2.position.x;
        tY = cv.v.y - xf2.position.y;
        cp.m_localPoint.x = tX * tMat.col1.x + tY * tMat.col1.y;
        cp.m_localPoint.y = tX * tMat.col2.x + tY * tMat.col2.y;
        cp.m_id.Set(cv.id);
        cp.m_id.features.flip = flip;
        ++pointCount;
      }
      ++i;
    }
    manifold.m_pointCount = pointCount;
  };

  b2Collision.CollideCircles = function(manifold, circle1, xf1, circle2, xf2) {
    var dX, dY, distSqr, p1X, p1Y, p2X, p2Y, radius, tMat, tVec;
    manifold.m_pointCount = 0;
    tMat = void 0;
    tVec = void 0;
    tMat = xf1.R;
    tVec = circle1.m_p;
    p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tMat = xf2.R;
    tVec = circle2.m_p;
    p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    dX = p2X - p1X;
    dY = p2Y - p1Y;
    distSqr = dX * dX + dY * dY;
    radius = circle1.m_radius + circle2.m_radius;
    if (distSqr > radius * radius) {
      return;
    }
    manifold.m_type = b2Manifold.e_circles;
    manifold.m_localPoint.SetV(circle1.m_p);
    manifold.m_localPlaneNormal.SetZero();
    manifold.m_pointCount = 1;
    manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
    manifold.m_points[0].m_id.key = 0;
  };

  b2Collision.CollidePolygonAndCircle = function(manifold, polygon, xf1, circle, xf2) {
    var cLocalX, cLocalY, cX, cY, dX, dY, dist, faceCenterX, faceCenterY, i, normalIndex, normals, positionX, positionY, radius, s, separation, tMat, tPoint, tVec, u1, u2, v1, v2, vertIndex1, vertIndex2, vertexCount, vertices;
    manifold.m_pointCount = 0;
    tPoint = void 0;
    dX = 0;
    dY = 0;
    positionX = 0;
    positionY = 0;
    tVec = void 0;
    tMat = void 0;
    tMat = xf2.R;
    tVec = circle.m_p;
    cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    dX = cX - xf1.position.x;
    dY = cY - xf1.position.y;
    tMat = xf1.R;
    cLocalX = dX * tMat.col1.x + dY * tMat.col1.y;
    cLocalY = dX * tMat.col2.x + dY * tMat.col2.y;
    dist = 0;
    normalIndex = 0;
    separation = -Number.MAX_VALUE;
    radius = polygon.m_radius + circle.m_radius;
    vertexCount = parseInt(polygon.m_vertexCount);
    vertices = polygon.m_vertices;
    normals = polygon.m_normals;
    i = 0;
    while (i < vertexCount) {
      tVec = vertices[i];
      dX = cLocalX - tVec.x;
      dY = cLocalY - tVec.y;
      tVec = normals[i];
      s = tVec.x * dX + tVec.y * dY;
      if (s > radius) {
        return;
      }
      if (s > separation) {
        separation = s;
        normalIndex = i;
      }
      ++i;
    }
    vertIndex1 = parseInt(normalIndex);
    vertIndex2 = parseInt((vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0));
    v1 = vertices[vertIndex1];
    v2 = vertices[vertIndex2];
    if (separation < Number.MIN_VALUE) {
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
      manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
      manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
      return;
    }
    u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
    u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
    if (u1 <= 0.0) {
      if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) {
        return;
      }
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = cLocalX - v1.x;
      manifold.m_localPlaneNormal.y = cLocalY - v1.y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.SetV(v1);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    } else if (u2 <= 0) {
      if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) {
        return;
      }
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = cLocalX - v2.x;
      manifold.m_localPlaneNormal.y = cLocalY - v2.y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.SetV(v2);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    } else {
      faceCenterX = 0.5 * (v1.x + v2.x);
      faceCenterY = 0.5 * (v1.y + v2.y);
      separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
      if (separation > radius) {
        return;
      }
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
      manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.Set(faceCenterX, faceCenterY);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    }
  };

  b2Collision.TestOverlap = function(a, b) {
    var d1X, d1Y, d2X, d2Y, t1, t2;
    t1 = b.lowerBound;
    t2 = a.upperBound;
    d1X = t1.x - t2.x;
    d1Y = t1.y - t2.y;
    t1 = a.lowerBound;
    t2 = b.upperBound;
    d2X = t1.x - t2.x;
    d2Y = t1.y - t2.y;
    if (d1X > 0.0 || d1Y > 0.0) {
      return false;
    }
    if (d2X > 0.0 || d2Y > 0.0) {
      return false;
    }
    return true;
  };

  b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();

  b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();

  b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();

  return b2Collision;

})();

//# sourceMappingURL=b2Collision.js.map

},{"../index":103}],7:[function(require,module,exports){
var Box2D, Features;

Box2D = require('../index');

Features = Box2D.Collision.Features;

Box2D.Collision.b2ContactID = (function() {
  b2ContactID.prototype.key = null;

  b2ContactID.prototype.features = null;

  function b2ContactID() {
    this.features = new Features();
    this.features._m_id = this;
    return;
  }

  b2ContactID.prototype.Set = function(id) {
    this.key = id._key;
  };

  b2ContactID.prototype.Copy = function() {
    var id;
    id = new b2ContactID();
    id.key = this.key;
    return id;
  };

  Object.defineProperties(b2ContactID.prototype, {
    key: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._key;
      },
      set: function(value) {
        if (value === void 0) {
          value = 0;
        }
        this._key = value;
        this.features._referenceEdge = this._key & 0x000000ff;
        this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
        this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
        this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
      }
    }
  });

  return b2ContactID;

})();

//# sourceMappingURL=b2ContactID.js.map

},{"../index":103}],8:[function(require,module,exports){
var Box2D, b2ContactID, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2ContactID = Box2D.Collision.b2ContactID;

Box2D.Collision.b2ContactPoint = (function() {
  b2ContactPoint.prototype.position = null;

  b2ContactPoint.prototype.velocity = null;

  b2ContactPoint.prototype.normal = null;

  b2ContactPoint.prototype.id = null;

  function b2ContactPoint() {
    this.position = new b2Vec2();
    this.velocity = new b2Vec2();
    this.normal = new b2Vec2();
    this.id = new b2ContactID();
    return;
  }

  return b2ContactPoint;

})();

//# sourceMappingURL=b2ContactPoint.js.map

},{"../index":103}],9:[function(require,module,exports){
var Box2D, Vector, b2Math, b2Settings, b2Simplex;

Box2D = require('../index');

Vector = Box2D.Vector;

b2Math = Box2D.Common.Math.b2Math;

b2Simplex = Box2D.Collision.b2Simplex;

b2Settings = Box2D.Common.b2Settings;

Box2D.Collision.b2Distance = (function() {
  function b2Distance() {}

  b2Distance.Distance = function(output, cache, input) {
    var closestPoint, d, distanceSqr1, distanceSqr2, duplicate, i, iter, k_maxIters, normal, p, proxyA, proxyB, rA, rB, saveA, saveB, saveCount, simplex, transformA, transformB, vertex, vertices;
    proxyA = input.proxyA;
    proxyB = input.proxyB;
    transformA = input.transformA;
    transformB = input.transformB;
    simplex = b2Distance.s_simplex;
    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
    vertices = simplex.m_vertices;
    k_maxIters = 20;
    saveA = b2Distance.s_saveA;
    saveB = b2Distance.s_saveB;
    saveCount = 0;
    closestPoint = simplex.GetClosestPoint();
    distanceSqr1 = closestPoint.LengthSquared();
    distanceSqr2 = distanceSqr1;
    i = 0;
    p = void 0;
    iter = 0;
    while (iter < k_maxIters) {
      saveCount = simplex.m_count;
      i = 0;
      while (i < saveCount) {
        saveA[i] = vertices[i].indexA;
        saveB[i] = vertices[i].indexB;
        i++;
      }
      switch (simplex.m_count) {
        case 1:
          i = i;
          break;
        case 2:
          simplex.Solve2();
          break;
        case 3:
          simplex.Solve3();
          break;
        default:
          b2Settings.b2Assert(false);
      }
      if (simplex.m_count === 3) {
        break;
      }
      p = simplex.GetClosestPoint();
      distanceSqr2 = p.LengthSquared();

      /**
      todo: this looks like a bug
       */
      distanceSqr1 = distanceSqr2;
      d = simplex.GetSearchDirection();
      if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
        break;
      }
      vertex = vertices[simplex.m_count];
      vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
      vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
      vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
      vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
      vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
      ++iter;
      ++b2Distance.b2_gjkIters;
      duplicate = false;
      i = 0;
      while (i < saveCount) {
        if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
          duplicate = true;
          break;
        }
        i++;
      }
      if (duplicate) {
        break;
      }
      ++simplex.m_count;
    }
    b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);
    simplex.GetWitnessPoints(output.pointA, output.pointB);
    output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
    output.iterations = iter;
    simplex.WriteCache(cache);
    if (input.useRadii) {
      rA = proxyA.m_radius;
      rB = proxyB.m_radius;
      if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
        output.distance -= rA + rB;
        normal = b2Math.SubtractVV(output.pointB, output.pointA);
        normal.Normalize();
        output.pointA.x += rA * normal.x;
        output.pointA.y += rA * normal.y;
        output.pointB.x -= rB * normal.x;
        output.pointB.y -= rB * normal.y;
      } else {
        p = new b2Vec2();
        p.x = .5 * (output.pointA.x + output.pointB.x);
        p.y = .5 * (output.pointA.y + output.pointB.y);
        output.pointA.x = output.pointB.x = p.x;
        output.pointA.y = output.pointB.y = p.y;
        output.distance = 0.0;
      }
    }
  };

  b2Distance.s_simplex = new b2Simplex();

  b2Distance.s_saveA = new Vector(3);

  b2Distance.s_saveB = new Vector(3);

  return b2Distance;

})();

//# sourceMappingURL=b2Distance.js.map

},{"../index":103}],10:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Collision.b2DistanceInput = (function() {
  function b2DistanceInput() {}

  return b2DistanceInput;

})();

//# sourceMappingURL=b2DistanceInput.js.map

},{"../index":103}],11:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2DistanceOutput = (function() {
  b2DistanceOutput.prototype.pointA = null;

  b2DistanceOutput.prototype.pointB = null;

  function b2DistanceOutput() {
    this.pointA = new b2Vec2();
    this.pointB = new b2Vec2();
    return;
  }

  return b2DistanceOutput;

})();

//# sourceMappingURL=b2DistanceOutput.js.map

},{"../index":103}],12:[function(require,module,exports){
var Box2D, b2CircleShape, b2PolygonShape, b2Settings, b2Shape;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Shape = Box2D.Collision.Shapes.b2Shape;

b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;

b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

Box2D.Collision.b2DistanceProxy = (function() {
  function b2DistanceProxy() {}

  b2DistanceProxy.prototype.m_vertices = null;

  b2DistanceProxy.prototype.m_count = 0;

  b2DistanceProxy.prototype.m_radius = 0;

  b2DistanceProxy.prototype.Set = function(shape) {
    var circle, polygon;
    switch (shape.GetType()) {
      case b2Shape.e_circleShape:
        circle = (shape instanceof b2CircleShape ? shape : null);
        this.m_vertices = new Array(1, true);
        this.m_vertices[0] = circle.m_p;
        this.m_count = 1;
        this.m_radius = circle.m_radius;
        break;
      case b2Shape.e_polygonShape:
        polygon = (shape instanceof b2PolygonShape ? shape : null);
        this.m_vertices = polygon.m_vertices;
        this.m_count = polygon.m_vertexCount;
        this.m_radius = polygon.m_radius;
        break;
      default:
        b2Settings.b2Assert(false);
    }
  };

  b2DistanceProxy.prototype.GetSupport = function(d) {
    var bestIndex, bestValue, i, value;
    bestIndex = 0;
    bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    i = 1;
    while (i < this.m_count) {
      value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
      ++i;
    }
    return bestIndex;
  };

  b2DistanceProxy.prototype.GetSupportVertex = function(d) {
    var bestIndex, bestValue, i, value;
    bestIndex = 0;
    bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    i = 1;
    while (i < this.m_count) {
      value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
      ++i;
    }
    return this.m_vertices[bestIndex];
  };

  b2DistanceProxy.prototype.GetVertexCount = function() {
    return this.m_count;
  };

  b2DistanceProxy.prototype.GetVertex = function(index) {
    if (index === void 0) {
      index = 0;
    }
    b2Settings.b2Assert(0 <= index && index < this.m_count);
    return this.m_vertices[index];
  };

  return b2DistanceProxy;

})();

//# sourceMappingURL=b2DistanceProxy.js.map

},{"../index":103}],13:[function(require,module,exports){
var Box2D, b2AABB, b2DynamicTreeNode, b2Math, b2RayCastInput, b2Settings;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2AABB = Box2D.Collision.b2AABB;

b2RayCastInput = Box2D.Collision.b2RayCastInput;

b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode;

Box2D.Collision.b2DynamicTree = (function() {
  function b2DynamicTree() {
    this.m_root = null;
    this.m_freeList = null;
    this.m_path = 0;
    this.m_insertionCount = 0;
    return;
  }

  b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
    var extendX, extendY, node;
    node = this.AllocateNode();
    extendX = b2Settings.b2_aabbExtension;
    extendY = b2Settings.b2_aabbExtension;
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    node.aabb.upperBound.x = aabb.upperBound.x + extendX;
    node.aabb.upperBound.y = aabb.upperBound.y + extendY;
    node.userData = userData;
    this.InsertLeaf(node);
    return node;
  };

  b2DynamicTree.prototype.DestroyProxy = function(proxy) {
    this.RemoveLeaf(proxy);
    this.FreeNode(proxy);
  };

  b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
    var extendX, extendY;
    b2Settings.b2Assert(proxy.IsLeaf());
    if (proxy.aabb.Contains(aabb)) {
      return false;
    }
    this.RemoveLeaf(proxy);
    extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : -displacement.x);
    extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : -displacement.y);
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
    this.InsertLeaf(proxy);
    return true;
  };

  b2DynamicTree.prototype.Rebalance = function(iterations) {
    var bit, i, node;
    if (iterations === void 0) {
      iterations = 0;
    }
    if (this.m_root == null) {
      return;
    }
    i = 0;
    while (i < iterations) {
      node = this.m_root;
      bit = 0;
      while (node.IsLeaf() === false) {
        node = ((this.m_path >> bit) & 1 ? node.child2 : node.child1);
        bit = (bit + 1) & 31;
      }
      ++this.m_path;
      this.RemoveLeaf(node);
      this.InsertLeaf(node);
      i++;
    }
  };

  b2DynamicTree.prototype.GetFatAABB = function(proxy) {
    return proxy.aabb;
  };

  b2DynamicTree.prototype.GetUserData = function(proxy) {
    return proxy.userData;
  };

  b2DynamicTree.prototype.Query = function(callback, aabb) {
    var count, node, proceed, stack;
    if (this.m_root == null) {
      return;
    }
    stack = new Array();
    count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
      node = stack[--count];
      if (node.aabb.TestOverlap(aabb)) {
        if (node.IsLeaf()) {
          proceed = callback(node);
          if (!proceed) {
            return;
          }
        } else {
          stack[count++] = node.child1;
          stack[count++] = node.child2;
        }
      }
    }
  };

  b2DynamicTree.prototype.RayCast = function(callback, input) {
    var abs_v, c, count, h, maxFraction, node, p1, p2, r, segmentAABB, separation, stack, subInput, tX, tY, v;
    if (this.m_root == null) {
      return;
    }
    p1 = input.p1;
    p2 = input.p2;
    r = b2Math.SubtractVV(p1, p2);
    r.Normalize();
    v = b2Math.CrossFV(1.0, r);
    abs_v = b2Math.AbsV(v);
    maxFraction = input.maxFraction;
    segmentAABB = new b2AABB();
    tX = 0;
    tY = 0;
    tX = p1.x + maxFraction * (p2.x - p1.x);
    tY = p1.y + maxFraction * (p2.y - p1.y);
    segmentAABB.lowerBound.x = Math.min(p1.x, tX);
    segmentAABB.lowerBound.y = Math.min(p1.y, tY);
    segmentAABB.upperBound.x = Math.max(p1.x, tX);
    segmentAABB.upperBound.y = Math.max(p1.y, tY);
    stack = new Array();
    count = 0;
    stack[count++] = this.m_root;
    while (count > 0) {
      node = stack[--count];
      if (node.aabb.TestOverlap(segmentAABB) === false) {
        continue;
      }
      c = node.aabb.GetCenter();
      h = node.aabb.GetExtents();
      separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
      if (separation > 0.0) {
        continue;
      }
      if (node.IsLeaf()) {
        subInput = new b2RayCastInput();
        subInput.p1 = input.p1;
        subInput.p2 = input.p2;
        subInput.maxFraction = input.maxFraction;
        maxFraction = callback(subInput, node);
        if (maxFraction === 0.0) {
          return;
        }
        if (maxFraction > 0.0) {
          tX = p1.x + maxFraction * (p2.x - p1.x);
          tY = p1.y + maxFraction * (p2.y - p1.y);
          segmentAABB.lowerBound.x = Math.min(p1.x, tX);
          segmentAABB.lowerBound.y = Math.min(p1.y, tY);
          segmentAABB.upperBound.x = Math.max(p1.x, tX);
          segmentAABB.upperBound.y = Math.max(p1.y, tY);
        }
      } else {
        stack[count++] = node.child1;
        stack[count++] = node.child2;
      }
    }
  };

  b2DynamicTree.prototype.AllocateNode = function() {
    var node;
    if (this.m_freeList) {
      node = this.m_freeList;
      this.m_freeList = node.parent;
      node.parent = null;
      node.child1 = null;
      node.child2 = null;
      return node;
    }
    return new b2DynamicTreeNode();
  };

  b2DynamicTree.prototype.FreeNode = function(node) {
    node.parent = this.m_freeList;
    this.m_freeList = node;
  };

  b2DynamicTree.prototype.InsertLeaf = function(leaf) {
    var center, child1, child2, node1, node2, norm1, norm2, sibling;
    ++this.m_insertionCount;
    if (this.m_root == null) {
      this.m_root = leaf;
      this.m_root.parent = null;
      return;
    }
    center = leaf.aabb.GetCenter();
    sibling = this.m_root;
    if (sibling.IsLeaf() === false) {
      while (true) {
        child1 = sibling.child1;
        child2 = sibling.child2;
        norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
        norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
        if (norm1 < norm2) {
          sibling = child1;
        } else {
          sibling = child2;
        }
        if (sibling.IsLeaf() !== false) {
          break;
        }
      }
    }
    node1 = sibling.parent;
    node2 = this.AllocateNode();
    node2.parent = node1;
    node2.userData = null;
    node2.aabb.Combine(leaf.aabb, sibling.aabb);
    if (node1) {
      if (sibling.parent.child1 === sibling) {
        node1.child1 = node2;
      } else {
        node1.child2 = node2;
      }
      node2.child1 = sibling;
      node2.child2 = leaf;
      sibling.parent = node2;
      leaf.parent = node2;
      while (true) {
        if (node1.aabb.Contains(node2.aabb)) {
          break;
        }
        node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
        node2 = node1;
        node1 = node1.parent;
        if (!node1) {
          break;
        }
      }
    } else {
      node2.child1 = sibling;
      node2.child2 = leaf;
      sibling.parent = node2;
      leaf.parent = node2;
      this.m_root = node2;
    }
  };

  b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
    var node1, node2, oldAABB, sibling;
    if (leaf === this.m_root) {
      this.m_root = null;
      return;
    }
    node2 = leaf.parent;
    node1 = node2.parent;
    sibling = void 0;
    if (node2.child1 === leaf) {
      sibling = node2.child2;
    } else {
      sibling = node2.child1;
    }
    if (node1) {
      if (node1.child1 === node2) {
        node1.child1 = sibling;
      } else {
        node1.child2 = sibling;
      }
      sibling.parent = node1;
      this.FreeNode(node2);
      while (node1) {
        oldAABB = node1.aabb;
        node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
        if (oldAABB.Contains(node1.aabb)) {
          break;
        }
        node1 = node1.parent;
      }
    } else {
      this.m_root = sibling;
      sibling.parent = null;
      this.FreeNode(node2);
    }
  };

  return b2DynamicTree;

})();

//# sourceMappingURL=b2DynamicTree.js.map

},{"../index":103}],14:[function(require,module,exports){
var Box2D, b2DynamicTree, b2DynamicTreePair;

Box2D = require('../index');

b2DynamicTree = Box2D.Collision.b2DynamicTree;

b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair;

Box2D.Collision.b2DynamicTreeBroadPhase = (function() {
  b2DynamicTreeBroadPhase.prototype.m_tree = null;

  b2DynamicTreeBroadPhase.prototype.m_moveBuffer = null;

  b2DynamicTreeBroadPhase.prototype.m_pairBuffer = null;

  b2DynamicTreeBroadPhase.prototype.m_pairCount = 0;

  b2DynamicTreeBroadPhase.m_proxyCount = 0;

  function b2DynamicTreeBroadPhase() {
    this.m_tree = new b2DynamicTree();
    this.m_moveBuffer = new Array();
    this.m_pairBuffer = new Array();
    return;
  }

  b2DynamicTreeBroadPhase.prototype.CreateProxy = function(aabb, userData) {
    var proxy;
    proxy = this.m_tree.CreateProxy(aabb, userData);
    ++this.m_proxyCount;
    this.BufferMove(proxy);
    return proxy;
  };

  b2DynamicTreeBroadPhase.prototype.DestroyProxy = function(proxy) {
    this.UnBufferMove(proxy);
    --this.m_proxyCount;
    this.m_tree.DestroyProxy(proxy);
  };

  b2DynamicTreeBroadPhase.prototype.MoveProxy = function(proxy, aabb, displacement) {
    var buffer;
    buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
    if (buffer) {
      this.BufferMove(proxy);
    }
  };

  b2DynamicTreeBroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
    var aabbA, aabbB;
    aabbA = this.m_tree.GetFatAABB(proxyA);
    aabbB = this.m_tree.GetFatAABB(proxyB);
    return aabbA.TestOverlap(aabbB);
  };

  b2DynamicTreeBroadPhase.prototype.GetUserData = function(proxy) {
    return this.m_tree.GetUserData(proxy);
  };

  b2DynamicTreeBroadPhase.prototype.GetFatAABB = function(proxy) {
    return this.m_tree.GetFatAABB(proxy);
  };

  b2DynamicTreeBroadPhase.prototype.GetProxyCount = function() {
    return this.m_proxyCount;
  };

  b2DynamicTreeBroadPhase.prototype.UpdatePairs = function(callback) {
    var QueryCallback, fatAABB, i, pair, primaryPair, queryProxy, userDataA, userDataB;
    this.m_pairCount = 0;
    i = 0;
    queryProxy = void 0;
    i = 0;
    while (i < this.m_moveBuffer.length) {
      QueryCallback = (function(_this) {
        return function(proxy) {
          var pair;
          if (proxy === queryProxy) {
            return true;
          }
          if (_this.m_pairCount === _this.m_pairBuffer.length) {
            _this.m_pairBuffer[_this.m_pairCount] = new b2DynamicTreePair();
          }
          pair = _this.m_pairBuffer[_this.m_pairCount];
          pair.proxyA = (proxy < queryProxy ? proxy : queryProxy);
          pair.proxyB = (proxy >= queryProxy ? proxy : queryProxy);
          ++_this.m_pairCount;
          return true;
        };
      })(this);
      queryProxy = this.m_moveBuffer[i];
      fatAABB = this.m_tree.GetFatAABB(queryProxy);
      this.m_tree.Query(QueryCallback, fatAABB);
      ++i;
    }
    this.m_moveBuffer.length = 0;
    i = 0;
    while (i < this.m_pairCount) {
      primaryPair = this.m_pairBuffer[i];
      userDataA = this.m_tree.GetUserData(primaryPair.proxyA);
      userDataB = this.m_tree.GetUserData(primaryPair.proxyB);
      callback(userDataA, userDataB);
      ++i;
      while (i < this.m_pairCount) {
        pair = this.m_pairBuffer[i];
        if (pair.proxyA !== primaryPair.proxyA || pair.proxyB !== primaryPair.proxyB) {
          break;
        }
        ++i;
      }
    }
  };

  b2DynamicTreeBroadPhase.prototype.Query = function(callback, aabb) {
    this.m_tree.Query(callback, aabb);
  };

  b2DynamicTreeBroadPhase.prototype.RayCast = function(callback, input) {
    this.m_tree.RayCast(callback, input);
  };

  b2DynamicTreeBroadPhase.prototype.Validate = function() {};

  b2DynamicTreeBroadPhase.prototype.Rebalance = function(iterations) {
    if (iterations === void 0) {
      iterations = 0;
    }
    this.m_tree.Rebalance(iterations);
  };

  b2DynamicTreeBroadPhase.prototype.BufferMove = function(proxy) {
    this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
  };

  b2DynamicTreeBroadPhase.prototype.UnBufferMove = function(proxy) {
    var i;
    i = parseInt(this.m_moveBuffer.indexOf(proxy));
    this.m_moveBuffer.splice(i, 1);
  };

  b2DynamicTreeBroadPhase.prototype.ComparePairs = function(pair1, pair2) {
    return 0;
  };

  b2DynamicTreeBroadPhase.__implements = {
    IBroadPhase: true
  };

  return b2DynamicTreeBroadPhase;

})();

//# sourceMappingURL=b2DynamicTreeBroadPhase.js.map

},{"../index":103}],15:[function(require,module,exports){
var Box2D, b2AABB;

Box2D = require('../index');

b2AABB = Box2D.Collision.b2AABB;

Box2D.Collision.b2DynamicTreeNode = (function() {
  b2DynamicTreeNode.prototype.aabb = null;

  b2DynamicTreeNode.prototype.child1 = null;

  function b2DynamicTreeNode() {
    this.aabb = new b2AABB();
    return;
  }

  b2DynamicTreeNode.prototype.IsLeaf = function() {
    return this.child1 == null;
  };

  return b2DynamicTreeNode;

})();

//# sourceMappingURL=b2DynamicTreeNode.js.map

},{"../index":103}],16:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Collision.b2DynamicTreePair = (function() {
  function b2DynamicTreePair() {}

  return b2DynamicTreePair;

})();

//# sourceMappingURL=b2DynamicTreePair.js.map

},{"../index":103}],17:[function(require,module,exports){
var Box2D, b2ManifoldPoint, b2Settings, b2Vec2;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint;

Box2D.Collision.b2Manifold = (function() {
  b2Manifold.e_circles = 0x0001;

  b2Manifold.e_faceA = 0x0002;

  b2Manifold.e_faceB = 0x0004;

  b2Manifold.prototype.m_type = 0;

  b2Manifold.prototype.m_pointCount = 0;

  b2Manifold.prototype.m_points = null;

  b2Manifold.prototype.m_localPlaneNormal = null;

  b2Manifold.prototype.m_localPoint = null;

  function b2Manifold() {
    var i;
    this.m_pointCount = 0;
    this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      this.m_points[i] = new b2ManifoldPoint();
      i++;
    }
    this.m_localPlaneNormal = new b2Vec2();
    this.m_localPoint = new b2Vec2();
    return;
  }

  b2Manifold.prototype.Reset = function() {
    var i;
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      (this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null).Reset();
      i++;
    }
    this.m_localPlaneNormal.SetZero();
    this.m_localPoint.SetZero();
    this.m_type = 0;
    this.m_pointCount = 0;
  };

  b2Manifold.prototype.Set = function(m) {
    var i;
    this.m_pointCount = m.m_pointCount;
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      (this.m_points[i] instanceof b2ManifoldPoint ? this.m_points[i] : null).Set(m.m_points[i]);
      i++;
    }
    this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
    this.m_localPoint.SetV(m.m_localPoint);
    this.m_type = m.m_type;
  };

  b2Manifold.prototype.Copy = function() {
    var copy;
    copy = new b2Manifold();
    copy.Set(this);
    return copy;
  };

  return b2Manifold;

})();

//# sourceMappingURL=b2Manifold.js.map

},{"../index":103}],18:[function(require,module,exports){
var Box2D, b2ContactID, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2ContactID = Box2D.Collision.b2ContactID;

Box2D.Collision.b2ManifoldPoint = (function() {
  b2ManifoldPoint.prototype.m_localPoint = null;

  b2ManifoldPoint.prototype.m_id = null;

  b2ManifoldPoint.prototype.m_localPoint = null;

  b2ManifoldPoint.prototype.m_normalImpulse = 0.0;

  b2ManifoldPoint.prototype.m_tangentImpulse = 0.0;

  function b2ManifoldPoint() {
    this.m_localPoint = new b2Vec2();
    this.m_id = new b2ContactID();
    this.Reset();
    return;
  }

  b2ManifoldPoint.prototype.Reset = function() {
    this.m_localPoint.SetZero();
    this.m_normalImpulse = 0.0;
    this.m_tangentImpulse = 0.0;
    this.m_id.key = 0;
  };

  b2ManifoldPoint.prototype.Set = function(m) {
    this.m_localPoint.SetV(m.m_localPoint);
    this.m_normalImpulse = m.m_normalImpulse;
    this.m_tangentImpulse = m.m_tangentImpulse;
    this.m_id.Set(m.m_id);
  };

  return b2ManifoldPoint;

})();

//# sourceMappingURL=b2ManifoldPoint.js.map

},{"../index":103}],19:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2Point = (function() {
  b2Point.prototype.p = null;

  function b2Point() {
    this.p = new b2Vec2();
    return;
  }

  b2Point.prototype.Support = function(xf, vX, vY) {
    if (vX === void 0) {
      vX = 0;
    }
    if (vY === void 0) {
      vY = 0;
    }
    return this.p;
  };

  b2Point.prototype.GetFirstVertex = function(xf) {
    return this.p;
  };

  return b2Point;

})();

//# sourceMappingURL=b2Point.js.map

},{"../index":103}],20:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2RayCastInput = (function() {
  b2RayCastInput.prototype.p1 = null;

  b2RayCastInput.prototype.p2 = null;

  b2RayCastInput.prototype.maxFraction = 1;

  function b2RayCastInput(p1, p2, maxFraction) {
    this.p1 = new b2Vec2();
    this.p2 = new b2Vec2();
    if (p1 === void 0) {
      p1 = null;
    }
    if (p2 === void 0) {
      p2 = null;
    }
    if (maxFraction === void 0) {
      maxFraction = 1;
    }
    if (p1) {
      this.p1.SetV(p1);
    }
    if (p2) {
      this.p2.SetV(p2);
    }
    this.maxFraction = maxFraction;
    return;
  }

  return b2RayCastInput;

})();

//# sourceMappingURL=b2RayCastInput.js.map

},{"../index":103}],21:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2RayCastOutput = (function() {
  b2RayCastOutput.prototype.normal = null;

  function b2RayCastOutput() {
    this.normal = new b2Vec2();
    return;
  }

  return b2RayCastOutput;

})();

//# sourceMappingURL=b2RayCastOutput.js.map

},{"../index":103}],22:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2Segment = (function() {
  b2Segment.prototype.p1 = null;

  b2Segment.prototype.p2 = null;

  function b2Segment() {
    this.p1 = new b2Vec2();
    this.p2 = new b2Vec2();
    return;
  }

  b2Segment.prototype.TestSegment = function(lambda, normal, segment, maxLambda) {
    var a, bX, bY, dX, dY, denom, k_slop, mu2, nLen, nX, nY, rX, rY, s;
    if (maxLambda === void 0) {
      maxLambda = 0;
    }
    s = segment.p1;
    rX = segment.p2.x - s.x;
    rY = segment.p2.y - s.y;
    dX = this.p2.x - this.p1.x;
    dY = this.p2.y - this.p1.y;
    nX = dY;
    nY = -dX;
    k_slop = 100.0 * Number.MIN_VALUE;
    denom = -(rX * nX + rY * nY);
    if (denom > k_slop) {
      bX = s.x - this.p1.x;
      bY = s.y - this.p1.y;
      a = bX * nX + bY * nY;
      if (0.0 <= a && a <= maxLambda * denom) {
        mu2 = (-rX * bY) + rY * bX;
        if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
          a /= denom;
          nLen = Math.sqrt(nX * nX + nY * nY);
          nX /= nLen;
          nY /= nLen;
          lambda[0] = a;
          normal.Set(nX, nY);
          return true;
        }
      }
    }
    return false;
  };

  b2Segment.prototype.Extend = function(aabb) {
    this.ExtendForward(aabb);
    this.ExtendBackward(aabb);
  };

  b2Segment.prototype.ExtendForward = function(aabb) {
    var dX, dY, lambda;
    dX = this.p2.x - this.p1.x;
    dY = this.p2.y - this.p1.y;
    lambda = Math.min((dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : (dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY)), (dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : (dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY)));
    this.p2.x = this.p1.x + dX * lambda;
    this.p2.y = this.p1.y + dY * lambda;
  };

  b2Segment.prototype.ExtendBackward = function(aabb) {
    var dX, dY, lambda;
    dX = (-this.p2.x) + this.p1.x;
    dY = (-this.p2.y) + this.p1.y;
    lambda = Math.min((dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : (dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY)), (dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : (dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY)));
    this.p1.x = this.p2.x + dX * lambda;
    this.p1.y = this.p2.y + dY * lambda;
  };

  return b2Segment;

})();

//# sourceMappingURL=b2Segment.js.map

},{"../index":103}],23:[function(require,module,exports){
var Box2D, b2Math, b2Settings, b2Vec2;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.b2SeparationFunction = (function() {
  b2SeparationFunction.prototype.m_localPoint = null;

  b2SeparationFunction.prototype.m_axis = null;

  b2SeparationFunction.prototype.m_proxyA = null;

  b2SeparationFunction.prototype.m_proxyB = null;

  function b2SeparationFunction() {
    this.m_localPoint = new b2Vec2();
    this.m_axis = new b2Vec2();
    return;
  }

  b2SeparationFunction.prototype.Initialize = function(cache, proxyA, transformA, proxyB, transformB) {
    var a, b, c, count, dA, dB, denom, e, f, localPointA, localPointA1, localPointA2, localPointB, localPointB1, localPointB2, normalX, normalY, pA, pB, pointAX, pointAY, pointBX, pointBY, r, s, sgn, t, tMat, tVec;
    this.m_proxyA = proxyA;
    this.m_proxyB = proxyB;
    count = parseInt(cache.count);
    b2Settings.b2Assert(0 < count && count < 3);
    localPointA = void 0;
    localPointA1 = void 0;
    localPointA2 = void 0;
    localPointB = void 0;
    localPointB1 = void 0;
    localPointB2 = void 0;
    pointAX = 0;
    pointAY = 0;
    pointBX = 0;
    pointBY = 0;
    normalX = 0;
    normalY = 0;
    tMat = void 0;
    tVec = void 0;
    s = 0;
    sgn = 0;
    if (count === 1) {
      this.m_type = b2SeparationFunction.e_points;
      localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
      tVec = localPointA;
      tMat = transformA.R;
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointB;
      tMat = transformB.R;
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      this.m_axis.x = pointBX - pointAX;
      this.m_axis.y = pointBY - pointAY;
      this.m_axis.Normalize();
    } else if (cache.indexB[0] === cache.indexB[1]) {
      this.m_type = b2SeparationFunction.e_faceA;
      localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
      localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
      this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
      this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
      this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
      this.m_axis.Normalize();
      tVec = this.m_axis;
      tMat = transformA.R;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tVec = this.m_localPoint;
      tMat = transformA.R;
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointB;
      tMat = transformB.R;
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
      if (s < 0.0) {
        this.m_axis.NegativeSelf();
      }
    } else if (cache.indexA[0] === cache.indexA[0]) {
      this.m_type = b2SeparationFunction.e_faceB;
      localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
      localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
      localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
      this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
      this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
      this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
      this.m_axis.Normalize();
      tVec = this.m_axis;
      tMat = transformB.R;
      normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tVec = this.m_localPoint;
      tMat = transformB.R;
      pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = localPointA;
      tMat = transformA.R;
      pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
      if (s < 0.0) {
        this.m_axis.NegativeSelf();
      }
    } else {
      localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
      localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
      localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
      localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
      pA = b2Math.MulX(transformA, localPointA);
      dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1));
      pB = b2Math.MulX(transformB, localPointB);
      dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1));
      a = dA.x * dA.x + dA.y * dA.y;
      e = dB.x * dB.x + dB.y * dB.y;
      r = b2Math.SubtractVV(dB, dA);
      c = dA.x * r.x + dA.y * r.y;
      f = dB.x * r.x + dB.y * r.y;
      b = dA.x * dB.x + dA.y * dB.y;
      denom = a * e - b * b;
      s = 0.0;
      if (denom !== 0.0) {
        s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = b2Math.Clamp((b - c) / a, 0.0, 1.0);
      }
      localPointA = new b2Vec2();
      localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
      localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
      localPointB = new b2Vec2();
      localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
      localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
      if (s === 0.0 || s === 1.0) {
        this.m_type = b2SeparationFunction.e_faceB;
        this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
        this.m_axis.Normalize();
        this.m_localPoint = localPointB;
        tVec = this.m_axis;
        tMat = transformB.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointA;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
        if (s < 0.0) {
          this.m_axis.NegativeSelf();
        }
      } else {
        this.m_type = b2SeparationFunction.e_faceA;
        this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
        this.m_localPoint = localPointA;
        tVec = this.m_axis;
        tMat = transformA.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformA.R;
        pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = localPointB;
        tMat = transformB.R;
        pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
        if (s < 0.0) {
          this.m_axis.NegativeSelf();
        }
      }
    }
  };

  b2SeparationFunction.prototype.Evaluate = function(transformA, transformB) {
    var axisA, axisB, localPointA, localPointB, normal, pointA, pointB, seperation;
    axisA = void 0;
    axisB = void 0;
    localPointA = void 0;
    localPointB = void 0;
    pointA = void 0;
    pointB = void 0;
    seperation = 0;
    normal = void 0;
    switch (this.m_type) {
      case b2SeparationFunction.e_points:
        axisA = b2Math.MulTMV(transformA.R, this.m_axis);
        axisB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative());
        localPointA = this.m_proxyA.GetSupportVertex(axisA);
        localPointB = this.m_proxyB.GetSupportVertex(axisB);
        pointA = b2Math.MulX(transformA, localPointA);
        pointB = b2Math.MulX(transformB, localPointB);
        seperation = (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
        return seperation;
      case b2SeparationFunction.e_faceA:
        normal = b2Math.MulMV(transformA.R, this.m_axis);
        pointA = b2Math.MulX(transformA, this.m_localPoint);
        axisB = b2Math.MulTMV(transformB.R, normal.GetNegative());
        localPointB = this.m_proxyB.GetSupportVertex(axisB);
        pointB = b2Math.MulX(transformB, localPointB);
        seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
        return seperation;
      case b2SeparationFunction.e_faceB:
        normal = b2Math.MulMV(transformB.R, this.m_axis);
        pointB = b2Math.MulX(transformB, this.m_localPoint);
        axisA = b2Math.MulTMV(transformA.R, normal.GetNegative());
        localPointA = this.m_proxyA.GetSupportVertex(axisA);
        pointA = b2Math.MulX(transformA, localPointA);
        seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
        return seperation;
      default:
        b2Settings.b2Assert(false);
        return 0.0;
    }
  };

  b2SeparationFunction.e_points = 0x01;

  b2SeparationFunction.e_faceA = 0x02;

  b2SeparationFunction.e_faceB = 0x04;

  return b2SeparationFunction;

})();

//# sourceMappingURL=b2SeparationFunction.js.map

},{"../index":103}],24:[function(require,module,exports){
var Box2D, b2Math, b2Settings, b2SimplexVertex, b2Vec2;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2SimplexVertex = Box2D.Collision.b2SimplexVertex;

Box2D.Collision.b2Simplex = (function() {
  b2Simplex.prototype.m_v1 = null;

  b2Simplex.prototype.m_v2 = null;

  b2Simplex.prototype.m_v3 = null;

  b2Simplex.prototype.m_vertices = null;

  b2Simplex.prototype.m_count = 0;

  function b2Simplex() {
    this.m_v1 = new b2SimplexVertex();
    this.m_v2 = new b2SimplexVertex();
    this.m_v3 = new b2SimplexVertex();
    this.m_vertices = new Array(3);
    this.m_vertices[0] = this.m_v1;
    this.m_vertices[1] = this.m_v2;
    this.m_vertices[2] = this.m_v3;
    return;
  }

  b2Simplex.prototype.ReadCache = function(cache, proxyA, transformA, proxyB, transformB) {
    var i, metric1, metric2, v, vertices, wALocal, wBLocal;
    b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
    wALocal = void 0;
    wBLocal = void 0;
    this.m_count = cache.count;
    vertices = this.m_vertices;
    i = 0;
    while (i < this.m_count) {
      v = vertices[i];
      v.indexA = cache.indexA[i];
      v.indexB = cache.indexB[i];
      wALocal = proxyA.GetVertex(v.indexA);
      wBLocal = proxyB.GetVertex(v.indexB);
      v.wA = b2Math.MulX(transformA, wALocal);
      v.wB = b2Math.MulX(transformB, wBLocal);
      v.w = b2Math.SubtractVV(v.wB, v.wA);
      v.a = 0;
      i++;
    }
    if (this.m_count > 1) {
      metric1 = cache.metric;
      metric2 = this.GetMetric();
      if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
        this.m_count = 0;
      }
    }
    if (this.m_count === 0) {
      v = vertices[0];
      v.indexA = 0;
      v.indexB = 0;
      wALocal = proxyA.GetVertex(0);
      wBLocal = proxyB.GetVertex(0);
      v.wA = b2Math.MulX(transformA, wALocal);
      v.wB = b2Math.MulX(transformB, wBLocal);
      v.w = b2Math.SubtractVV(v.wB, v.wA);
      this.m_count = 1;
    }
  };

  b2Simplex.prototype.WriteCache = function(cache) {
    var i, vertices;
    cache.metric = this.GetMetric();
    cache.count = Box2D.parseUInt(this.m_count);
    vertices = this.m_vertices;
    i = 0;
    while (i < this.m_count) {
      cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA);
      cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB);
      i++;
    }
  };

  b2Simplex.prototype.GetSearchDirection = function() {
    var e12, sgn;
    switch (this.m_count) {
      case 1:
        return this.m_v1.w.GetNegative();
      case 2:
        e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
        sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
        if (sgn > 0.0) {
          return b2Math.CrossFV(1.0, e12);
        } else {
          return b2Math.CrossVF(e12, 1.0);
        }
        break;
      default:
        b2Settings.b2Assert(false);
        return new b2Vec2();
    }
  };

  b2Simplex.prototype.GetClosestPoint = function() {
    switch (this.m_count) {
      case 0:
        b2Settings.b2Assert(false);
        return new b2Vec2();
      case 1:
        return this.m_v1.w;
      case 2:
        return new b2Vec2(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
      default:
        b2Settings.b2Assert(false);
        return new b2Vec2();
    }
  };

  b2Simplex.prototype.GetWitnessPoints = function(pA, pB) {
    switch (this.m_count) {
      case 0:
        return b2Settings.b2Assert(false);
      case 1:
        pA.SetV(this.m_v1.wA);
        return pB.SetV(this.m_v1.wB);
      case 2:
        pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
        pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
        pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
        return pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
      case 3:
        pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
        return pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
      default:
        return b2Settings.b2Assert(false);
    }
  };

  b2Simplex.prototype.GetMetric = function() {
    switch (this.m_count) {
      case 0:
        b2Settings.b2Assert(false);
        return 0.0;
      case 1:
        return 0.0;
      case 2:
        return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Length();
      case 3:
        return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w), b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));
      default:
        b2Settings.b2Assert(false);
        return 0.0;
    }
  };

  b2Simplex.prototype.Solve2 = function() {
    var d12_1, d12_2, e12, inv_d12, w1, w2;
    w1 = this.m_v1.w;
    w2 = this.m_v2.w;
    e12 = b2Math.SubtractVV(w2, w1);
    d12_2 = -(w1.x * e12.x + w1.y * e12.y);
    if (d12_2 <= 0.0) {
      this.m_v1.a = 1.0;
      this.m_count = 1;
      return;
    }
    d12_1 = w2.x * e12.x + w2.y * e12.y;
    if (d12_1 <= 0.0) {
      this.m_v2.a = 1.0;
      this.m_count = 1;
      this.m_v1.Set(this.m_v2);
      return;
    }
    inv_d12 = 1.0 / (d12_1 + d12_2);
    this.m_v1.a = d12_1 * inv_d12;
    this.m_v2.a = d12_2 * inv_d12;
    this.m_count = 2;
  };

  b2Simplex.prototype.Solve3 = function() {
    var d123_1, d123_2, d123_3, d12_1, d12_2, d13_1, d13_2, d23_1, d23_2, e12, e13, e23, inv_d12, inv_d123, inv_d13, inv_d23, n123, w1, w1e12, w1e13, w2, w2e12, w2e23, w3, w3e13, w3e23;
    w1 = this.m_v1.w;
    w2 = this.m_v2.w;
    w3 = this.m_v3.w;
    e12 = b2Math.SubtractVV(w2, w1);
    w1e12 = b2Math.Dot(w1, e12);
    w2e12 = b2Math.Dot(w2, e12);
    d12_1 = w2e12;
    d12_2 = -w1e12;
    e13 = b2Math.SubtractVV(w3, w1);
    w1e13 = b2Math.Dot(w1, e13);
    w3e13 = b2Math.Dot(w3, e13);
    d13_1 = w3e13;
    d13_2 = -w1e13;
    e23 = b2Math.SubtractVV(w3, w2);
    w2e23 = b2Math.Dot(w2, e23);
    w3e23 = b2Math.Dot(w3, e23);
    d23_1 = w3e23;
    d23_2 = -w2e23;
    n123 = b2Math.CrossVV(e12, e13);
    d123_1 = n123 * b2Math.CrossVV(w2, w3);
    d123_2 = n123 * b2Math.CrossVV(w3, w1);
    d123_3 = n123 * b2Math.CrossVV(w1, w2);
    if (d12_2 <= 0.0 && d13_2 <= 0.0) {
      this.m_v1.a = 1.0;
      this.m_count = 1;
      return;
    }
    if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
      inv_d12 = 1.0 / (d12_1 + d12_2);
      this.m_v1.a = d12_1 * inv_d12;
      this.m_v2.a = d12_2 * inv_d12;
      this.m_count = 2;
      return;
    }
    if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
      inv_d13 = 1.0 / (d13_1 + d13_2);
      this.m_v1.a = d13_1 * inv_d13;
      this.m_v3.a = d13_2 * inv_d13;
      this.m_count = 2;
      this.m_v2.Set(this.m_v3);
      return;
    }
    if (d12_1 <= 0.0 && d23_2 <= 0.0) {
      this.m_v2.a = 1.0;
      this.m_count = 1;
      this.m_v1.Set(this.m_v2);
      return;
    }
    if (d13_1 <= 0.0 && d23_1 <= 0.0) {
      this.m_v3.a = 1.0;
      this.m_count = 1;
      this.m_v1.Set(this.m_v3);
      return;
    }
    if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
      inv_d23 = 1.0 / (d23_1 + d23_2);
      this.m_v2.a = d23_1 * inv_d23;
      this.m_v3.a = d23_2 * inv_d23;
      this.m_count = 2;
      this.m_v1.Set(this.m_v3);
      return;
    }
    inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
    this.m_v1.a = d123_1 * inv_d123;
    this.m_v2.a = d123_2 * inv_d123;
    this.m_v3.a = d123_3 * inv_d123;
    this.m_count = 3;
  };

  return b2Simplex;

})();

//# sourceMappingURL=b2Simplex.js.map

},{"../index":103}],25:[function(require,module,exports){
var Box2D, Vector;

Box2D = require('../index');

Vector = Box2D.Vector;

Box2D.Collision.b2SimplexCache = (function() {
  b2SimplexCache.prototype.indexA = null;

  b2SimplexCache.prototype.indexB = null;

  function b2SimplexCache() {
    this.indexA = new Vector(3);
    this.indexB = new Vector(3);
    return;
  }

  return b2SimplexCache;

})();

//# sourceMappingURL=b2SimplexCache.js.map

},{"../index":103}],26:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Collision.b2SimplexVertex = (function() {
  function b2SimplexVertex() {}

  b2SimplexVertex.prototype.Set = function(other) {
    this.wA.SetV(other.wA);
    this.wB.SetV(other.wB);
    this.w.SetV(other.w);
    this.a = other.a;
    this.indexA = other.indexA;
    this.indexB = other.indexB;
  };

  return b2SimplexVertex;

})();

//# sourceMappingURL=b2SimplexVertex.js.map

},{"../index":103}],27:[function(require,module,exports){
var Box2D, b2DistanceProxy, b2Sweep;

Box2D = require('../index');

b2Sweep = Box2D.Common.Math.b2Sweep;

b2DistanceProxy = Box2D.Collision.b2DistanceProxy;

Box2D.Collision.b2TOIInput = (function() {
  b2TOIInput.prototype.proxyA = null;

  b2TOIInput.prototype.proxyB = null;

  b2TOIInput.prototype.sweepA = null;

  b2TOIInput.prototype.sweepB = null;

  function b2TOIInput() {
    this.proxyA = new b2DistanceProxy();
    this.proxyB = new b2DistanceProxy();
    this.sweepA = new b2Sweep();
    this.sweepB = new b2Sweep();
    return;
  }

  return b2TOIInput;

})();

//# sourceMappingURL=b2TOIInput.js.map

},{"../index":103}],28:[function(require,module,exports){
var Box2D, b2Distance, b2DistanceInput, b2DistanceOutput, b2Math, b2SeparationFunction, b2Settings, b2SimplexCache, b2Transform;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Transform = Box2D.Common.Math.b2Transform;

b2Distance = Box2D.Collision.b2Distance;

b2SimplexCache = Box2D.Collision.b2SimplexCache;

b2SeparationFunction = Box2D.Collision.b2SeparationFunction;

b2DistanceOutput = Box2D.Collision.b2DistanceOutput;

b2DistanceInput = Box2D.Collision.b2DistanceInput;

Box2D.Collision.b2TimeOfImpact = (function() {
  function b2TimeOfImpact() {}

  b2TimeOfImpact.b2_toiCalls = 0;

  b2TimeOfImpact.b2_toiIters = 0;

  b2TimeOfImpact.b2_toiMaxIters = 0;

  b2TimeOfImpact.b2_toiRootIters = 0;

  b2TimeOfImpact.b2_toiMaxRootIters = 0;

  b2TimeOfImpact.s_cache = new b2SimplexCache();

  b2TimeOfImpact.s_distanceInput = new b2DistanceInput();

  b2TimeOfImpact.s_xfA = new b2Transform();

  b2TimeOfImpact.s_xfB = new b2Transform();

  b2TimeOfImpact.s_fcn = new b2SeparationFunction();

  b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();

  b2TimeOfImpact.TimeOfImpact = function(input) {
    var alpha, f, f1, f2, iter, k_maxIterations, newAlpha, proxyA, proxyB, radius, rootIterCount, separation, sweepA, sweepB, target, tolerance, x, x1, x2;
    ++b2TimeOfImpact.b2_toiCalls;
    proxyA = input.proxyA;
    proxyB = input.proxyB;
    sweepA = input.sweepA;
    sweepB = input.sweepB;
    b2Settings.b2Assert(sweepA.t0 === sweepB.t0);
    b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
    radius = proxyA.m_radius + proxyB.m_radius;
    tolerance = input.tolerance;
    alpha = 0.0;
    k_maxIterations = 1000;
    iter = 0;
    target = 0.0;
    b2TimeOfImpact.s_cache.count = 0;
    b2TimeOfImpact.s_distanceInput.useRadii = false;
    while (true) {
      sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
      sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);
      b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
      b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
      b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
      b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;
      b2Distance.Distance(b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput);
      if (b2TimeOfImpact.s_distanceOutput.distance <= 0.0) {
        alpha = 1.0;
        break;
      }
      b2TimeOfImpact.s_fcn.Initialize(b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB);
      separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
      if (separation <= 0.0) {
        alpha = 1.0;
        break;
      }
      if (iter === 0) {
        if (separation > radius) {
          target = b2Math.Max(radius - tolerance, 0.75 * radius);
        } else {
          target = b2Math.Max(separation - tolerance, 0.02 * radius);
        }
      }
      if (separation - target < 0.5 * tolerance) {
        if (iter === 0) {
          alpha = 1.0;
          break;
        }
        break;
      }
      newAlpha = alpha;
      x1 = alpha;
      x2 = 1.0;
      f1 = separation;
      sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
      sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
      f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
      if (f2 >= target) {
        alpha = 1.0;
        break;
      }
      rootIterCount = 0;
      while (true) {
        x = 0;
        if (rootIterCount & 1) {
          x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
        } else {
          x = 0.5 * (x1 + x2);
        }
        sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
        sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
        f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
        if (b2Math.Abs(f - target) < 0.025 * tolerance) {
          newAlpha = x;
          break;
        }
        if (f > target) {
          x1 = x;
          f1 = f;
        } else {
          x2 = x;
          f2 = f;
        }
        ++rootIterCount;
        ++b2TimeOfImpact.b2_toiRootIters;
        if (rootIterCount === 50) {
          break;
        }
      }
      b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);
      if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha) {
        break;
      }
      alpha = newAlpha;
      iter++;
      ++b2TimeOfImpact.b2_toiIters;
      if (iter === k_maxIterations) {
        break;
      }
    }
    b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter);
    return alpha;
  };

  return b2TimeOfImpact;

})();

//# sourceMappingURL=b2TimeOfImpact.js.map

},{"../index":103}],29:[function(require,module,exports){
var Box2D, b2Manifold, b2Settings, b2Vec2;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Manifold = Box2D.Collision.b2Manifold;

Box2D.Collision.b2WorldManifold = (function() {
  b2WorldManifold.prototype.m_normal = null;

  b2WorldManifold.prototype.m_points = null;

  function b2WorldManifold() {
    var i;
    this.m_normal = new b2Vec2();
    this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      this.m_points[i] = new b2Vec2();
      i++;
    }
    return;
  }

  b2WorldManifold.prototype.Initialize = function(manifold, xfA, radiusA, xfB, radiusB) {
    var cAX, cAY, cBX, cBY, clipPointX, clipPointY, d, d2, dX, dY, i, normalX, normalY, planePointX, planePointY, pointAX, pointAY, pointBX, pointBY, tMat, tVec, _results, _results1;
    if (radiusA === void 0) {
      radiusA = 0;
    }
    if (radiusB === void 0) {
      radiusB = 0;
    }
    if (manifold.m_pointCount === 0) {
      return;
    }
    i = 0;
    tVec = void 0;
    tMat = void 0;
    normalX = 0;
    normalY = 0;
    planePointX = 0;
    planePointY = 0;
    clipPointX = 0;
    clipPointY = 0;
    switch (manifold.m_type) {
      case b2Manifold.e_circles:
        tMat = xfA.R;
        tVec = manifold.m_localPoint;
        pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = xfB.R;
        tVec = manifold.m_points[0].m_localPoint;
        pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        dX = pointBX - pointAX;
        dY = pointBY - pointAY;
        d2 = dX * dX + dY * dY;
        if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
          d = Math.sqrt(d2);
          this.m_normal.x = dX / d;
          this.m_normal.y = dY / d;
        } else {
          this.m_normal.x = 1;
          this.m_normal.y = 0;
        }
        cAX = pointAX + radiusA * this.m_normal.x;
        cAY = pointAY + radiusA * this.m_normal.y;
        cBX = pointBX - radiusB * this.m_normal.x;
        cBY = pointBY - radiusB * this.m_normal.y;
        this.m_points[0].x = 0.5 * (cAX + cBX);
        return this.m_points[0].y = 0.5 * (cAY + cBY);
      case b2Manifold.e_faceA:
        tMat = xfA.R;
        tVec = manifold.m_localPlaneNormal;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = xfA.R;
        tVec = manifold.m_localPoint;
        planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        this.m_normal.x = normalX;
        this.m_normal.y = normalY;
        i = 0;
        _results = [];
        while (i < manifold.m_pointCount) {
          tMat = xfB.R;
          tVec = manifold.m_points[i].m_localPoint;
          clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          this.m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
          this.m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;
          _results.push(i++);
        }
        return _results;
        break;
      case b2Manifold.e_faceB:
        tMat = xfB.R;
        tVec = manifold.m_localPlaneNormal;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = xfB.R;
        tVec = manifold.m_localPoint;
        planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        this.m_normal.x = -normalX;
        this.m_normal.y = -normalY;
        i = 0;
        _results1 = [];
        while (i < manifold.m_pointCount) {
          tMat = xfA.R;
          tVec = manifold.m_points[i].m_localPoint;
          clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          this.m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
          this.m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;
          _results1.push(i++);
        }
        return _results1;
    }
  };

  return b2WorldManifold;

})();

//# sourceMappingURL=b2WorldManifold.js.map

},{"../index":103}],30:[function(require,module,exports){
var Box2D, b2Math, b2Settings, b2Shape, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Shape = Box2D.Collision.Shapes.b2Shape;

Box2D.Collision.Shapes.b2CircleShape = (function(_super) {
  __extends(b2CircleShape, _super);

  b2CircleShape.prototype.m_type = b2Shape.e_circleShape;

  b2CircleShape.prototype.m_p = null;

  function b2CircleShape(radius) {
    if (radius === void 0) {
      radius = 0;
    }
    b2CircleShape.__super__.constructor.call(this, radius);
    this.m_p = new b2Vec2();
    this.m_type = b2Shape.e_circleShape;
    this.m_radius = radius;
    return;
  }

  b2CircleShape.prototype.Copy = function() {
    var s;
    s = new b2CircleShape();
    s.Set(this);
    return s;
  };

  b2CircleShape.prototype.Set = function(other) {
    var other2;
    b2CircleShape.__super__.Set.call(this, other);
    if (Box2D.equals(other, b2CircleShape)) {
      other2 = (other instanceof b2CircleShape ? other : null);
      this.m_p.SetV(other2.m_p);
    }
  };

  b2CircleShape.prototype.TestPoint = function(transform, p) {
    var dX, dY, tMat;
    tMat = transform.R;
    dX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
    dY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
    dX = p.x - dX;
    dY = p.y - dY;
    return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
  };

  b2CircleShape.prototype.RayCast = function(output, input, transform) {
    var a, b, c, positionX, positionY, rX, rY, rr, sX, sY, sigma, tMat;
    tMat = transform.R;
    positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
    positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
    sX = input.p1.x - positionX;
    sY = input.p1.y - positionY;
    b = (sX * sX + sY * sY) - this.m_radius * this.m_radius;
    rX = input.p2.x - input.p1.x;
    rY = input.p2.y - input.p1.y;
    c = sX * rX + sY * rY;
    rr = rX * rX + rY * rY;
    sigma = c * c - rr * b;
    if (sigma < 0.0 || rr < Number.MIN_VALUE) {
      return false;
    }
    a = -(c + Math.sqrt(sigma));
    if (0.0 <= a && a <= input.maxFraction * rr) {
      a /= rr;
      output.fraction = a;
      output.normal.x = sX + a * rX;
      output.normal.y = sY + a * rY;
      output.normal.Normalize();
      return true;
    }
    return false;
  };

  b2CircleShape.prototype.ComputeAABB = function(aabb, transform) {
    var pX, pY, tMat;
    tMat = transform.R;
    pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
    pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
    aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
    aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
  };

  b2CircleShape.prototype.ComputeMass = function(massData, density) {
    if (density === void 0) {
      density = 0;
    }
    massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
    massData.center.SetV(this.m_p);
    massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
  };

  b2CircleShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
    var area, com, l, l2, p, r2;
    if (offset === void 0) {
      offset = 0;
    }
    p = b2Math.MulX(xf, this.m_p);
    l = -(b2Math.Dot(normal, p) - offset);
    if (l < (-this.m_radius) + Number.MIN_VALUE) {
      return 0;
    }
    if (l > this.m_radius) {
      c.SetV(p);
      return Math.PI * this.m_radius * this.m_radius;
    }
    r2 = this.m_radius * this.m_radius;
    l2 = l * l;
    area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
    com = -2 / 3 * Math.pow(r2 - l2, 1.5) / area;
    c.x = p.x + normal.x * com;
    c.y = p.y + normal.y * com;
    return area;
  };

  b2CircleShape.prototype.GetLocalPosition = function() {
    return this.m_p;
  };

  b2CircleShape.prototype.SetLocalPosition = function(position) {
    this.m_p.SetV(position);
  };

  b2CircleShape.prototype.GetRadius = function() {
    return this.m_radius;
  };

  b2CircleShape.prototype.SetRadius = function(radius) {
    if (radius === void 0) {
      radius = 0;
    }
    this.m_radius = radius;
  };

  return b2CircleShape;

})(b2Shape);

//# sourceMappingURL=b2CircleShape.js.map

},{"../../index":103}],31:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Collision.Shapes.b2EdgeChainDef = (function() {
  b2EdgeChainDef.prototype.vertexCount = 0;

  b2EdgeChainDef.prototype.isALoop = true;

  b2EdgeChainDef.prototype.vertices = null;

  function b2EdgeChainDef() {
    this.vertices = [];
    return;
  }

  return b2EdgeChainDef;

})();

//# sourceMappingURL=b2EdgeChainDef.js.map

},{"../../index":103}],32:[function(require,module,exports){
var Box2D, b2Shape,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Shape = Box2D.Collision.Shapes.b2Shape;

Box2D.Collision.Shapes.b2EdgeShape = (function(_super) {
  __extends(b2EdgeShape, _super);

  function b2EdgeShape() {
    b2EdgeShape.__super__.constructor.apply(this, arguments);
    this.s_supportVec = new b2Vec2();
    this.m_v1 = new b2Vec2();
    this.m_v2 = new b2Vec2();
    this.m_coreV1 = new b2Vec2();
    this.m_coreV2 = new b2Vec2();
    this.m_normal = new b2Vec2();
    this.m_direction = new b2Vec2();
    this.m_cornerDir1 = new b2Vec2();
    this.m_cornerDir2 = new b2Vec2();
    this.m_type = b2Shape.e_edgeShape;
    this.m_prevEdge = null;
    this.m_nextEdge = null;
    this.m_v1 = v1;
    this.m_v2 = v2;
    this.m_direction.Set(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
    this.m_length = this.m_direction.Normalize();
    this.m_normal.Set(this.m_direction.y, -this.m_direction.x);
    this.m_coreV1.Set((-b2Settings.b2_toiSlop * (this.m_normal.x - this.m_direction.x)) + this.m_v1.x, (-b2Settings.b2_toiSlop * (this.m_normal.y - this.m_direction.y)) + this.m_v1.y);
    this.m_coreV2.Set((-b2Settings.b2_toiSlop * (this.m_normal.x + this.m_direction.x)) + this.m_v2.x, (-b2Settings.b2_toiSlop * (this.m_normal.y + this.m_direction.y)) + this.m_v2.y);
    this.m_cornerDir1 = this.m_normal;
    this.m_cornerDir2.Set(-this.m_normal.x, -this.m_normal.y);
    return;
  }

  b2EdgeShape.prototype.TestPoint = function(transform, p) {
    return false;
  };

  b2EdgeShape.prototype.RayCast = function(output, input, transform) {
    var a, bX, bY, denom, k_slop, mu2, nLen, nX, nY, rX, rY, tMat, v1X, v1Y;
    tMat = void 0;
    rX = input.p2.x - input.p1.x;
    rY = input.p2.y - input.p1.y;
    tMat = transform.R;
    v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
    v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
    nX = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) - v1Y;
    nY = -(transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) - v1X);
    k_slop = 100.0 * Number.MIN_VALUE;
    denom = -(rX * nX + rY * nY);
    if (denom > k_slop) {
      bX = input.p1.x - v1X;
      bY = input.p1.y - v1Y;
      a = bX * nX + bY * nY;
      if (0.0 <= a && a <= input.maxFraction * denom) {
        mu2 = (-rX * bY) + rY * bX;
        if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
          a /= denom;
          output.fraction = a;
          nLen = Math.sqrt(nX * nX + nY * nY);
          output.normal.x = nX / nLen;
          output.normal.y = nY / nLen;
          return true;
        }
      }
    }
    return false;
  };

  b2EdgeShape.prototype.ComputeAABB = function(aabb, transform) {
    var tMat, v1X, v1Y, v2X, v2Y;
    tMat = transform.R;
    v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
    v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
    v2X = transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y);
    v2Y = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
    if (v1X < v2X) {
      aabb.lowerBound.x = v1X;
      aabb.upperBound.x = v2X;
    } else {
      aabb.lowerBound.x = v2X;
      aabb.upperBound.x = v1X;
    }
    if (v1Y < v2Y) {
      aabb.lowerBound.y = v1Y;
      aabb.upperBound.y = v2Y;
    } else {
      aabb.lowerBound.y = v2Y;
      aabb.upperBound.y = v1Y;
    }
  };

  b2EdgeShape.prototype.ComputeMass = function(massData, density) {
    if (density === void 0) {
      density = 0;
    }
    massData.mass = 0;
    massData.center.SetV(this.m_v1);
    massData.I = 0;
  };

  b2EdgeShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
    var d1, d2, v0, v1, v2;
    if (offset === void 0) {
      offset = 0;
    }
    v0 = new b2Vec2(normal.x * offset, normal.y * offset);
    v1 = b2Math.MulX(xf, this.m_v1);
    v2 = b2Math.MulX(xf, this.m_v2);
    d1 = b2Math.Dot(normal, v1) - offset;
    d2 = b2Math.Dot(normal, v2) - offset;
    if (d1 > 0) {
      if (d2 > 0) {
        return 0;
      } else {
        v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
        v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
      }
    } else {
      if (d2 > 0) {
        v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
        v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
      } else {

      }
    }
    c.x = (v0.x + v1.x + v2.x) / 3;
    c.y = (v0.y + v1.y + v2.y) / 3;
    return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
  };

  b2EdgeShape.prototype.GetLength = function() {
    return this.m_length;
  };

  b2EdgeShape.prototype.GetVertex1 = function() {
    return this.m_v1;
  };

  b2EdgeShape.prototype.GetVertex2 = function() {
    return this.m_v2;
  };

  b2EdgeShape.prototype.GetCoreVertex1 = function() {
    return this.m_coreV1;
  };

  b2EdgeShape.prototype.GetCoreVertex2 = function() {
    return this.m_coreV2;
  };

  b2EdgeShape.prototype.GetNormalVector = function() {
    return this.m_normal;
  };

  b2EdgeShape.prototype.GetDirectionVector = function() {
    return this.m_direction;
  };

  b2EdgeShape.prototype.GetCorner1Vector = function() {
    return this.m_cornerDir1;
  };

  b2EdgeShape.prototype.GetCorner2Vector = function() {
    return this.m_cornerDir2;
  };

  b2EdgeShape.prototype.Corner1IsConvex = function() {
    return this.m_cornerConvex1;
  };

  b2EdgeShape.prototype.Corner2IsConvex = function() {
    return this.m_cornerConvex2;
  };

  b2EdgeShape.prototype.GetFirstVertex = function(xf) {
    var tMat;
    tMat = xf.R;
    return new b2Vec2(xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y), xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y));
  };

  b2EdgeShape.prototype.GetNextEdge = function() {
    return this.m_nextEdge;
  };

  b2EdgeShape.prototype.GetPrevEdge = function() {
    return this.m_prevEdge;
  };

  b2EdgeShape.prototype.Support = function(xf, dX, dY) {
    var tMat, v1X, v1Y, v2X, v2Y;
    if (dX === void 0) {
      dX = 0;
    }
    if (dY === void 0) {
      dY = 0;
    }
    tMat = xf.R;
    v1X = xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y);
    v1Y = xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y);
    v2X = xf.position.x + (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y);
    v2Y = xf.position.y + (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);
    if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
      this.s_supportVec.x = v1X;
      this.s_supportVec.y = v1Y;
    } else {
      this.s_supportVec.x = v2X;
      this.s_supportVec.y = v2Y;
    }
    return this.s_supportVec;
  };

  b2EdgeShape.prototype.SetPrevEdge = function(edge, core, cornerDir, convex) {
    this.m_prevEdge = edge;
    this.m_coreV1 = core;
    this.m_cornerDir1 = cornerDir;
    this.m_cornerConvex1 = convex;
  };

  b2EdgeShape.prototype.SetNextEdge = function(edge, core, cornerDir, convex) {
    this.m_nextEdge = edge;
    this.m_coreV2 = core;
    this.m_cornerDir2 = cornerDir;
    this.m_cornerConvex2 = convex;
  };

  return b2EdgeShape;

})(b2Shape);

//# sourceMappingURL=b2EdgeShape.js.map

},{"../../index":103}],33:[function(require,module,exports){
var Box2D, b2Shape, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Shape = Box2D.Collision.Shapes.b2Shape;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Collision.Shapes.b2MassData = (function(_super) {
  __extends(b2MassData, _super);

  b2MassData.prototype.mass = 0.0;

  b2MassData.prototype.center = null;

  b2MassData.prototype.I = 0.0;

  function b2MassData() {
    this.center = new b2Vec2(0, 0);
    return;
  }

  return b2MassData;

})(b2Shape);

//# sourceMappingURL=b2MassData.js.map

},{"../../index":103}],34:[function(require,module,exports){
var Box2D, Vector, b2Mat22, b2Math, b2Settings, b2Shape, b2Transform, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

Vector = Box2D.Vector;

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Transform = Box2D.Common.Math.b2Transform;

b2Shape = Box2D.Collision.Shapes.b2Shape;

Box2D.Collision.Shapes.b2PolygonShape = (function(_super) {
  __extends(b2PolygonShape, _super);

  b2PolygonShape.s_mat = new b2Mat22();

  b2PolygonShape.prototype.m_type = b2Shape.e_polygonShape;

  b2PolygonShape.prototype.m_centroid = null;

  b2PolygonShape.prototype.m_vertices = null;

  b2PolygonShape.prototype.m_normals = null;

  b2PolygonShape.prototype.m_vertexCount = 0;

  function b2PolygonShape() {
    this.m_centroid = new b2Vec2();
    this.m_vertices = new Array();
    this.m_normals = new Array();
    return;
  }

  b2PolygonShape.prototype.Copy = function() {
    var s;
    s = new b2PolygonShape();
    s.Set(this);
    return s;
  };

  b2PolygonShape.prototype.Set = function(other) {
    var i, other2;
    b2PolygonShape.__super__.Set.call(this, other);
    if (Box2D.equals(other, b2PolygonShape)) {
      other2 = (other instanceof b2PolygonShape ? other : null);
      this.m_centroid.SetV(other2.m_centroid);
      this.m_vertexCount = other2.m_vertexCount;
      this.Reserve(this.m_vertexCount);
      i = 0;
      while (i < this.m_vertexCount) {
        this.m_vertices[i].SetV(other2.m_vertices[i]);
        this.m_normals[i].SetV(other2.m_normals[i]);
        i++;
      }
    }
  };

  b2PolygonShape.prototype.SetAsArray = function(vertices, vertexCount) {
    var i, tVec, v;
    if (vertexCount === void 0) {
      vertexCount = 0;
    }
    v = new Array();
    i = 0;
    tVec = void 0;
    i = 0;
    while (i < vertices.length) {
      tVec = vertices[i];
      v.push(tVec);
      ++i;
    }
    this.SetAsVector(v, vertexCount);
  };

  b2PolygonShape.AsArray = function(vertices, vertexCount) {
    var polygonShape;
    if (vertexCount === void 0) {
      vertexCount = 0;
    }
    polygonShape = new b2PolygonShape();
    polygonShape.SetAsArray(vertices, vertexCount);
    return polygonShape;
  };

  b2PolygonShape.prototype.SetAsVector = function(vertices, vertexCount) {
    var edge, i, i1, i2;
    if (vertexCount === void 0) {
      vertexCount = 0;
    }
    if (vertexCount === 0) {
      vertexCount = vertices.length;
    }
    b2Settings.b2Assert(2 <= vertexCount);
    this.m_vertexCount = vertexCount;
    this.Reserve(vertexCount);
    i = 0;
    i = 0;
    while (i < this.m_vertexCount) {
      this.m_vertices[i].SetV(vertices[i]);
      i++;
    }
    i = 0;
    while (i < this.m_vertexCount) {
      i1 = parseInt(i);
      i2 = parseInt((i + 1 < this.m_vertexCount ? i + 1 : 0));
      edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
      b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
      this.m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
      this.m_normals[i].Normalize();
      ++i;
    }
    this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
  };

  b2PolygonShape.AsVector = function(vertices, vertexCount) {
    var polygonShape;
    if (vertexCount === void 0) {
      vertexCount = 0;
    }
    polygonShape = new b2PolygonShape();
    polygonShape.SetAsVector(vertices, vertexCount);
    return polygonShape;
  };

  b2PolygonShape.prototype.SetAsBox = function(hx, hy) {
    if (hx === void 0) {
      hx = 0;
    }
    if (hy === void 0) {
      hy = 0;
    }
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].Set(-hx, -hy);
    this.m_vertices[1].Set(hx, -hy);
    this.m_vertices[2].Set(hx, hy);
    this.m_vertices[3].Set(-hx, hy);
    this.m_normals[0].Set(0.0, -1.0);
    this.m_normals[1].Set(1.0, 0.0);
    this.m_normals[2].Set(0.0, 1.0);
    this.m_normals[3].Set(-1.0, 0.0);
    this.m_centroid.SetZero();
  };

  b2PolygonShape.AsBox = function(hx, hy) {
    var polygonShape;
    if (hx === void 0) {
      hx = 0;
    }
    if (hy === void 0) {
      hy = 0;
    }
    polygonShape = new b2PolygonShape();
    polygonShape.SetAsBox(hx, hy);
    return polygonShape;
  };

  b2PolygonShape.prototype.SetAsOrientedBox = function(hx, hy, center, angle) {
    var i, xf;
    if (hx === void 0) {
      hx = 0;
    }
    if (hy === void 0) {
      hy = 0;
    }
    if (center === void 0) {
      center = null;
    }
    if (angle === void 0) {
      angle = 0.0;
    }
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].Set(-hx, -hy);
    this.m_vertices[1].Set(hx, -hy);
    this.m_vertices[2].Set(hx, hy);
    this.m_vertices[3].Set(-hx, hy);
    this.m_normals[0].Set(0.0, -1.0);
    this.m_normals[1].Set(1.0, 0.0);
    this.m_normals[2].Set(0.0, 1.0);
    this.m_normals[3].Set(-1.0, 0.0);
    this.m_centroid = center;
    xf = new b2Transform();
    xf.position = center;
    xf.R.Set(angle);
    i = 0;
    while (i < this.m_vertexCount) {
      this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]);
      this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]);
      ++i;
    }
  };

  b2PolygonShape.AsOrientedBox = function(hx, hy, center, angle) {
    var polygonShape;
    if (hx === void 0) {
      hx = 0;
    }
    if (hy === void 0) {
      hy = 0;
    }
    if (center === void 0) {
      center = null;
    }
    if (angle === void 0) {
      angle = 0.0;
    }
    polygonShape = new b2PolygonShape();
    polygonShape.SetAsOrientedBox(hx, hy, center, angle);
    return polygonShape;
  };

  b2PolygonShape.prototype.SetAsEdge = function(v1, v2) {
    this.m_vertexCount = 2;
    this.Reserve(2);
    this.m_vertices[0].SetV(v1);
    this.m_vertices[1].SetV(v2);
    this.m_centroid.x = 0.5 * (v1.x + v2.x);
    this.m_centroid.y = 0.5 * (v1.y + v2.y);
    this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0);
    this.m_normals[0].Normalize();
    this.m_normals[1].x = -this.m_normals[0].x;
    this.m_normals[1].y = -this.m_normals[0].y;
  };

  b2PolygonShape.AsEdge = function(v1, v2) {
    var polygonShape;
    polygonShape = new b2PolygonShape();
    polygonShape.SetAsEdge(v1, v2);
    return polygonShape;
  };

  b2PolygonShape.prototype.TestPoint = function(xf, p) {
    var dot, i, pLocalX, pLocalY, tMat, tVec, tX, tY;
    tVec = void 0;
    tMat = xf.R;
    tX = p.x - xf.position.x;
    tY = p.y - xf.position.y;
    pLocalX = tX * tMat.col1.x + tY * tMat.col1.y;
    pLocalY = tX * tMat.col2.x + tY * tMat.col2.y;
    i = 0;
    while (i < this.m_vertexCount) {
      tVec = this.m_vertices[i];
      tX = pLocalX - tVec.x;
      tY = pLocalY - tVec.y;
      tVec = this.m_normals[i];
      dot = tVec.x * tX + tVec.y * tY;
      if (dot > 0.0) {
        return false;
      }
      ++i;
    }
    return true;
  };

  b2PolygonShape.prototype.RayCast = function(output, input, transform) {
    var dX, dY, denominator, i, index, lower, numerator, p1X, p1Y, p2X, p2Y, tMat, tVec, tX, tY, upper;
    lower = 0.0;
    upper = input.maxFraction;
    tX = 0;
    tY = 0;
    tMat = void 0;
    tVec = void 0;
    tX = input.p1.x - transform.position.x;
    tY = input.p1.y - transform.position.y;
    tMat = transform.R;
    p1X = tX * tMat.col1.x + tY * tMat.col1.y;
    p1Y = tX * tMat.col2.x + tY * tMat.col2.y;
    tX = input.p2.x - transform.position.x;
    tY = input.p2.y - transform.position.y;
    tMat = transform.R;
    p2X = tX * tMat.col1.x + tY * tMat.col1.y;
    p2Y = tX * tMat.col2.x + tY * tMat.col2.y;
    dX = p2X - p1X;
    dY = p2Y - p1Y;
    index = parseInt(-1.);
    i = 0;
    while (i < this.m_vertexCount) {
      tVec = this.m_vertices[i];
      tX = tVec.x - p1X;
      tY = tVec.y - p1Y;
      tVec = this.m_normals[i];
      numerator = tVec.x * tX + tVec.y * tY;
      denominator = tVec.x * dX + tVec.y * dY;
      if (denominator === 0.0) {
        if (numerator < 0.0) {
          return false;
        }
      } else {
        if (denominator < 0.0 && numerator < lower * denominator) {
          lower = numerator / denominator;
          index = i;
        } else {
          if (denominator > 0.0 && numerator < upper * denominator) {
            upper = numerator / denominator;
          }
        }
      }
      if (upper < lower - Number.MIN_VALUE) {
        return false;
      }
      ++i;
    }
    if (index >= 0) {
      output.fraction = lower;
      tMat = transform.R;
      tVec = this.m_normals[index];
      output.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      output.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      return true;
    }
    return false;
  };

  b2PolygonShape.prototype.ComputeAABB = function(aabb, xf) {
    var i, lowerX, lowerY, tMat, tVec, upperX, upperY, vX, vY;
    tMat = xf.R;
    tVec = this.m_vertices[0];
    lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    upperX = lowerX;
    upperY = lowerY;
    i = 1;
    while (i < this.m_vertexCount) {
      tVec = this.m_vertices[i];
      vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      lowerX = (lowerX < vX ? lowerX : vX);
      lowerY = (lowerY < vY ? lowerY : vY);
      upperX = (upperX > vX ? upperX : vX);
      upperY = (upperY > vY ? upperY : vY);
      ++i;
    }
    aabb.lowerBound.x = lowerX - this.m_radius;
    aabb.lowerBound.y = lowerY - this.m_radius;
    aabb.upperBound.x = upperX + this.m_radius;
    aabb.upperBound.y = upperY + this.m_radius;
  };

  b2PolygonShape.prototype.ComputeMass = function(massData, density) {
    var D, I, area, centerX, centerY, e1X, e1Y, e2X, e2Y, ex1, ex2, ey1, ey2, i, intx2, inty2, k_inv3, p1X, p1Y, p2, p3, px, py, triangleArea;
    if (density === void 0) {
      density = 0;
    }
    if (this.m_vertexCount === 2) {
      massData.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x);
      massData.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y);
      massData.mass = 0.0;
      massData.I = 0.0;
      return;
    }
    centerX = 0.0;
    centerY = 0.0;
    area = 0.0;
    I = 0.0;
    p1X = 0.0;
    p1Y = 0.0;
    k_inv3 = 1.0 / 3.0;
    i = 0;
    while (i < this.m_vertexCount) {
      p2 = this.m_vertices[i];
      p3 = (i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i + 1)] : this.m_vertices[0]);
      e1X = p2.x - p1X;
      e1Y = p2.y - p1Y;
      e2X = p3.x - p1X;
      e2Y = p3.y - p1Y;
      D = e1X * e2Y - e1Y * e2X;
      triangleArea = 0.5 * D;
      area += triangleArea;
      centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
      centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
      px = p1X;
      py = p1Y;
      ex1 = e1X;
      ey1 = e1Y;
      ex2 = e2X;
      ey2 = e2Y;
      intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px;
      inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;
      I += D * (intx2 + inty2);
      ++i;
    }
    massData.mass = density * area;
    centerX *= 1.0 / area;
    centerY *= 1.0 / area;
    massData.center.Set(centerX, centerY);
    massData.I = density * I;
  };

  b2PolygonShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
    var area, center, depths, diveCount, i, intoIndex, intoIndex2, intoLamdda, intoVec, isSubmerged, lastSubmerged, md, normalL, offsetL, outoIndex, outoIndex2, outoLamdda, outoVec, p2, p3, triangleArea;
    if (offset === void 0) {
      offset = 0;
    }
    normalL = b2Math.MulTMV(xf.R, normal);
    offsetL = offset - b2Math.Dot(normal, xf.position);
    depths = new Vector();
    diveCount = 0;
    intoIndex = parseInt(-1.);
    outoIndex = parseInt(-1.);
    lastSubmerged = false;
    i = 0;
    i = 0;
    while (i < this.m_vertexCount) {
      depths[i] = b2Math.Dot(normalL, this.m_vertices[i]) - offsetL;
      isSubmerged = depths[i] < (-Number.MIN_VALUE);
      if (i > 0) {
        if (isSubmerged) {
          if (!lastSubmerged) {
            intoIndex = i - 1;
            diveCount++;
          }
        } else {
          if (lastSubmerged) {
            outoIndex = i - 1;
            diveCount++;
          }
        }
      }
      lastSubmerged = isSubmerged;
      ++i;
    }
    switch (diveCount) {
      case 0:
        if (lastSubmerged) {
          md = new b2MassData();
          this.ComputeMass(md, 1);
          c.SetV(b2Math.MulX(xf, md.center));
          return md.mass;
        } else {
          return 0;
        }
        break;
      case 1:
        if (intoIndex === (-1)) {
          intoIndex = this.m_vertexCount - 1;
        } else {
          outoIndex = this.m_vertexCount - 1;
        }
    }
    intoIndex2 = parseInt((intoIndex + 1) % this.m_vertexCount);
    outoIndex2 = parseInt((outoIndex + 1) % this.m_vertexCount);
    intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
    outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
    intoVec = new b2Vec2(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda, this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
    outoVec = new b2Vec2(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda, this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
    area = 0;
    center = new b2Vec2();
    p2 = this.m_vertices[intoIndex2];
    p3 = void 0;
    i = intoIndex2;
    while (i !== outoIndex2) {
      i = (i + 1) % this.m_vertexCount;
      if (i === outoIndex2) {
        p3 = outoVec;
      } else {
        p3 = this.m_vertices[i];
      }
      triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
      area += triangleArea;
      center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
      center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
      p2 = p3;
    }
    center.Multiply(1 / area);
    c.SetV(b2Math.MulX(xf, center));
    return area;
  };

  b2PolygonShape.prototype.GetVertexCount = function() {
    return this.m_vertexCount;
  };

  b2PolygonShape.prototype.GetVertices = function() {
    return this.m_vertices;
  };

  b2PolygonShape.prototype.GetNormals = function() {
    return this.m_normals;
  };

  b2PolygonShape.prototype.GetSupport = function(d) {
    var bestIndex, bestValue, i, value;
    bestIndex = 0;
    bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    i = 1;
    while (i < this.m_vertexCount) {
      value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
      ++i;
    }
    return bestIndex;
  };

  b2PolygonShape.prototype.GetSupportVertex = function(d) {
    var bestIndex, bestValue, i, value;
    bestIndex = 0;
    bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    i = 1;
    while (i < this.m_vertexCount) {
      value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
      ++i;
    }
    return this.m_vertices[bestIndex];
  };

  b2PolygonShape.prototype.Validate = function() {
    return false;
  };

  b2PolygonShape.prototype.Reserve = function(count) {
    var i;
    if (count === void 0) {
      count = 0;
    }
    i = parseInt(this.m_vertices.length);
    while (i < count) {
      this.m_vertices[i] = new b2Vec2();
      this.m_normals[i] = new b2Vec2();
      i++;
    }
  };

  b2PolygonShape.ComputeCentroid = function(vs, count) {
    var D, area, c, e1X, e1Y, e2X, e2Y, i, inv3, p1X, p1Y, p2, p3, triangleArea;
    if (count === void 0) {
      count = 0;
    }
    c = new b2Vec2();
    area = 0.0;
    p1X = 0.0;
    p1Y = 0.0;
    inv3 = 1.0 / 3.0;
    i = 0;
    while (i < count) {
      p2 = vs[i];
      p3 = (i + 1 < count ? vs[parseInt(i + 1)] : vs[0]);
      e1X = p2.x - p1X;
      e1Y = p2.y - p1Y;
      e2X = p3.x - p1X;
      e2Y = p3.y - p1Y;
      D = e1X * e2Y - e1Y * e2X;
      triangleArea = 0.5 * D;
      area += triangleArea;
      c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
      c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
      ++i;
    }
    c.x *= 1.0 / area;
    c.y *= 1.0 / area;
    return c;
  };

  b2PolygonShape.ComputeOBB = function(obb, vs, count) {
    var area, centerX, centerY, dX, dY, i, j, length, lowerX, lowerY, minArea, p, rX, rY, root, tMat, upperX, upperY, uxX, uxY, uyX, uyY;
    if (count === void 0) {
      count = 0;
    }
    i = 0;
    p = new Array(count + 1);
    i = 0;
    while (i < count) {
      p[i] = vs[i];
      ++i;
    }
    p[count] = p[0];
    minArea = Number.MAX_VALUE;
    i = 1;
    while (i <= count) {
      root = p[parseInt(i - 1)];
      uxX = p[i].x - root.x;
      uxY = p[i].y - root.y;
      length = Math.sqrt(uxX * uxX + uxY * uxY);
      uxX /= length;
      uxY /= length;
      uyX = -uxY;
      uyY = uxX;
      lowerX = Number.MAX_VALUE;
      lowerY = Number.MAX_VALUE;
      upperX = -Number.MAX_VALUE;
      upperY = -Number.MAX_VALUE;
      j = 0;
      while (j < count) {
        dX = p[j].x - root.x;
        dY = p[j].y - root.y;
        rX = uxX * dX + uxY * dY;
        rY = uyX * dX + uyY * dY;
        if (rX < lowerX) {
          lowerX = rX;
        }
        if (rY < lowerY) {
          lowerY = rY;
        }
        if (rX > upperX) {
          upperX = rX;
        }
        if (rY > upperY) {
          upperY = rY;
        }
        ++j;
      }
      area = (upperX - lowerX) * (upperY - lowerY);
      if (area < 0.95 * minArea) {
        minArea = area;
        obb.R.col1.x = uxX;
        obb.R.col1.y = uxY;
        obb.R.col2.x = uyX;
        obb.R.col2.y = uyY;
        centerX = 0.5 * (lowerX + upperX);
        centerY = 0.5 * (lowerY + upperY);
        tMat = obb.R;
        obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
        obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
        obb.extents.x = 0.5 * (upperX - lowerX);
        obb.extents.y = 0.5 * (upperY - lowerY);
      }
      ++i;
    }
  };

  return b2PolygonShape;

})(b2Shape);

//# sourceMappingURL=b2PolygonShape.js.map

},{"../../index":103}],35:[function(require,module,exports){
var Box2D, b2Distance, b2DistanceInput, b2DistanceOutput, b2DistanceProxy, b2Settings, b2SimplexCache;

Box2D = require('../../index');

b2Settings = Box2D.Common.b2Settings;

b2SimplexCache = Box2D.Collision.b2SimplexCache;

b2Distance = Box2D.Collision.b2Distance;

b2DistanceOutput = Box2D.Collision.b2DistanceOutput;

b2DistanceInput = Box2D.Collision.b2DistanceInput;

b2DistanceProxy = Box2D.Collision.b2DistanceProxy;

Box2D.Collision.Shapes.b2Shape = (function() {
  function b2Shape() {}

  b2Shape.e_unknownShape = parseInt(-1.);

  b2Shape.e_circleShape = 0;

  b2Shape.e_polygonShape = 1;

  b2Shape.e_edgeShape = 2;

  b2Shape.e_shapeTypeCount = 3;

  b2Shape.e_hitCollide = 1;

  b2Shape.e_missCollide = 0;

  b2Shape.e_startsInsideCollide = parseInt(-1.);

  b2Shape.prototype.m_type = b2Shape.e_unknownShape;

  b2Shape.prototype.m_radius = b2Settings.b2_linearSlop;

  b2Shape.prototype.Copy = function() {
    return null;
  };

  b2Shape.prototype.Set = function(other) {
    this.m_radius = other.m_radius;
  };

  b2Shape.prototype.GetType = function() {
    return this.m_type;
  };

  b2Shape.prototype.TestPoint = function(xf, p) {
    return false;
  };

  b2Shape.prototype.RayCast = function(output, input, transform) {
    return false;
  };

  b2Shape.prototype.ComputeAABB = function(aabb, xf) {};

  b2Shape.prototype.ComputeMass = function(massData, density) {
    if (density === void 0) {
      density = 0;
    }
  };

  b2Shape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
    if (offset === void 0) {
      offset = 0;
    }
    return 0;
  };

  b2Shape.TestOverlap = function(shape1, transform1, shape2, transform2) {
    var input, output, simplexCache;
    input = new b2DistanceInput();
    input.proxyA = new b2DistanceProxy();
    input.proxyA.Set(shape1);
    input.proxyB = new b2DistanceProxy();
    input.proxyB.Set(shape2);
    input.transformA = transform1;
    input.transformB = transform2;
    input.useRadii = true;
    simplexCache = new b2SimplexCache();
    simplexCache.count = 0;
    output = new b2DistanceOutput();
    b2Distance.Distance(output, simplexCache, input);
    return output.distance < 10.0 * Number.MIN_VALUE;
  };

  return b2Shape;

})();

//# sourceMappingURL=b2Shape.js.map

},{"../../index":103}],36:[function(require,module,exports){
var Box2D, b2Math, parseUInt;

Box2D = require('../index');

b2Math = Box2D.Common.Math.b2Math;

parseUInt = function(v) {
  return Math.abs(parseInt(v));
};

Box2D.Common.b2Color = (function() {
  b2Color.prototype._r = 0;

  b2Color.prototype._g = 0;

  b2Color.prototype._b = 0;

  function b2Color(rr, gg, bb) {
    if (rr === void 0) {
      rr = 0;
    }
    if (gg === void 0) {
      gg = 0;
    }
    if (bb === void 0) {
      bb = 0;
    }
    this._r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
    this._g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
    this._b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
  }

  b2Color.prototype.Set = function(rr, gg, bb) {
    if (rr === void 0) {
      rr = 0;
    }
    if (gg === void 0) {
      gg = 0;
    }
    if (bb === void 0) {
      bb = 0;
    }
    this._r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
    this._g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
    this._b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
  };

  Object.defineProperties(b2Color.prototype, {
    r: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._r;
      },
      set: function(rr) {
        if (rr == null) {
          rr = 0;
        }
        return this._r = parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      }
    },
    g: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._g;
      },
      set: function(gg) {
        if (gg == null) {
          gg = 0;
        }
        return this._g = parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      }
    },
    b: {
      enumerable: false,
      configurable: true,
      get: function() {
        return this._b;
      },
      set: function(bb) {
        if (bb == null) {
          bb = 0;
        }
        return this._b = parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
      }
    },
    color: {
      enumerable: false,
      configurable: true,
      get: function() {
        return (this._r << 16) | (this._g << 8) | this._b;
      }
    }
  });

  return b2Color;

})();

//# sourceMappingURL=b2Color.js.map

},{"../index":103}],37:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Common.b2Settings = (function() {
  function b2Settings() {}

  b2Settings.VERSION = "2.1alpha";

  b2Settings.USHRT_MAX = 0x0000ffff;

  b2Settings.b2_pi = Math.PI;

  b2Settings.b2_maxManifoldPoints = 2;

  b2Settings.b2_aabbExtension = 0.1;

  b2Settings.b2_aabbMultiplier = 2.0;

  b2Settings.b2_polygonRadius = 2.0 * b2Settings.b2_linearSlop;

  b2Settings.b2_linearSlop = 0.005;

  b2Settings.b2_angularSlop = 2.0 / 180.0 * b2Settings.b2_pi;

  b2Settings.b2_toiSlop = 8.0 * b2Settings.b2_linearSlop;

  b2Settings.b2_maxTOIContactsPerIsland = 32;

  b2Settings.b2_maxTOIJointsPerIsland = 32;

  b2Settings.b2_velocityThreshold = 1.0;

  b2Settings.b2_maxLinearCorrection = 0.2;

  b2Settings.b2_maxAngularCorrection = 8.0 / 180.0 * b2Settings.b2_pi;

  b2Settings.b2_maxTranslation = 2.0;

  b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;

  b2Settings.b2_maxRotation = 0.5 * b2Settings.b2_pi;

  b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;

  b2Settings.b2_contactBaumgarte = 0.2;

  b2Settings.b2_timeToSleep = 0.5;

  b2Settings.b2_linearSleepTolerance = 0.01;

  b2Settings.b2_angularSleepTolerance = 2.0 / 180.0 * b2Settings.b2_pi;

  b2Settings.b2MixFriction = function(friction1, friction2) {
    if (friction1 === void 0) {
      friction1 = 0;
    }
    if (friction2 === void 0) {
      friction2 = 0;
    }
    return Math.sqrt(friction1 * friction2);
  };

  b2Settings.b2MixRestitution = function(restitution1, restitution2) {
    var _ref;
    if (restitution1 === void 0) {
      restitution1 = 0;
    }
    if (restitution2 === void 0) {
      restitution2 = 0;
    }
    return (_ref = restitution1 > restitution2) != null ? _ref : {
      restitution1: restitution2
    };
  };

  b2Settings.b2Assert = function(a) {
    if (!a) {
      throw "Assertion Failed";
    }
  };

  return b2Settings;

})();

//# sourceMappingURL=b2Settings.js.map

},{"../index":103}],38:[function(require,module,exports){
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

},{"../../index":103}],39:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Mat33 = (function() {
  b2Mat33.prototype.col1 = null;

  b2Mat33.prototype.col2 = null;

  b2Mat33.prototype.col3 = null;

  function b2Mat33(c1, c2, c3) {
    if (c1 === void 0) {
      c1 = null;
    }
    if (c2 === void 0) {
      c2 = null;
    }
    if (c3 === void 0) {
      c3 = null;
    }
    this.col1 = new b2Vec3();
    this.col2 = new b2Vec3();
    this.col3 = new b2Vec3();
    if (!c1 && !c2 && !c3) {
      this.col1.SetZero();
      this.col2.SetZero();
      this.col3.SetZero();
    } else {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
      this.col3.SetV(c3);
    }
    return;
  }

  b2Mat33.prototype.SetVVV = function(c1, c2, c3) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
    this.col3.SetV(c3);
  };

  b2Mat33.prototype.Copy = function() {
    new b2Mat33(this.col1, this.col2, this.col3);
  };

  b2Mat33.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
    this.col3.SetV(m.col3);
  };

  b2Mat33.prototype.AddM = function(m) {
    this.col1.x += m.col1.x;
    this.col1.y += m.col1.y;
    this.col1.z += m.col1.z;
    this.col2.x += m.col2.x;
    this.col2.y += m.col2.y;
    this.col2.z += m.col2.z;
    this.col3.x += m.col3.x;
    this.col3.y += m.col3.y;
    this.col3.z += m.col3.z;
  };

  b2Mat33.prototype.SetIdentity = function() {
    this.col1.x = 1.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 1.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 1.0;
  };

  b2Mat33.prototype.SetZero = function() {
    this.col1.x = 0.0;
    this.col2.x = 0.0;
    this.col3.x = 0.0;
    this.col1.y = 0.0;
    this.col2.y = 0.0;
    this.col3.y = 0.0;
    this.col1.z = 0.0;
    this.col2.z = 0.0;
    this.col3.z = 0.0;
  };

  b2Mat33.prototype.Solve22 = function(out, bX, bY) {
    var a11, a12, a21, a22, det;
    if (bX === void 0) {
      bX = 0;
    }
    if (bY === void 0) {
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

  b2Mat33.prototype.Solve33 = function(out, bX, bY, bZ) {
    var a11, a12, a13, a21, a22, a23, a31, a32, a33, det;
    if (bX === void 0) {
      bX = 0;
    }
    if (bY === void 0) {
      bY = 0;
    }
    if (bZ === void 0) {
      bZ = 0;
    }
    a11 = this.col1.x;
    a21 = this.col1.y;
    a31 = this.col1.z;
    a12 = this.col2.x;
    a22 = this.col2.y;
    a32 = this.col2.z;
    a13 = this.col3.x;
    a23 = this.col3.y;
    a33 = this.col3.z;
    det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
    if (det !== 0.0) {
      det = 1.0 / det;
    }
    out.x = det * (bX * (a22 * a33 - a32 * a23) + bY * (a32 * a13 - a12 * a33) + bZ * (a12 * a23 - a22 * a13));
    out.y = det * (a11 * (bY * a33 - bZ * a23) + a21 * (bZ * a13 - bX * a33) + a31 * (bX * a23 - bY * a13));
    out.z = det * (a11 * (a22 * bZ - a32 * bY) + a21 * (a32 * bX - a12 * bZ) + a31 * (a12 * bY - a22 * bX));
    return out;
  };

  return b2Mat33;

})();

//# sourceMappingURL=b2Mat33.js.map

},{"../../index":103}],40:[function(require,module,exports){
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
    return (a > 0.0 ? a : -a);
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
    return (a < b ? a : b);
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
    return (a > b ? a : b);
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
    return (a < low ? low : (a > high ? high : a));
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

},{"../../index":103}],41:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Sweep = (function() {
  b2Sweep.prototype.localCenter = null;

  b2Sweep.prototype.c0 = null;

  b2Sweep.prototype.c = null;

  b2Sweep.prototype.a0 = null;

  b2Sweep.prototype.a = null;

  b2Sweep.prototype.t0 = null;

  function b2Sweep() {
    this.localCenter = new b2Vec2();
    this.c0 = new b2Vec2;
    this.c = new b2Vec2();
    return;
  }

  b2Sweep.prototype.Set = function(other) {
    this.localCenter.SetV(other.localCenter);
    this.c0.SetV(other.c0);
    this.c.SetV(other.c);
    this.a0 = other.a0;
    this.a = other.a;
    this.t0 = other.t0;
  };

  b2Sweep.prototype.Copy = function() {
    var copy;
    copy = new b2Sweep();
    copy.localCenter.SetV(this.localCenter);
    copy.c0.SetV(this.c0);
    copy.c.SetV(this.c);
    copy.a0 = this.a0;
    copy.a = this.a;
    copy.t0 = this.t0;
    return copy;
  };

  b2Sweep.prototype.GetTransform = function(xf, alpha) {
    var angle, tMat;
    if (alpha === void 0) {
      alpha = 0;
    }
    xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
    xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
    angle = (1.0 - alpha) * this.a0 + alpha * this.a;
    xf.R.Set(angle);
    tMat = xf.R;
    xf.position.x -= tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y;
    xf.position.y -= tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y;
  };

  b2Sweep.prototype.Advance = function(t) {
    var alpha;
    if (t === void 0) {
      t = 0;
    }
    if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
      alpha = (t - this.t0) / (1.0 - this.t0);
      this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
      this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
      this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
      this.t0 = t;
    }
  };

  return b2Sweep;

})();

//# sourceMappingURL=b2Sweep.js.map

},{"../../index":103}],42:[function(require,module,exports){
var Box2D, b2Mat22, b2Vec2;

Box2D = require('../../index');

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Common.Math.b2Transform = (function() {
  b2Transform.prototype.position = null;

  b2Transform.prototype.R = null;

  function b2Transform(pos, r) {
    this.position = new b2Vec2;
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

  b2Transform.prototype.GetAngle = function() {
    return Math.atan2(this.R.col1.y, this.R.col1.x);
  };

  return b2Transform;

})();

//# sourceMappingURL=b2Transform.js.map

},{"../../index":103}],43:[function(require,module,exports){
var Box2D, b2Math;

Box2D = require('../../index');

b2Math = Box2D.Common.Math.b2Math;

Box2D.Common.Math.b2Vec2 = (function() {
  b2Vec2.prototype.x = 0;

  b2Vec2.prototype.y = 0;

  function b2Vec2(x_, y_) {
    if (x_ === void 0) {
      x_ = 0;
    }
    if (y_ === void 0) {
      y_ = 0;
    }
    this.x = x_;
    this.y = y_;
    return;
  }

  b2Vec2.prototype.SetZero = function() {
    this.x = 0.0;
    this.y = 0.0;
  };

  b2Vec2.prototype.Set = function(x_, y_) {
    if (x_ === void 0) {
      x_ = 0;
    }
    if (y_ === void 0) {
      y_ = 0;
    }
    this.x = x_;
    this.y = y_;
  };

  b2Vec2.prototype.SetV = function(v) {
    this.x = v.x;
    this.y = v.y;
  };

  b2Vec2.prototype.GetNegative = function() {
    return new b2Vec2(-this.x, -this.y);
  };

  b2Vec2.prototype.NegativeSelf = function() {
    this.x = -this.x;
    this.y = -this.y;
  };

  b2Vec2.Make = function(x_, y_) {
    if (x_ === void 0) {
      x_ = 0;
    }
    if (y_ === void 0) {
      y_ = 0;
    }
    return new b2Vec2(x_, y_);
  };

  b2Vec2.prototype.Copy = function() {
    return new b2Vec2(this.x, this.y);
  };

  b2Vec2.prototype.Add = function(v) {
    this.x += v.x;
    this.y += v.y;
  };

  b2Vec2.prototype.Subtract = function(v) {
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

  b2Vec2.prototype.MulM = function(A) {
    var tX;
    tX = this.x;
    this.x = A.col1.x * tX + A.col2.x * this.y;
    this.y = A.col1.y * tX + A.col2.y * this.y;
  };

  b2Vec2.prototype.MulTM = function(A) {
    var tX;
    tX = b2Math.Dot(this, A.col1);
    this.y = b2Math.Dot(this, A.col2);
    this.x = tX;
  };

  b2Vec2.prototype.CrossVF = function(s) {
    var tX;
    if (s === void 0) {
      s = 0;
    }
    tX = this.x;
    this.x = s * this.y;
    this.y = -s * tX;
  };

  b2Vec2.prototype.CrossFV = function(s) {
    var tX;
    if (s === void 0) {
      s = 0;
    }
    tX = this.x;
    this.x = -s * this.y;
    this.y = s * tX;
  };

  b2Vec2.prototype.MinV = function(b) {
    this.x = (this.x < b.x ? this.x : b.x);
    this.y = (this.y < b.y ? this.y : b.y);
  };

  b2Vec2.prototype.MaxV = function(b) {
    this.x = (this.x > b.x ? this.x : b.x);
    this.y = (this.y > b.y ? this.y : b.y);
  };

  b2Vec2.prototype.Abs = function() {
    if (this.x < 0) {
      this.x = -this.x;
    }
    if (this.y < 0) {
      this.y = -this.y;
    }
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

  b2Vec2.prototype.IsValid = function() {
    return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
  };

  return b2Vec2;

})();

//# sourceMappingURL=b2Vec2.js.map

},{"../../index":103}],44:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Common.Math.b2Vec3 = (function() {
  b2Vec3.prototype.x = 0;

  b2Vec3.prototype.y = 0;

  b2Vec3.prototype.z = 0;

  function b2Vec3(x, y, z) {
    if (x === void 0) {
      x = 0;
    }
    if (y === void 0) {
      y = 0;
    }
    if (z === void 0) {
      z = 0;
    }
    this.x = x;
    this.y = y;
    this.z = z;
    return;
  }

  b2Vec3.prototype.SetZero = function() {
    this.x = this.y = this.z = 0.0;
  };

  b2Vec3.prototype.Set = function(x, y, z) {
    if (x === void 0) {
      x = 0;
    }
    if (y === void 0) {
      y = 0;
    }
    if (z === void 0) {
      z = 0;
    }
    this.x = x;
    this.y = y;
    this.z = z;
  };

  b2Vec3.prototype.SetV = function(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
  };

  b2Vec3.prototype.GetNegative = function() {
    return new b2Vec3(-this.x, -this.y, -this.z);
  };

  b2Vec3.prototype.NegativeSelf = function() {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
  };

  b2Vec3.prototype.Copy = function() {
    return new b2Vec3(this.x, this.y, this.z);
  };

  b2Vec3.prototype.Add = function(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
  };

  b2Vec3.prototype.Subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
  };

  b2Vec3.prototype.Multiply = function(a) {
    if (a === void 0) {
      a = 0;
    }
    this.x *= a;
    this.y *= a;
    this.z *= a;
  };

  return b2Vec3;

})();

//# sourceMappingURL=b2Vec3.js.map

},{"../../index":103}],45:[function(require,module,exports){
var Box2D, b2BodyDef, b2Fixture, b2FixtureDef, b2Math, b2Settings, b2Sweep, b2Transform, b2Vec2;

Box2D = require('../index');

b2Math = Box2D.Common.Math.b2Math;

b2Sweep = Box2D.Common.Math.b2Sweep;

b2Transform = Box2D.Common.Math.b2Transform;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Settings = Box2D.Common.b2Settings;

b2Fixture = Box2D.Dynamics.b2Fixture;

b2FixtureDef = Box2D.Dynamics.b2FixtureDef;

b2BodyDef = Box2D.Dynamics.b2BodyDef;

Box2D.Dynamics.b2Body = (function() {
  b2Body.s_xf1 = new b2Transform();

  b2Body.e_islandFlag = 0x0001;

  b2Body.e_awakeFlag = 0x0002;

  b2Body.e_allowSleepFlag = 0x0004;

  b2Body.e_bulletFlag = 0x0008;

  b2Body.e_fixedRotationFlag = 0x0010;

  b2Body.e_activeFlag = 0x0020;

  b2Body.b2_staticBody = 0;

  b2Body.b2_kinematicBody = 1;

  b2Body.b2_dynamicBody = 2;

  b2Body.prototype.m_xf = null;

  b2Body.prototype.m_sweep = null;

  b2Body.prototype.m_linearVelocity = null;

  b2Body.prototype.m_force = null;

  b2Body.prototype.m_flags = 0;

  b2Body.prototype.m_world = null;

  b2Body.prototype.m_jointList = null;

  b2Body.prototype.m_controllerList = null;

  b2Body.prototype.m_contactList = null;

  b2Body.prototype.m_controllerCount = null;

  b2Body.prototype.m_prev = null;

  b2Body.prototype.m_next = null;

  b2Body.prototype.m_linearVelocity = null;

  b2Body.prototype.m_angularVelocity = null;

  b2Body.prototype.m_linearDamping = null;

  b2Body.prototype.m_angularDamping = null;

  b2Body.prototype.m_force = null;

  b2Body.prototype.m_torque = 0;

  b2Body.prototype.m_sleepTime = 0;

  b2Body.prototype.m_type = 0;

  b2Body.prototype.m_mass = 0;

  b2Body.prototype.m_invMass = 0;

  b2Body.prototype.m_I = 0;

  b2Body.prototype.m_invI = 0;

  b2Body.prototype.m_inertiaScale = 0;

  b2Body.prototype.m_userData = null;

  b2Body.prototype.m_fixtureList = null;

  b2Body.prototype.m_fixtureCount = 0;

  function b2Body(bd, world) {
    var tMat, tVec;
    this.m_xf = new b2Transform();
    this.m_sweep = new b2Sweep();
    this.m_linearVelocity = new b2Vec2();
    this.m_force = new b2Vec2();
    this.m_flags = 0;
    if (bd.bullet) {
      this.m_flags |= b2Body.e_bulletFlag;
    }
    if (bd.fixedRotation) {
      this.m_flags |= b2Body.e_fixedRotationFlag;
    }
    if (bd.allowSleep) {
      this.m_flags |= b2Body.e_allowSleepFlag;
    }
    if (bd.awake) {
      this.m_flags |= b2Body.e_awakeFlag;
    }
    if (bd.active) {
      this.m_flags |= b2Body.e_activeFlag;
    }
    this.m_world = world;
    this.m_xf.position.SetV(bd.position);
    this.m_xf.R.Set(bd.angle);
    this.m_sweep.localCenter.SetZero();
    this.m_sweep.t0 = 1.0;
    this.m_sweep.a0 = this.m_sweep.a = bd.angle;
    tMat = this.m_xf.R;
    tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
    this.m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_jointList = null;
    this.m_controllerList = null;
    this.m_contactList = null;
    this.m_controllerCount = 0;
    this.m_prev = null;
    this.m_next = null;
    this.m_linearVelocity.SetV(bd.linearVelocity);
    this.m_angularVelocity = bd.angularVelocity;
    this.m_linearDamping = bd.linearDamping;
    this.m_angularDamping = bd.angularDamping;
    this.m_force.Set(0.0, 0.0);
    this.m_torque = 0.0;
    this.m_sleepTime = 0.0;
    this.m_type = bd.type;
    if (this.m_type === b2Body.b2_dynamicBody) {
      this.m_mass = 1.0;
      this.m_invMass = 1.0;
    } else {
      this.m_mass = 0.0;
      this.m_invMass = 0.0;
    }
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_inertiaScale = bd.inertiaScale;
    this.m_userData = bd.userData;
    this.m_fixtureList = null;
    this.m_fixtureCount = 0;
    return;
  }

  b2Body.prototype.ConnectEdges = function(s1, s2, angle1) {
    var angle2, convex, core, coreOffset, cornerDir;
    if (angle1 === void 0) {
      angle1 = 0;
    }
    angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
    coreOffset = Math.tan((angle2 - angle1) * 0.5);
    core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
    core = b2Math.SubtractVV(core, s2.GetNormalVector());
    core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
    core = b2Math.AddVV(core, s2.GetVertex1());
    cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
    cornerDir.Normalize();
    convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
    s1.SetNextEdge(s2, core, cornerDir, convex);
    s2.SetPrevEdge(s1, core, cornerDir, convex);
    return angle2;
  };

  b2Body.prototype.CreateFixture = function(def) {
    var broadPhase, fixture;
    if (this.m_world.IsLocked() === true) {
      return null;
    }
    fixture = new b2Fixture();
    fixture.Create(this, this.m_xf, def);
    if (this.m_flags & b2Body.e_activeFlag) {
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      fixture.CreateProxy(broadPhase, this.m_xf);
    }
    fixture.m_next = this.m_fixtureList;
    this.m_fixtureList = fixture;
    ++this.m_fixtureCount;
    fixture.m_body = this;
    if (fixture.m_density > 0.0) {
      this.ResetMassData();
    }
    this.m_world.m_flags |= 0x0001;
    return fixture;
  };

  b2Body.prototype.CreateFixture2 = function(shape, density) {
    var def;
    if (density === void 0) {
      density = 0.0;
    }
    def = new b2FixtureDef();
    def.shape = shape;
    def.density = density;
    return this.CreateFixture(def);
  };

  b2Body.prototype.DestroyFixture = function(fixture) {
    var broadPhase, c, edge, fixtureA, fixtureB, found, node, ppF;
    if (this.m_world.IsLocked() === true) {
      return;
    }
    node = this.m_fixtureList;
    ppF = null;
    found = false;
    while (node != null) {
      if (node === fixture) {
        if (ppF) {
          ppF.m_next = fixture.m_next;
        } else {
          this.m_fixtureList = fixture.m_next;
        }
        found = true;
        break;
      }
      ppF = node;
      node = node.m_next;
    }
    edge = this.m_contactList;
    while (edge) {
      c = edge.contact;
      edge = edge.next;
      fixtureA = c.GetFixtureA();
      fixtureB = c.GetFixtureB();
      if (fixture === fixtureA || fixture === fixtureB) {
        this.m_world.m_contactManager.Destroy(c);
      }
    }
    if (this.m_flags & b2Body.e_activeFlag) {
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      fixture.DestroyProxy(broadPhase);
    } else {

    }
    fixture.Destroy();
    fixture.m_body = null;
    fixture.m_next = null;
    --this.m_fixtureCount;
    this.ResetMassData();
  };

  b2Body.prototype.SetPositionAndAngle = function(position, angle) {
    var broadPhase, f, tMat, tVec;
    if (angle === void 0) {
      angle = 0;
    }
    f = void 0;
    if (this.m_world.IsLocked() === true) {
      return;
    }
    this.m_xf.R.Set(angle);
    this.m_xf.position.SetV(position);
    tMat = this.m_xf.R;
    tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
    this.m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_sweep.a0 = this.m_sweep.a = angle;
    broadPhase = this.m_world.m_contactManager.m_broadPhase;
    f = this.m_fixtureList;
    while (f) {
      f.Synchronize(broadPhase, this.m_xf, this.m_xf);
      f = f.m_next;
    }
    this.m_world.m_contactManager.FindNewContacts();
  };

  b2Body.prototype.SetTransform = function(xf) {
    this.SetPositionAndAngle(xf.position, xf.GetAngle());
  };

  b2Body.prototype.GetTransform = function() {
    return this.m_xf;
  };

  b2Body.prototype.GetPosition = function() {
    return this.m_xf.position;
  };

  b2Body.prototype.SetPosition = function(position) {
    this.SetPositionAndAngle(position, this.GetAngle());
  };

  b2Body.prototype.GetAngle = function() {
    return this.m_sweep.a;
  };

  b2Body.prototype.SetAngle = function(angle) {
    if (angle === void 0) {
      angle = 0;
    }
    this.SetPositionAndAngle(this.GetPosition(), angle);
  };

  b2Body.prototype.GetWorldCenter = function() {
    return this.m_sweep.c;
  };

  b2Body.prototype.GetLocalCenter = function() {
    return this.m_sweep.localCenter;
  };

  b2Body.prototype.SetLinearVelocity = function(v) {
    if (this.m_type === b2Body.b2_staticBody) {
      return;
    }
    this.m_linearVelocity.SetV(v);
  };

  b2Body.prototype.GetLinearVelocity = function() {
    return this.m_linearVelocity;
  };

  b2Body.prototype.SetAngularVelocity = function(omega) {
    if (omega === void 0) {
      omega = 0;
    }
    if (this.m_type === b2Body.b2_staticBody) {
      return;
    }
    this.m_angularVelocity = omega;
  };

  b2Body.prototype.GetAngularVelocity = function() {
    return this.m_angularVelocity;
  };

  b2Body.prototype.GetDefinition = function() {
    var bd;
    bd = new b2BodyDef();
    bd.type = this.GetType();
    bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
    bd.angle = this.GetAngle();
    bd.angularDamping = this.m_angularDamping;
    bd.angularVelocity = this.m_angularVelocity;
    bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
    bd.bullet = (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
    bd.awake = (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
    bd.linearDamping = this.m_linearDamping;
    bd.linearVelocity.SetV(this.GetLinearVelocity());
    bd.position = this.GetPosition();
    bd.userData = this.GetUserData();
    return bd;
  };

  b2Body.prototype.ApplyForce = function(force, point) {
    if (this.m_type !== b2Body.b2_dynamicBody) {
      return;
    }
    if (this.IsAwake() === false) {
      this.SetAwake(true);
    }
    this.m_force.x += force.x;
    this.m_force.y += force.y;
    this.m_torque += (point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x;
  };

  b2Body.prototype.ApplyTorque = function(torque) {
    if (torque === void 0) {
      torque = 0;
    }
    if (this.m_type !== b2Body.b2_dynamicBody) {
      return;
    }
    if (this.IsAwake() === false) {
      this.SetAwake(true);
    }
    this.m_torque += torque;
  };

  b2Body.prototype.ApplyImpulse = function(impulse, point) {
    if (this.m_type !== b2Body.b2_dynamicBody) {
      return;
    }
    if (this.IsAwake() === false) {
      this.SetAwake(true);
    }
    this.m_linearVelocity.x += this.m_invMass * impulse.x;
    this.m_linearVelocity.y += this.m_invMass * impulse.y;
    this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
  };

  b2Body.prototype.Split = function(callback) {
    var angularVelocity, body1, body2, center, center1, center2, f, linearVelocity, next, prev, velocity1, velocity2;
    linearVelocity = this.GetLinearVelocity().Copy();
    angularVelocity = this.GetAngularVelocity();
    center = this.GetWorldCenter();
    body1 = this;
    body2 = this.m_world.CreateBody(this.GetDefinition());
    prev = void 0;
    f = body1.m_fixtureList;
    while (f) {
      if (callback(f)) {
        next = f.m_next;
        if (prev) {
          prev.m_next = next;
        } else {
          body1.m_fixtureList = next;
        }
        body1.m_fixtureCount--;
        f.m_next = body2.m_fixtureList;
        body2.m_fixtureList = f;
        body2.m_fixtureCount++;
        f.m_body = body2;
        f = next;
      } else {
        prev = f;
        f = f.m_next;
      }
    }
    body1.ResetMassData();
    body2.ResetMassData();
    center1 = body1.GetWorldCenter();
    center2 = body2.GetWorldCenter();
    velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));
    velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
    body1.SetLinearVelocity(velocity1);
    body2.SetLinearVelocity(velocity2);
    body1.SetAngularVelocity(angularVelocity);
    body2.SetAngularVelocity(angularVelocity);
    body1.SynchronizeFixtures();
    body2.SynchronizeFixtures();
    return body2;
  };

  b2Body.prototype.Merge = function(other) {
    var angular, angular1, body1, body2, center1, center2, f, next, velocity1, velocity2;
    f = void 0;
    f = other.m_fixtureList;
    while (f) {
      next = f.m_next;
      other.m_fixtureCount--;
      f.m_next = this.m_fixtureList;
      this.m_fixtureList = f;
      this.m_fixtureCount++;
      f.m_body = body2;
      f = next;
    }
    body1.m_fixtureCount = 0;
    body1 = this;
    body2 = other;
    center1 = body1.GetWorldCenter();
    center2 = body2.GetWorldCenter();
    velocity1 = body1.GetLinearVelocity().Copy();
    velocity2 = body2.GetLinearVelocity().Copy();
    angular1 = body1.GetAngularVelocity();
    angular = body2.GetAngularVelocity();
    body1.ResetMassData();
    this.SynchronizeFixtures();
  };

  b2Body.prototype.GetMass = function() {
    return this.m_mass;
  };

  b2Body.prototype.GetInertia = function() {
    return this.m_I;
  };

  b2Body.prototype.GetMassData = function(data) {
    data.mass = this.m_mass;
    data.I = this.m_I;
    data.center.SetV(this.m_sweep.localCenter);
  };

  b2Body.prototype.SetMassData = function(massData) {
    var oldCenter;
    b2Settings.b2Assert(this.m_world.IsLocked() === false);
    if (this.m_world.IsLocked() === true) {
      return;
    }
    if (this.m_type !== b2Body.b2_dynamicBody) {
      return;
    }
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_mass = massData.mass;
    if (this.m_mass <= 0.0) {
      this.m_mass = 1.0;
    }
    this.m_invMass = 1.0 / this.m_mass;
    if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
      this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
      this.m_invI = 1.0 / this.m_I;
    }
    oldCenter = this.m_sweep.c.Copy();
    this.m_sweep.localCenter.SetV(massData.center);
    this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
  };

  b2Body.prototype.ResetMassData = function() {
    var center, f, massData, oldCenter;
    this.m_mass = 0.0;
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_sweep.localCenter.SetZero();
    if (this.m_type === b2Body.b2_staticBody || this.m_type === b2Body.b2_kinematicBody) {
      return;
    }
    center = b2Vec2.Make(0, 0);
    f = this.m_fixtureList;
    while (f) {
      if (f.m_density === 0.0) {
        continue;
      }
      massData = f.GetMassData();
      this.m_mass += massData.mass;
      center.x += massData.center.x * massData.mass;
      center.y += massData.center.y * massData.mass;
      this.m_I += massData.I;
      f = f.m_next;
    }
    if (this.m_mass > 0.0) {
      this.m_invMass = 1.0 / this.m_mass;
      center.x *= this.m_invMass;
      center.y *= this.m_invMass;
    } else {
      this.m_mass = 1.0;
      this.m_invMass = 1.0;
    }
    if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
      this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
      this.m_I *= this.m_inertiaScale;
      b2Settings.b2Assert(this.m_I > 0);
      this.m_invI = 1.0 / this.m_I;
    } else {
      this.m_I = 0.0;
      this.m_invI = 0.0;
    }
    oldCenter = this.m_sweep.c.Copy();
    this.m_sweep.localCenter.SetV(center);
    this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
  };

  b2Body.prototype.GetWorldPoint = function(localPoint) {
    var A, u;
    A = this.m_xf.R;
    u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    u.x += this.m_xf.position.x;
    u.y += this.m_xf.position.y;
    return u;
  };

  b2Body.prototype.GetWorldVector = function(localVector) {
    return b2Math.MulMV(this.m_xf.R, localVector);
  };

  b2Body.prototype.GetLocalPoint = function(worldPoint) {
    return b2Math.MulXT(this.m_xf, worldPoint);
  };

  b2Body.prototype.GetLocalVector = function(worldVector) {
    return b2Math.MulTMV(this.m_xf.R, worldVector);
  };

  b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
    return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
  };

  b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
    var A, worldPoint;
    A = this.m_xf.R;
    worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    worldPoint.x += this.m_xf.position.x;
    worldPoint.y += this.m_xf.position.y;
    return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
  };

  b2Body.prototype.GetLinearDamping = function() {
    return this.m_linearDamping;
  };

  b2Body.prototype.SetLinearDamping = function(linearDamping) {
    if (linearDamping === void 0) {
      linearDamping = 0;
    }
    this.m_linearDamping = linearDamping;
  };

  b2Body.prototype.GetAngularDamping = function() {
    return this.m_angularDamping;
  };

  b2Body.prototype.SetAngularDamping = function(angularDamping) {
    if (angularDamping === void 0) {
      angularDamping = 0;
    }
    this.m_angularDamping = angularDamping;
  };

  b2Body.prototype.SetType = function(type) {
    var ce;
    if (type === void 0) {
      type = 0;
    }
    if (this.m_type === type) {
      return;
    }
    this.m_type = type;
    this.ResetMassData();
    if (this.m_type === b2Body.b2_staticBody) {
      this.m_linearVelocity.SetZero();
      this.m_angularVelocity = 0.0;
    }
    this.SetAwake(true);
    this.m_force.SetZero();
    this.m_torque = 0.0;
    ce = this.m_contactList;
    while (ce) {
      ce.contact.FlagForFiltering();
      ce = ce.next;
    }
  };

  b2Body.prototype.GetType = function() {
    return this.m_type;
  };

  b2Body.prototype.SetBullet = function(flag) {
    if (flag) {
      this.m_flags |= b2Body.e_bulletFlag;
    } else {
      this.m_flags &= ~b2Body.e_bulletFlag;
    }
  };

  b2Body.prototype.IsBullet = function() {
    return (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
  };

  b2Body.prototype.SetSleepingAllowed = function(flag) {
    if (flag) {
      this.m_flags |= b2Body.e_allowSleepFlag;
    } else {
      this.m_flags &= ~b2Body.e_allowSleepFlag;
      this.SetAwake(true);
    }
  };

  b2Body.prototype.SetAwake = function(flag) {
    if (flag) {
      this.m_flags |= b2Body.e_awakeFlag;
      this.m_sleepTime = 0.0;
    } else {
      this.m_flags &= ~b2Body.e_awakeFlag;
      this.m_sleepTime = 0.0;
      this.m_linearVelocity.SetZero();
      this.m_angularVelocity = 0.0;
      this.m_force.SetZero();
      this.m_torque = 0.0;
    }
  };

  b2Body.prototype.IsAwake = function() {
    return (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
  };

  b2Body.prototype.SetFixedRotation = function(fixed) {
    if (fixed) {
      this.m_flags |= b2Body.e_fixedRotationFlag;
    } else {
      this.m_flags &= ~b2Body.e_fixedRotationFlag;
    }
    this.ResetMassData();
  };

  b2Body.prototype.IsFixedRotation = function() {
    return (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
  };

  b2Body.prototype.SetActive = function(flag) {
    var broadPhase, ce, ce0, f;
    if (flag === this.IsActive()) {
      return;
    }
    broadPhase = void 0;
    f = void 0;
    if (flag) {
      this.m_flags |= b2Body.e_activeFlag;
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      f = this.m_fixtureList;
      while (f) {
        f.CreateProxy(broadPhase, this.m_xf);
        f = f.m_next;
      }
    } else {
      this.m_flags &= ~b2Body.e_activeFlag;
      broadPhase = this.m_world.m_contactManager.m_broadPhase;
      f = this.m_fixtureList;
      while (f) {
        f.DestroyProxy(broadPhase);
        f = f.m_next;
      }
      ce = this.m_contactList;
      while (ce) {
        ce0 = ce;
        ce = ce.next;
        this.m_world.m_contactManager.Destroy(ce0.contact);
      }
      this.m_contactList = null;
    }
  };

  b2Body.prototype.IsActive = function() {
    return (this.m_flags & b2Body.e_activeFlag) === b2Body.e_activeFlag;
  };

  b2Body.prototype.IsSleepingAllowed = function() {
    return (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
  };

  b2Body.prototype.GetFixtureList = function() {
    return this.m_fixtureList;
  };

  b2Body.prototype.GetJointList = function() {
    return this.m_jointList;
  };

  b2Body.prototype.GetControllerList = function() {
    return this.m_controllerList;
  };

  b2Body.prototype.GetContactList = function() {
    return this.m_contactList;
  };

  b2Body.prototype.GetNext = function() {
    return this.m_next;
  };

  b2Body.prototype.GetUserData = function() {
    return this.m_userData;
  };

  b2Body.prototype.SetUserData = function(data) {
    this.m_userData = data;
  };

  b2Body.prototype.GetWorld = function() {
    return this.m_world;
  };

  b2Body.prototype.SynchronizeFixtures = function() {
    var broadPhase, f, tMat, tVec, xf1;
    xf1 = b2Body.s_xf1;
    xf1.R.Set(this.m_sweep.a0);
    tMat = xf1.R;
    tVec = this.m_sweep.localCenter;
    xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    f = void 0;
    broadPhase = this.m_world.m_contactManager.m_broadPhase;
    f = this.m_fixtureList;
    while (f) {
      f.Synchronize(broadPhase, xf1, this.m_xf);
      f = f.m_next;
    }
  };

  b2Body.prototype.SynchronizeTransform = function() {
    var tMat, tVec;
    this.m_xf.R.Set(this.m_sweep.a);
    tMat = this.m_xf.R;
    tVec = this.m_sweep.localCenter;
    this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  };

  b2Body.prototype.ShouldCollide = function(other) {
    var jn;
    if (this.m_type !== b2Body.b2_dynamicBody && other.m_type !== b2Body.b2_dynamicBody) {
      return false;
    }
    jn = this.m_jointList;
    while (jn) {
      if (jn.other === other) {
        if (jn.joint.m_collideConnected === false) {
          return false;
        }
      }
      jn = jn.next;
    }
    return true;
  };

  b2Body.prototype.Advance = function(t) {
    if (t === void 0) {
      t = 0;
    }
    this.m_sweep.Advance(t);
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_sweep.a = this.m_sweep.a0;
    this.SynchronizeTransform();
  };

  return b2Body;

})();

//# sourceMappingURL=b2Body.js.map

},{"../index":103}],46:[function(require,module,exports){
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

  b2BodyDef.prototype.linearDamping = 0.0;

  b2BodyDef.prototype.angularDamping = 0.0;

  b2BodyDef.prototype.allowSleep = true;

  b2BodyDef.prototype.awake = true;

  b2BodyDef.prototype.fixedRotation = false;

  b2BodyDef.prototype.bullet = false;

  b2BodyDef.prototype.type = b2Body.b2_staticBody;

  b2BodyDef.prototype.active = true;

  b2BodyDef.prototype.inertiaScale = 1.0;

  function b2BodyDef() {
    this.position = new b2Vec2();
    this.position.Set(0.0, 0.0);
    this.linearVelocity = new b2Vec2();
    this.linearVelocity.Set(0, 0);
    return;
  }

  return b2BodyDef;

})();

//# sourceMappingURL=b2BodyDef.js.map

},{"../index":103}],47:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2ContactFilter = (function() {
  function b2ContactFilter() {}

  b2ContactFilter.prototype.ShouldCollide = function(fixtureA, fixtureB) {
    var collide, filter1, filter2;
    filter1 = fixtureA.GetFilterData();
    filter2 = fixtureB.GetFilterData();
    if (filter1.groupIndex === filter2.groupIndex && filter1.groupIndex !== 0) {
      return filter1.groupIndex > 0;
    }
    collide = (filter1.maskBits & filter2.categoryBits) !== 0 && (filter1.categoryBits & filter2.maskBits) !== 0;
    return collide;
  };

  b2ContactFilter.prototype.RayCollide = function(userData, fixture) {
    if (!userData) {
      return true;
    }
    return this.ShouldCollide((userData instanceof b2Fixture ? userData : null), fixture);
  };

  b2ContactFilter.b2_defaultFilter = new b2ContactFilter();

  return b2ContactFilter;

})();

//# sourceMappingURL=b2ContactFilter.js.map

},{"../index":103}],48:[function(require,module,exports){
var Box2D, Vector, b2Settings;

Box2D = require('../index');

Vector = Box2D.Vector;

b2Settings = Box2D.Common.b2Settings;

Box2D.Dynamics.b2ContactImpulse = (function() {
  function b2ContactImpulse() {}

  b2ContactImpulse.prototype.normalImpulses = null;

  b2ContactImpulse.prototype.tangentImpulses = null;

  b2ContactImpulse.b2ContactImpulse = function() {
    this.normalImpulses = new Vector(b2Settings.b2_maxManifoldPoints);
    this.tangentImpulses = new Vector(b2Settings.b2_maxManifoldPoints);
  };

  return b2ContactImpulse;

})();

//# sourceMappingURL=b2ContactImpulse.js.map

},{"../index":103}],49:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2ContactListener = (function() {
  function b2ContactListener() {}

  b2ContactListener.prototype.BeginContact = function(contact) {};

  b2ContactListener.prototype.EndContact = function(contact) {};

  b2ContactListener.prototype.PreSolve = function(contact, oldManifold) {};

  b2ContactListener.prototype.PostSolve = function(contact, impulse) {};

  b2ContactListener.b2_defaultListener = new b2ContactListener();

  return b2ContactListener;

})();

//# sourceMappingURL=b2ContactListener.js.map

},{"../index":103}],50:[function(require,module,exports){
var Box2D, b2ContactFactory, b2ContactFilter, b2ContactListener, b2ContactPoint, b2DynamicTreeBroadPhase, b2Fixture;

Box2D = require('../index');

b2Fixture = Box2D.Dynamics.b2Fixture;

b2ContactFilter = Box2D.Dynamics.b2ContactFilter;

b2ContactListener = Box2D.Dynamics.b2ContactListener;

b2ContactFactory = Box2D.Dynamics.Contacts.b2ContactFactory;

b2ContactPoint = Box2D.Collision.b2ContactPoint;

b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase;

Box2D.Dynamics.b2ContactManager = (function() {
  b2ContactManager.prototype.m_world = null;

  b2ContactManager.prototype.m_contactCount = null;

  b2ContactManager.prototype.m_contactFilter = null;

  b2ContactManager.prototype.m_contactListener = null;

  b2ContactManager.prototype.m_contactFactory = null;

  b2ContactManager.prototype.m_broadPhase = null;

  function b2ContactManager() {
    this.m_world = null;
    this.m_contactCount = 0;
    this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
    this.m_contactListener = b2ContactListener.b2_defaultListener;
    this.m_contactFactory = new b2ContactFactory(this.m_allocator);
    this.m_broadPhase = new b2DynamicTreeBroadPhase();
    return;
  }

  b2ContactManager.prototype.AddPair = function(proxyUserDataA, proxyUserDataB) {
    var bodyA, bodyB, c, edge, fA, fB, fixtureA, fixtureB;
    fixtureA = (proxyUserDataA instanceof b2Fixture ? proxyUserDataA : null);
    fixtureB = (proxyUserDataB instanceof b2Fixture ? proxyUserDataB : null);
    bodyA = fixtureA.GetBody();
    bodyB = fixtureB.GetBody();
    if (bodyA === bodyB) {
      return;
    }
    edge = bodyB.GetContactList();
    while (edge) {
      if (edge.other === bodyA) {
        fA = edge.contact.GetFixtureA();
        fB = edge.contact.GetFixtureB();
        if (fA === fixtureA && fB === fixtureB) {
          return;
        }
        if (fA === fixtureB && fB === fixtureA) {
          return;
        }
      }
      edge = edge.next;
    }
    if (bodyB.ShouldCollide(bodyA) === false) {
      return;
    }
    if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) === false) {
      return;
    }
    c = this.m_contactFactory.Create(fixtureA, fixtureB);
    fixtureA = c.GetFixtureA();
    fixtureB = c.GetFixtureB();
    bodyA = fixtureA.m_body;
    bodyB = fixtureB.m_body;
    c.m_prev = null;
    c.m_next = this.m_world.m_contactList;
    if (this.m_world.m_contactList != null) {
      this.m_world.m_contactList.m_prev = c;
    }
    this.m_world.m_contactList = c;
    c.m_nodeA.contact = c;
    c.m_nodeA.other = bodyB;
    c.m_nodeA.prev = null;
    c.m_nodeA.next = bodyA.m_contactList;
    if (bodyA.m_contactList != null) {
      bodyA.m_contactList.prev = c.m_nodeA;
    }
    bodyA.m_contactList = c.m_nodeA;
    c.m_nodeB.contact = c;
    c.m_nodeB.other = bodyA;
    c.m_nodeB.prev = null;
    c.m_nodeB.next = bodyB.m_contactList;
    if (bodyB.m_contactList != null) {
      bodyB.m_contactList.prev = c.m_nodeB;
    }
    bodyB.m_contactList = c.m_nodeB;
    ++this.m_world.m_contactCount;
  };

  b2ContactManager.prototype.FindNewContacts = function() {
    this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
  };

  b2ContactManager.prototype.Destroy = function(c) {
    var bodyA, bodyB, fixtureA, fixtureB;
    fixtureA = c.GetFixtureA();
    fixtureB = c.GetFixtureB();
    bodyA = fixtureA.GetBody();
    bodyB = fixtureB.GetBody();
    if (c.IsTouching()) {
      this.m_contactListener.EndContact(c);
    }
    if (c.m_prev) {
      c.m_prev.m_next = c.m_next;
    }
    if (c.m_next) {
      c.m_next.m_prev = c.m_prev;
    }
    if (c === this.m_world.m_contactList) {
      this.m_world.m_contactList = c.m_next;
    }
    if (c.m_nodeA.prev) {
      c.m_nodeA.prev.next = c.m_nodeA.next;
    }
    if (c.m_nodeA.next) {
      c.m_nodeA.next.prev = c.m_nodeA.prev;
    }
    if (c.m_nodeA === bodyA.m_contactList) {
      bodyA.m_contactList = c.m_nodeA.next;
    }
    if (c.m_nodeB.prev) {
      c.m_nodeB.prev.next = c.m_nodeB.next;
    }
    if (c.m_nodeB.next) {
      c.m_nodeB.next.prev = c.m_nodeB.prev;
    }
    if (c.m_nodeB === bodyB.m_contactList) {
      bodyB.m_contactList = c.m_nodeB.next;
    }
    this.m_contactFactory.Destroy(c);
    --this.m_contactCount;
  };

  b2ContactManager.prototype.Collide = function() {
    var bodyA, bodyB, c, cNuke, fixtureA, fixtureB, overlap, proxyA, proxyB;
    c = this.m_world.m_contactList;
    while (c) {
      fixtureA = c.GetFixtureA();
      fixtureB = c.GetFixtureB();
      bodyA = fixtureA.GetBody();
      bodyB = fixtureB.GetBody();
      if (bodyA.IsAwake() === false && bodyB.IsAwake() === false) {
        c = c.GetNext();
        continue;
      }
      if (c.m_flags & 0x0040) {
        if (bodyB.ShouldCollide(bodyA) === false) {
          cNuke = c;
          c = cNuke.GetNext();
          this.Destroy(cNuke);
          continue;
        }
        if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) === false) {
          cNuke = c;
          c = cNuke.GetNext();
          this.Destroy(cNuke);
          continue;
        }
        c.m_flags &= 0x0040;
      }
      proxyA = fixtureA.m_proxy;
      proxyB = fixtureB.m_proxy;
      overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
      if (overlap === false) {
        cNuke = c;
        c = cNuke.GetNext();
        this.Destroy(cNuke);
        continue;
      }
      c.Update(this.m_contactListener);
      c = c.GetNext();
    }
  };

  b2ContactManager.s_evalCP = new b2ContactPoint();

  return b2ContactManager;

})();

//# sourceMappingURL=b2ContactManager.js.map

},{"../index":103}],51:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2DebugDraw = (function() {
  b2DebugDraw.e_shapeBit = 0x0001;

  b2DebugDraw.e_jointBit = 0x0002;

  b2DebugDraw.e_aabbBit = 0x0004;

  b2DebugDraw.e_pairBit = 0x0008;

  b2DebugDraw.e_centerOfMassBit = 0x0010;

  b2DebugDraw.e_controllerBit = 0x0020;

  b2DebugDraw.prototype.m_drawScale = 1.0;

  b2DebugDraw.prototype.m_lineThickness = 1.0;

  b2DebugDraw.prototype.m_alpha = 1.0;

  b2DebugDraw.prototype.m_fillAlpha = 1.0;

  b2DebugDraw.prototype.m_xformScale = 1.0;

  b2DebugDraw.prototype.m_drawFlags = 0;

  b2DebugDraw.prototype.m_ctx = null;

  function b2DebugDraw() {
    this.m_sprite = {
      graphics: {
        clear: (function(_this) {
          return function() {
            _this.m_ctx.clearRect(0, 0, _this.m_ctx.canvas.width, _this.m_ctx.canvas.height);
          };
        })(this)
      }
    };
    return;
  }

  b2DebugDraw.prototype._color = function(color, alpha) {
    return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
  };

  b2DebugDraw.prototype.SetFlags = function(flags) {
    if (flags === undefined) {
      flags = 0;
    }
    this.m_drawFlags = flags;
  };

  b2DebugDraw.prototype.GetFlags = function() {
    return this.m_drawFlags;
  };

  b2DebugDraw.prototype.AppendFlags = function(flags) {
    if (flags === undefined) {
      flags = 0;
    }
    this.m_drawFlags |= flags;
  };

  b2DebugDraw.prototype.ClearFlags = function(flags) {
    if (flags === undefined) {
      flags = 0;
    }
    this.m_drawFlags &= ~flags;
  };

  b2DebugDraw.prototype.SetSprite = function(sprite) {
    this.m_ctx = sprite;
  };

  b2DebugDraw.prototype.GetSprite = function() {
    return this.m_ctx;
  };

  b2DebugDraw.prototype.SetDrawScale = function(drawScale) {
    if (drawScale === undefined) {
      drawScale = 0;
    }
    this.m_drawScale = drawScale;
  };

  b2DebugDraw.prototype.GetDrawScale = function() {
    return this.m_drawScale;
  };

  b2DebugDraw.prototype.SetLineThickness = function(lineThickness) {
    if (lineThickness === undefined) {
      lineThickness = 0;
    }
    this.m_lineThickness = lineThickness;
    this.m_ctx.strokeWidth = lineThickness;
  };

  b2DebugDraw.prototype.GetLineThickness = function() {
    return this.m_lineThickness;
  };

  b2DebugDraw.prototype.SetAlpha = function(alpha) {
    if (alpha === undefined) {
      alpha = 0;
    }
    this.m_alpha = alpha;
  };

  b2DebugDraw.prototype.GetAlpha = function() {
    return this.m_alpha;
  };

  b2DebugDraw.prototype.SetFillAlpha = function(alpha) {
    if (alpha === undefined) {
      alpha = 0;
    }
    this.m_fillAlpha = alpha;
  };

  b2DebugDraw.prototype.GetFillAlpha = function() {
    return this.m_fillAlpha;
  };

  b2DebugDraw.prototype.SetXFormScale = function(xformScale) {
    if (xformScale === undefined) {
      xformScale = 0;
    }
    this.m_xformScale = xformScale;
  };

  b2DebugDraw.prototype.GetXFormScale = function() {
    return this.m_xformScale;
  };

  b2DebugDraw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
    var drawScale, i, s;
    if (!vertexCount) {
      return;
    }
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    s.beginPath();
    s.strokeStyle = this._color(color.color, this.m_alpha);
    s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
    i = 1;
    while (i < vertexCount) {
      s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
      i++;
    }
    s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
    s.closePath();
    s.stroke();
  };

  b2DebugDraw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
    var drawScale, i, s;
    if (!vertexCount) {
      return;
    }
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    s.beginPath();
    s.strokeStyle = this._color(color.color, this.m_alpha);
    s.fillStyle = this._color(color.color, this.m_fillAlpha);
    s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
    i = 1;
    while (i < vertexCount) {
      s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
      i++;
    }
    s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
    s.closePath();
    s.fill();
    s.stroke();
  };

  b2DebugDraw.prototype.DrawCircle = function(center, radius, color) {
    var drawScale, s;
    if (!radius) {
      return;
    }
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    s.beginPath();
    s.strokeStyle = this._color(color.color, this.m_alpha);
    s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
    s.closePath();
    s.stroke();
  };

  b2DebugDraw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
    var cx, cy, drawScale, s;
    if (!radius) {
      return;
    }
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    cx = center.x * drawScale;
    cy = center.y * drawScale;
    s.moveTo(0, 0);
    s.beginPath();
    s.strokeStyle = this._color(color.color, this.m_alpha);
    s.fillStyle = this._color(color.color, this.m_fillAlpha);
    s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
    s.moveTo(cx, cy);
    s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
    s.closePath();
    s.fill();
    s.stroke();
  };

  b2DebugDraw.prototype.DrawSegment = function(p1, p2, color) {
    var drawScale, s;
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    s.strokeStyle = this._color(color.color, this.m_alpha);
    s.beginPath();
    s.moveTo(p1.x * drawScale, p1.y * drawScale);
    s.lineTo(p2.x * drawScale, p2.y * drawScale);
    s.closePath();
    s.stroke();
  };

  b2DebugDraw.prototype.DrawTransform = function(xf) {
    var drawScale, s;
    s = this.m_ctx;
    drawScale = this.m_drawScale;
    s.beginPath();
    s.strokeStyle = this._color(0xff0000, this.m_alpha);
    s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
    s.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale);
    s.strokeStyle = this._color(0xff00, this.m_alpha);
    s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
    s.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale);
    s.closePath();
    s.stroke();
  };

  return b2DebugDraw;

})();

//# sourceMappingURL=b2DebugDraw.js.map

},{"../index":103}],52:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2DestructionListener = (function() {
  function b2DestructionListener() {}

  b2DestructionListener.prototype.SayGoodbyeJoint = function(joint) {};

  b2DestructionListener.prototype.SayGoodbyeFixture = function(fixture) {};

  return b2DestructionListener;

})();

//# sourceMappingURL=b2DestructionListener.js.map

},{"../index":103}],53:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2FilterData = (function() {
  function b2FilterData() {}

  b2FilterData.prototype.categoryBits = 0x0001;

  b2FilterData.prototype.maskBits = 0xFFFF;

  b2FilterData.prototype.groupIndex = 0;

  b2FilterData.prototype.Copy = function() {
    var copy;
    copy = new b2FilterData();
    copy.categoryBits = this.categoryBits;
    copy.maskBits = this.maskBits;
    copy.groupIndex = this.groupIndex;
    return copy;
  };

  return b2FilterData;

})();

//# sourceMappingURL=b2FilterData.js.map

},{"../index":103}],54:[function(require,module,exports){
var Box2D, b2AABB, b2FilterData, b2MassData, b2Math, b2Vec2;

Box2D = require('../index');

b2Math = Box2D.Common.Math.b2Math;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2FilterData = Box2D.Dynamics.b2FilterData;

b2AABB = Box2D.Collision.b2AABB;

b2MassData = Box2D.Collision.Shapes.b2MassData;

Box2D.Dynamics.b2Fixture = (function() {
  b2Fixture.prototype.m_filter = null;

  b2Fixture.prototype.m_shape = null;

  b2Fixture.prototype.m_isSensor = false;

  b2Fixture.prototype.m_body = null;

  b2Fixture.prototype.m_next = null;

  b2Fixture.prototype.m_userData = null;

  b2Fixture.prototype.m_density = 0.0;

  b2Fixture.prototype.m_friction = 0.0;

  b2Fixture.prototype.m_restitution = 0.0;

  b2Fixture.prototype.m_aabb = null;

  b2Fixture.prototype.m_proxy = null;

  function b2Fixture() {
    this.m_filter = new b2FilterData();
    this.m_aabb = new b2AABB();
  }

  b2Fixture.prototype.GetType = function() {
    return this.m_shape.GetType();
  };

  b2Fixture.prototype.GetShape = function() {
    return this.m_shape;
  };

  b2Fixture.prototype.SetSensor = function(sensor) {
    var contact, edge, fixtureA, fixtureB;
    if (this.m_isSensor === sensor) {
      return;
    }
    this.m_isSensor = sensor;
    if (this.m_body == null) {
      return;
    }
    edge = this.m_body.GetContactList();
    while (edge) {
      contact = edge.contact;
      fixtureA = contact.GetFixtureA();
      fixtureB = contact.GetFixtureB();
      if (fixtureA === this || fixtureB === this) {
        contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
      }
      edge = edge.next;
    }
  };

  b2Fixture.prototype.IsSensor = function() {
    return this.m_isSensor;
  };

  b2Fixture.prototype.SetFilterData = function(filter) {
    var contact, edge, fixtureA, fixtureB;
    this.m_filter = filter.Copy();
    if (this.m_body) {
      return;
    }
    edge = this.m_body.GetContactList();
    while (edge) {
      contact = edge.contact;
      fixtureA = contact.GetFixtureA();
      fixtureB = contact.GetFixtureB();
      if (fixtureA === this || fixtureB === this) {
        contact.FlagForFiltering();
      }
      edge = edge.next;
    }
  };

  b2Fixture.prototype.GetFilterData = function() {
    return this.m_filter.Copy();
  };

  b2Fixture.prototype.GetBody = function() {
    return this.m_body;
  };

  b2Fixture.prototype.GetNext = function() {
    return this.m_next;
  };

  b2Fixture.prototype.GetUserData = function() {
    return this.m_userData;
  };

  b2Fixture.prototype.SetUserData = function(data) {
    return this.m_userData = data;
  };

  b2Fixture.prototype.TestPoint = function(p) {
    return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
  };

  b2Fixture.prototype.RayCast = function(output, input) {
    return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
  };

  b2Fixture.prototype.GetMassData = function(massData) {
    if (massData === void 0) {
      massData = null;
    }
    if (massData == null) {
      massData = new b2MassData();
    }
    this.m_shape.ComputeMass(massData, this.m_density);
    return massData;
  };

  b2Fixture.prototype.SetDensity = function(density) {
    if (density === void 0) {
      density = 0;
    }
    this.m_density = density;
  };

  b2Fixture.prototype.GetDensity = function() {
    return this.m_density;
  };

  b2Fixture.prototype.GetFriction = function() {
    return this.m_friction;
  };

  b2Fixture.prototype.SetFriction = function(friction) {
    if (friction === void 0) {
      friction = 0;
    }
    this.m_friction = friction;
  };

  b2Fixture.prototype.GetRestitution = function() {
    return this.m_restitution;
  };

  b2Fixture.prototype.SetRestitution = function(restitution) {
    if (restitution === void 0) {
      restitution = 0;
    }
    this.m_restitution = restitution;
  };

  b2Fixture.prototype.GetAABB = function() {
    return this.m_aabb;
  };

  b2Fixture.prototype.Create = function(body, xf, def) {
    this.m_userData = def.userData;
    this.m_friction = def.friction;
    this.m_restitution = def.restitution;
    this.m_body = body;
    this.m_next = null;
    this.m_filter = def.filter.Copy();
    this.m_isSensor = def.isSensor;
    this.m_shape = def.shape.Copy();
    this.m_density = def.density;
  };

  b2Fixture.prototype.Destroy = function() {
    this.m_shape = null;
  };

  b2Fixture.prototype.CreateProxy = function(broadPhase, xf) {
    this.m_shape.ComputeAABB(this.m_aabb, xf);
    this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
  };

  b2Fixture.prototype.DestroyProxy = function(broadPhase) {
    if (this.m_proxy == null) {
      return;
    }
    broadPhase.DestroyProxy(this.m_proxy);
    this.m_proxy = null;
  };

  b2Fixture.prototype.Synchronize = function(broadPhase, transform1, transform2) {
    var aabb1, aabb2, displacement;
    if (!this.m_proxy) {
      return;
    }
    aabb1 = new b2AABB();
    aabb2 = new b2AABB();
    this.m_shape.ComputeAABB(aabb1, transform1);
    this.m_shape.ComputeAABB(aabb2, transform2);
    this.m_aabb.Combine(aabb1, aabb2);
    displacement = b2Math.SubtractVV(transform2.position, transform1.position);
    broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
  };

  return b2Fixture;

})();

//# sourceMappingURL=b2Fixture.js.map

},{"../index":103}],55:[function(require,module,exports){
var Box2D, b2FilterData;

Box2D = require('../index');

b2FilterData = Box2D.Dynamics.b2FilterData;

Box2D.Dynamics.b2FixtureDef = (function() {
  b2FixtureDef.prototype.shape = null;

  b2FixtureDef.prototype.userData = null;

  b2FixtureDef.prototype.friction = 0.2;

  b2FixtureDef.prototype.restitution = 0.0;

  b2FixtureDef.prototype.density = 0.0;

  b2FixtureDef.prototype.isSensor = false;

  b2FixtureDef.prototype.filter = null;

  function b2FixtureDef() {
    this.filter = new b2FilterData();
    this.filter.categoryBits = 0x0001;
    this.filter.maskBits = 0xFFFF;
    this.filter.groupIndex = 0;
    return;
  }

  return b2FixtureDef;

})();

//# sourceMappingURL=b2FixtureDef.js.map

},{"../index":103}],56:[function(require,module,exports){
var Box2D, b2Body, b2ContactImpulse, b2Math, b2Settings;

Box2D = require('../index');

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse;

b2Body = Box2D.Dynamics.b2Body;

Box2D.Dynamics.b2Island = (function() {
  b2Island.prototype.m_bodies = null;

  b2Island.prototype.m_contacts = null;

  b2Island.prototype.m_joints = null;

  b2Island.prototype.m_bodyCapacity = 0;

  b2Island.prototype.m_contactCapacity = 0;

  b2Island.prototype.m_jointCapacity = 0;

  b2Island.prototype.m_bodyCount = 0;

  b2Island.prototype.m_contactCount = 0;

  b2Island.prototype.m_jointCount = 0;

  b2Island.prototype.m_allocator = null;

  b2Island.prototype.m_listener = null;

  b2Island.prototype.m_contactSolver = null;

  function b2Island() {
    this.m_bodies = new Array();
    this.m_contacts = new Array();
    this.m_joints = new Array();
    return;
  }

  b2Island.prototype.Initialize = function(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) {
    var i;
    if (bodyCapacity === void 0) {
      bodyCapacity = 0;
    }
    if (contactCapacity === void 0) {
      contactCapacity = 0;
    }
    if (jointCapacity === void 0) {
      jointCapacity = 0;
    }
    i = 0;
    this.m_bodyCapacity = bodyCapacity;
    this.m_contactCapacity = contactCapacity;
    this.m_jointCapacity = jointCapacity;
    this.m_bodyCount = 0;
    this.m_contactCount = 0;
    this.m_jointCount = 0;
    this.m_allocator = allocator;
    this.m_listener = listener;
    this.m_contactSolver = contactSolver;
    i = this.m_bodies.length;
    while (i < bodyCapacity) {
      this.m_bodies[i] = null;
      i++;
    }
    i = this.m_contacts.length;
    while (i < contactCapacity) {
      this.m_contacts[i] = null;
      i++;
    }
    i = this.m_joints.length;
    while (i < jointCapacity) {
      this.m_joints[i] = null;
      i++;
    }
  };

  b2Island.prototype.Clear = function() {
    this.m_bodyCount = 0;
    this.m_contactCount = 0;
    this.m_jointCount = 0;
  };

  b2Island.prototype.Solve = function(step, gravity, allowSleep) {
    var angTolSqr, b, contactSolver, contactsOkay, i, j, joint, jointOkay, jointsOkay, linTolSqr, minSleepTime, rotation, translationX, translationY;
    i = 0;
    j = 0;
    b = void 0;
    joint = void 0;
    i = 0;
    while (i < this.m_bodyCount) {
      b = this.m_bodies[i];
      if (b.GetType() !== b2Body.b2_dynamicBody) {
        continue;
      }
      b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
      b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
      b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
      b.m_linearVelocity.Multiply(b2Math.Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0));
      b.m_angularVelocity *= b2Math.Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0);
      ++i;
    }
    this.m_contactSolver.Initialize(step, this.m_contacts, this.m_contactCount, this.m_allocator);
    contactSolver = this.m_contactSolver;
    contactSolver.InitVelocityConstraints(step);
    i = 0;
    while (i < this.m_jointCount) {
      joint = this.m_joints[i];
      joint.InitVelocityConstraints(step);
      ++i;
    }
    i = 0;
    while (i < step.velocityIterations) {
      j = 0;
      while (j < this.m_jointCount) {
        joint = this.m_joints[j];
        joint.SolveVelocityConstraints(step);
        ++j;
      }
      contactSolver.SolveVelocityConstraints();
      ++i;
    }
    i = 0;
    while (i < this.m_jointCount) {
      joint = this.m_joints[i];
      joint.FinalizeVelocityConstraints();
      ++i;
    }
    contactSolver.FinalizeVelocityConstraints();
    i = 0;
    while (i < this.m_bodyCount) {
      b = this.m_bodies[i];
      if (b.GetType() === b2Body.b2_staticBody) {
        continue;
      }
      translationX = step.dt * b.m_linearVelocity.x;
      translationY = step.dt * b.m_linearVelocity.y;
      if ((translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared) {
        b.m_linearVelocity.Normalize();
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt;
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt;
      }
      rotation = step.dt * b.m_angularVelocity;
      if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
        if (b.m_angularVelocity < 0.0) {
          b.m_angularVelocity = -b2Settings.b2_maxRotation * step.inv_dt;
        } else {
          b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt;
        }
      }
      b.m_sweep.c0.SetV(b.m_sweep.c);
      b.m_sweep.a0 = b.m_sweep.a;
      b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
      b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
      b.m_sweep.a += step.dt * b.m_angularVelocity;
      b.SynchronizeTransform();
      ++i;
    }
    i = 0;
    while (i < step.positionIterations) {
      contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
      jointsOkay = true;
      j = 0;
      while (j < this.m_jointCount) {
        joint = this.m_joints[j];
        jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
        jointsOkay = jointsOkay && jointOkay;
        ++j;
      }
      if (contactsOkay && jointsOkay) {
        break;
      }
      ++i;
    }
    this.Report(contactSolver.m_constraints);
    if (allowSleep) {
      minSleepTime = Number.MAX_VALUE;
      linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
      angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;
      i = 0;
      while (i < this.m_bodyCount) {
        b = this.m_bodies[i];
        if (b.GetType() === b2Body.b2_staticBody) {
          continue;
        }
        if ((b.m_flags & b2Body.e_allowSleepFlag) === 0) {
          b.m_sleepTime = 0.0;
          minSleepTime = 0.0;
        }
        if ((b.m_flags & b2Body.e_allowSleepFlag) === 0 || b.m_angularVelocity * b.m_angularVelocity > angTolSqr || b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
          b.m_sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b.m_sleepTime += step.dt;
          minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
        }
        ++i;
      }
      if (minSleepTime >= b2Settings.b2_timeToSleep) {
        i = 0;
        while (i < this.m_bodyCount) {
          b = this.m_bodies[i];
          b.SetAwake(false);
          ++i;
        }
      }
    }
  };

  b2Island.prototype.SolveTOI = function(subStep) {
    var b, contactSolver, contactsOkay, i, j, jointOkay, jointsOkay, k_toiBaumgarte, rotation, translationX, translationY;
    i = 0;
    j = 0;
    this.m_contactSolver.Initialize(subStep, this.m_contacts, this.m_contactCount, this.m_allocator);
    contactSolver = this.m_contactSolver;
    i = 0;
    while (i < this.m_jointCount) {
      this.m_joints[i].InitVelocityConstraints(subStep);
      ++i;
    }
    i = 0;
    while (i < subStep.velocityIterations) {
      contactSolver.SolveVelocityConstraints();
      j = 0;
      while (j < this.m_jointCount) {
        this.m_joints[j].SolveVelocityConstraints(subStep);
        ++j;
      }
      ++i;
    }
    i = 0;
    while (i < this.m_bodyCount) {
      b = this.m_bodies[i];
      if (b.GetType() === b2Body.b2_staticBody) {
        continue;
      }
      translationX = subStep.dt * b.m_linearVelocity.x;
      translationY = subStep.dt * b.m_linearVelocity.y;
      if ((translationX * translationX + translationY * translationY) > b2Settings.b2_maxTranslationSquared) {
        b.m_linearVelocity.Normalize();
        b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt;
        b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt;
      }
      rotation = subStep.dt * b.m_angularVelocity;
      if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
        if (b.m_angularVelocity < 0.0) {
          b.m_angularVelocity = -b2Settings.b2_maxRotation * subStep.inv_dt;
        } else {
          b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt;
        }
      }
      b.m_sweep.c0.SetV(b.m_sweep.c);
      b.m_sweep.a0 = b.m_sweep.a;
      b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
      b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
      b.m_sweep.a += subStep.dt * b.m_angularVelocity;
      b.SynchronizeTransform();
      ++i;
    }
    k_toiBaumgarte = 0.75;
    i = 0;
    while (i < subStep.positionIterations) {
      contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
      jointsOkay = true;
      j = 0;
      while (j < this.m_jointCount) {
        jointOkay = this.m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
        jointsOkay = jointsOkay && jointOkay;
        ++j;
      }
      if (contactsOkay && jointsOkay) {
        break;
      }
      ++i;
    }
    this.Report(contactSolver.m_constraints);
  };

  b2Island.prototype.Report = function(constraints) {
    var c, cc, i, j;
    if (this.m_listener == null) {
      return;
    }
    i = 0;
    while (i < this.m_contactCount) {
      c = this.m_contacts[i];
      cc = constraints[i];
      j = 0;
      while (j < cc.pointCount) {
        b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
        b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
        ++j;
      }
      this.m_listener.PostSolve(c, b2Island.s_impulse);
      ++i;
    }
  };

  b2Island.prototype.AddBody = function(body) {
    body.m_islandIndex = this.m_bodyCount;
    this.m_bodies[this.m_bodyCount++] = body;
  };

  b2Island.prototype.AddContact = function(contact) {
    this.m_contacts[this.m_contactCount++] = contact;
  };

  b2Island.prototype.AddJoint = function(joint) {
    this.m_joints[this.m_jointCount++] = joint;
  };

  b2Island.s_impulse = new b2ContactImpulse();

  return b2Island;

})();

//# sourceMappingURL=b2Island.js.map

},{"../index":103}],57:[function(require,module,exports){
var Box2D;

Box2D = require('../index');

Box2D.Dynamics.b2TimeStep = (function() {
  function b2TimeStep() {}

  b2TimeStep.prototype.dt = 0;

  b2TimeStep.prototype.inv_dt = 0;

  b2TimeStep.prototype.positionIterations = 0;

  b2TimeStep.prototype.velocityIterations = 0;

  b2TimeStep.prototype.warmStarting = 0;

  b2TimeStep.prototype.Set = function(step) {
    this.dt = step.dt;
    this.inv_dt = step.inv_dt;
    this.positionIterations = step.positionIterations;
    this.velocityIterations = step.velocityIterations;
    this.warmStarting = step.warmStarting;
  };

  return b2TimeStep;

})();

//# sourceMappingURL=b2TimeStep.js.map

},{"../index":103}],58:[function(require,module,exports){
var Box2D, b2AABB, b2Body, b2BodyDef, b2Color, b2Contact, b2ContactManager, b2ContactSolver, b2DebugDraw, b2EdgeShape, b2Island, b2Joint, b2Math, b2PolygonShape, b2RayCastInput, b2RayCastOutput, b2Settings, b2Shape, b2Sweep, b2TimeStep, b2Transform, b2Vec2;

Box2D = require('../index');

b2AABB = Box2D.Collision.b2AABB;

b2Color = Box2D.Common.b2Color;

b2Settings = Box2D.Common.b2Settings;

b2Math = Box2D.Common.Math.b2Math;

b2Sweep = Box2D.Common.Math.b2Sweep;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Transform = Box2D.Common.Math.b2Transform;

b2Body = Box2D.Dynamics.b2Body;

b2BodyDef = Box2D.Dynamics.b2BodyDef;

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Contact = Box2D.Dynamics.b2Contact;

b2ContactManager = Box2D.Dynamics.b2ContactManager;

b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver;

b2Island = Box2D.Dynamics.b2Island;

b2TimeStep = Box2D.Dynamics.b2TimeStep;

b2DebugDraw = Box2D.Dynamics.b2DebugDraw;

b2RayCastInput = Box2D.Collision.b2RayCastInput;

b2RayCastOutput = Box2D.Collision.b2RayCastOutput;

b2Shape = Box2D.Collision.Shapes.b2Shape;

b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape;

Box2D.Dynamics.b2World = (function() {
  var b2Body_activeFlag, b2Body_allowSleepFlag, b2Body_awakeFlag, b2Body_bulletFlag, b2Body_dynamicBody, b2Body_fixedRotationFlag, b2Body_islandFlag, b2Body_kinematicBody, b2Body_staticBody, b2Contact_continuousFlag, b2Contact_enabledFlag, b2Contact_filterFlag, b2Contact_islandFlag, b2Contact_sensorFlag, b2Contact_toiFlag, b2Contact_touchingFlag;

  b2Contact_sensorFlag = 0x0001;

  b2Contact_continuousFlag = 0x0002;

  b2Contact_islandFlag = 0x0004;

  b2Contact_toiFlag = 0x0008;

  b2Contact_touchingFlag = 0x0010;

  b2Contact_enabledFlag = 0x0020;

  b2Contact_filterFlag = 0x0040;

  b2Body_islandFlag = 0x0001;

  b2Body_awakeFlag = 0x0002;

  b2Body_allowSleepFlag = 0x0004;

  b2Body_bulletFlag = 0x0008;

  b2Body_fixedRotationFlag = 0x0010;

  b2Body_activeFlag = 0x0020;

  b2Body_staticBody = 0;

  b2Body_kinematicBody = 1;

  b2Body_dynamicBody = 2;

  b2World.s_timestep2 = new b2TimeStep();

  b2World.s_xf = new b2Transform();

  b2World.s_backupA = new b2Sweep();

  b2World.s_backupB = new b2Sweep();

  b2World.s_timestep = new b2TimeStep();

  b2World.s_queue = new Array();

  b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);

  b2World.e_newFixture = 0x0001;

  b2World.e_locked = 0x0002;

  b2World.prototype.s_stack = null;

  b2World.prototype.m_contactManager = null;

  b2World.prototype.m_contactSolver = null;

  b2World.prototype.m_island = null;

  b2World.prototype.m_destructionListener = null;

  b2World.prototype.m_debugDraw = null;

  b2World.prototype.m_bodyList = null;

  b2World.prototype.m_contactList = null;

  b2World.prototype.m_jointList = null;

  b2World.prototype.m_controllerList = null;

  b2World.prototype.m_bodyCount = 0;

  b2World.prototype.m_contactCount = 0;

  b2World.prototype.m_jointCount = 0;

  b2World.prototype.m_controllerCount = 0;

  b2World.prototype.m_allowSleep = null;

  b2World.prototype.m_gravity = null;

  b2World.prototype.m_inv_dt0 = 0.0;

  b2World.prototype.m_groundBody = null;

  function b2World(gravity, doSleep) {
    var bd;
    this.s_stack = new Array();
    this.m_contactManager = new b2ContactManager();
    this.m_contactSolver = new b2ContactSolver();
    this.m_island = new b2Island();
    b2World.m_warmStarting = true;
    b2World.m_continuousPhysics = true;
    this.m_allowSleep = doSleep;
    this.m_gravity = gravity;
    this.m_contactManager.m_world = this;
    bd = new b2BodyDef();
    this.m_groundBody = this.CreateBody(bd);
    return;
  }

  b2World.prototype.SetDestructionListener = function(listener) {
    this.m_destructionListener = listener;
  };

  b2World.prototype.SetContactFilter = function(filter) {
    this.m_contactManager.m_contactFilter = filter;
  };

  b2World.prototype.SetContactListener = function(listener) {
    this.m_contactManager.m_contactListener = listener;
  };

  b2World.prototype.SetDebugDraw = function(debugDraw) {
    this.m_debugDraw = debugDraw;
  };

  b2World.prototype.SetBroadPhase = function(broadPhase) {
    var b, f, oldBroadPhase;
    oldBroadPhase = this.m_contactManager.m_broadPhase;
    this.m_contactManager.m_broadPhase = broadPhase;
    b = this.m_bodyList;
    while (b) {
      f = b.m_fixtureList;
      while (f) {
        f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
        f = f.m_next;
      }
      b = b.m_next;
    }
  };

  b2World.prototype.Validate = function() {
    this.m_contactManager.m_broadPhase.Validate();
  };

  b2World.prototype.GetProxyCount = function() {
    return this.m_contactManager.m_broadPhase.GetProxyCount();
  };

  b2World.prototype.CreateBody = function(def) {
    var b;
    if (this.IsLocked() === true) {
      return null;
    }
    b = new b2Body(def, this);
    b.m_prev = null;
    b.m_next = this.m_bodyList;
    if (this.m_bodyList) {
      this.m_bodyList.m_prev = b;
    }
    this.m_bodyList = b;
    ++this.m_bodyCount;
    return b;
  };

  b2World.prototype.DestroyBody = function(b) {
    var ce, ce0, coe, coe0, f, f0, jn, jn0;
    if (this.IsLocked() === true) {
      return;
    }
    jn = b.m_jointList;
    while (jn) {
      jn0 = jn;
      jn = jn.next;
      if (this.m_destructionListener) {
        this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
      }
      this.DestroyJoint(jn0.joint);
    }
    coe = b.m_controllerList;
    while (coe) {
      coe0 = coe;
      coe = coe.nextController;
      coe0.controller.RemoveBody(b);
    }
    ce = b.m_contactList;
    while (ce) {
      ce0 = ce;
      ce = ce.next;
      this.m_contactManager.Destroy(ce0.contact);
    }
    b.m_contactList = null;
    f = b.m_fixtureList;
    while (f) {
      f0 = f;
      f = f.m_next;
      if (this.m_destructionListener) {
        this.m_destructionListener.SayGoodbyeFixture(f0);
      }
      f0.DestroyProxy(this.m_contactManager.m_broadPhase);
      f0.Destroy();
    }
    b.m_fixtureList = null;
    b.m_fixtureCount = 0;
    if (b.m_prev) {
      b.m_prev.m_next = b.m_next;
    }
    if (b.m_next) {
      b.m_next.m_prev = b.m_prev;
    }
    if (b === this.m_bodyList) {
      this.m_bodyList = b.m_next;
    }
    --this.m_bodyCount;
  };

  b2World.prototype.CreateJoint = function(def) {
    var bodyA, bodyB, edge, j;
    j = b2Joint.Create(def, null);
    j.m_prev = null;
    j.m_next = this.m_jointList;
    if (this.m_jointList) {
      this.m_jointList.m_prev = j;
    }
    this.m_jointList = j;
    ++this.m_jointCount;
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.m_bodyB;
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.m_bodyA.m_jointList;
    if (j.m_bodyA.m_jointList) {
      j.m_bodyA.m_jointList.prev = j.m_edgeA;
    }
    j.m_bodyA.m_jointList = j.m_edgeA;
    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.m_bodyA;
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.m_bodyB.m_jointList;
    if (j.m_bodyB.m_jointList) {
      j.m_bodyB.m_jointList.prev = j.m_edgeB;
    }
    j.m_bodyB.m_jointList = j.m_edgeB;
    bodyA = def.bodyA;
    bodyB = def.bodyB;
    if (def.collideConnected === false) {
      edge = bodyB.GetContactList();
      while (edge) {
        if (edge.other === bodyA) {
          edge.contact.FlagForFiltering();
        }
        edge = edge.next;
      }
    }
    return j;
  };

  b2World.prototype.DestroyJoint = function(j) {
    var bodyA, bodyB, collideConnected, edge;
    collideConnected = j.m_collideConnected;
    if (j.m_prev) {
      j.m_prev.m_next = j.m_next;
    }
    if (j.m_next) {
      j.m_next.m_prev = j.m_prev;
    }
    if (j === this.m_jointList) {
      this.m_jointList = j.m_next;
    }
    bodyA = j.m_bodyA;
    bodyB = j.m_bodyB;
    bodyA.SetAwake(true);
    bodyB.SetAwake(true);
    if (j.m_edgeA.prev) {
      j.m_edgeA.prev.next = j.m_edgeA.next;
    }
    if (j.m_edgeA.next) {
      j.m_edgeA.next.prev = j.m_edgeA.prev;
    }
    if (j.m_edgeA === bodyA.m_jointList) {
      bodyA.m_jointList = j.m_edgeA.next;
    }
    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;
    if (j.m_edgeB.prev) {
      j.m_edgeB.prev.next = j.m_edgeB.next;
    }
    if (j.m_edgeB.next) {
      j.m_edgeB.next.prev = j.m_edgeB.prev;
    }
    if (j.m_edgeB === bodyB.m_jointList) {
      bodyB.m_jointList = j.m_edgeB.next;
    }
    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;
    b2Joint.Destroy(j, null);
    --this.m_jointCount;
    if (collideConnected === false) {
      edge = bodyB.GetContactList();
      while (edge) {
        if (edge.other === bodyA) {
          edge.contact.FlagForFiltering();
        }
        edge = edge.next;
      }
    }
  };

  b2World.prototype.AddController = function(c) {
    c.m_next = this.m_controllerList;
    c.m_prev = null;
    this.m_controllerList = c;
    c.m_world = this;
    this.m_controllerCount++;
    return c;
  };

  b2World.prototype.RemoveController = function(c) {
    if (c.m_prev) {
      c.m_prev.m_next = c.m_next;
    }
    if (c.m_next) {
      c.m_next.m_prev = c.m_prev;
    }
    if (this.m_controllerList === c) {
      this.m_controllerList = c.m_next;
    }
    this.m_controllerCount--;
  };

  b2World.prototype.CreateController = function(controller) {
    if (controller.m_world !== this) {
      throw new Error("Controller can only be a member of one world");
    }
    controller.m_next = this.m_controllerList;
    controller.m_prev = null;
    if (this.m_controllerList) {
      this.m_controllerList.m_prev = controller;
    }
    this.m_controllerList = controller;
    ++this.m_controllerCount;
    controller.m_world = this;
    return controller;
  };

  b2World.prototype.DestroyController = function(controller) {
    controller.Clear();
    if (controller.m_next) {
      controller.m_next.m_prev = controller.m_prev;
    }
    if (controller.m_prev) {
      controller.m_prev.m_next = controller.m_next;
    }
    if (controller === this.m_controllerList) {
      this.m_controllerList = controller.m_next;
    }
    --this.m_controllerCount;
  };

  b2World.prototype.SetWarmStarting = function(flag) {
    b2World.m_warmStarting = flag;
  };

  b2World.prototype.SetContinuousPhysics = function(flag) {
    b2World.m_continuousPhysics = flag;
  };

  b2World.prototype.GetBodyCount = function() {
    return this.m_bodyCount;
  };

  b2World.prototype.GetJointCount = function() {
    return this.m_jointCount;
  };

  b2World.prototype.GetContactCount = function() {
    return this.m_contactCount;
  };

  b2World.prototype.SetGravity = function(gravity) {
    this.m_gravity = gravity;
  };

  b2World.prototype.GetGravity = function() {
    return this.m_gravity;
  };

  b2World.prototype.GetGroundBody = function() {
    return this.m_groundBody;
  };

  b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
    var step;
    if (dt === void 0) {
      dt = 0;
    }
    if (velocityIterations === void 0) {
      velocityIterations = 0;
    }
    if (positionIterations === void 0) {
      positionIterations = 0;
    }
    if (this.m_flags & b2World.e_newFixture) {
      this.m_contactManager.FindNewContacts();
      this.m_flags &= ~b2World.e_newFixture;
    }
    this.m_flags |= b2World.e_locked;
    step = b2World.s_timestep2;
    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0) {
      step.inv_dt = 1.0 / dt;
    } else {
      step.inv_dt = 0.0;
    }
    step.dtRatio = this.m_inv_dt0 * dt;
    step.warmStarting = b2World.m_warmStarting;
    this.m_contactManager.Collide();
    if (step.dt > 0.0) {
      this.Solve(step);
    }
    if (b2World.m_continuousPhysics && step.dt > 0.0) {
      this.SolveTOI(step);
    }
    if (step.dt > 0.0) {
      this.m_inv_dt0 = step.inv_dt;
    }
    this.m_flags &= ~b2World.e_locked;
  };

  b2World.prototype.ClearForces = function() {
    var body;
    body = this.m_bodyList;
    while (body) {
      body.m_force.SetZero();
      body.m_torque = 0.0;
      body = body.m_next;
    }
  };

  b2World.prototype.DrawDebugData = function() {
    var aabb, b, b1, b2, bp, c, cA, cB, color, contact, f, fixtureA, fixtureB, flags, i, invQ, j, s, vs, x1, x2, xf;
    if (this.m_debugDraw == null) {
      return;
    }
    this.m_debugDraw.m_sprite.graphics.clear();
    flags = this.m_debugDraw.GetFlags();
    i = 0;
    b = void 0;
    f = void 0;
    s = void 0;
    j = void 0;
    bp = void 0;
    invQ = new b2Vec2;
    x1 = new b2Vec2;
    x2 = new b2Vec2;
    xf = void 0;
    b1 = new b2AABB();
    b2 = new b2AABB();
    vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
    color = new b2Color(0, 0, 0);
    if (flags & b2DebugDraw.e_shapeBit) {
      b = this.m_bodyList;
      while (b) {
        xf = b.m_xf;
        f = b.GetFixtureList();
        while (f) {
          s = f.GetShape();
          if (b.IsActive() === false) {
            color.Set(0.5, 0.5, 0.3);
            this.DrawShape(s, xf, color);
          } else if (b.GetType() === b2Body_staticBody) {
            color.Set(0.5, 0.9, 0.5);
            this.DrawShape(s, xf, color);
          } else if (b.GetType() === b2Body_kinematicBody) {
            color.Set(0.5, 0.5, 0.9);
            this.DrawShape(s, xf, color);
          } else if (b.IsAwake() === false) {
            color.Set(0.6, 0.6, 0.6);
            this.DrawShape(s, xf, color);
          } else {
            color.Set(0.9, 0.7, 0.7);
            this.DrawShape(s, xf, color);
          }
          f = f.m_next;
        }
        b = b.m_next;
      }
    }
    if (flags & b2DebugDraw.e_jointBit) {
      j = this.m_jointList;
      while (j) {
        this.DrawJoint(j);
        j = j.m_next;
      }
    }
    if (flags & b2DebugDraw.e_controllerBit) {
      c = this.m_controllerList;
      while (c) {
        c.Draw(this.m_debugDraw);
        c = c.m_next;
      }
    }
    if (flags & b2DebugDraw.e_pairBit) {
      color.Set(0.3, 0.9, 0.9);
      contact = this.m_contactManager.m_contactList;
      while (contact) {
        fixtureA = contact.GetFixtureA();
        fixtureB = contact.GetFixtureB();
        cA = fixtureA.GetAABB().GetCenter();
        cB = fixtureB.GetAABB().GetCenter();
        this.m_debugDraw.DrawSegment(cA, cB, color);
        contact = contact.GetNext();
      }
    }
    if (flags & b2DebugDraw.e_aabbBit) {
      bp = this.m_contactManager.m_broadPhase;
      vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
      b = this.m_bodyList;
      while (b) {
        if (b.IsActive() === false) {
          continue;
        }
        f = b.GetFixtureList();
        while (f) {
          aabb = bp.GetFatAABB(f.m_proxy);
          vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
          vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
          vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
          vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
          this.m_debugDraw.DrawPolygon(vs, 4, color);
          f = f.GetNext();
        }
        b = b.GetNext();
      }
    }
    if (flags & b2DebugDraw.e_centerOfMassBit) {
      b = this.m_bodyList;
      while (b) {
        xf = b2World.s_xf;
        xf.R = b.m_xf.R;
        xf.position = b.GetWorldCenter();
        this.m_debugDraw.DrawTransform(xf);
        b = b.m_next;
      }
    }
  };

  b2World.prototype.QueryAABB = function(callback, aabb) {
    var WorldQueryWrapper, broadPhase, __this;
    WorldQueryWrapper = function(proxy) {
      return callback(broadPhase.GetUserData(proxy));
    };
    __this = this;
    broadPhase = __this.m_contactManager.m_broadPhase;
    broadPhase.Query(WorldQueryWrapper, aabb);
  };

  b2World.prototype.QueryShape = function(callback, shape, transform) {
    var WorldQueryWrapper, aabb, broadPhase, __this;
    WorldQueryWrapper = function(proxy) {
      var fixture;
      fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
      if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) {
        return callback(fixture);
      }
      return true;
    };
    __this = this;
    if (transform === void 0) {
      transform = null;
    }
    if (transform == null) {
      transform = new b2Transform();
      transform.SetIdentity();
    }
    broadPhase = __this.m_contactManager.m_broadPhase;
    aabb = new b2AABB();
    shape.ComputeAABB(aabb, transform);
    broadPhase.Query(WorldQueryWrapper, aabb);
  };

  b2World.prototype.QueryPoint = function(callback, p) {
    var WorldQueryWrapper, aabb, broadPhase, __this;
    WorldQueryWrapper = function(proxy) {
      var fixture;
      fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
      if (fixture.TestPoint(p)) {
        return callback(fixture);
      }
      return true;
    };
    __this = this;
    broadPhase = __this.m_contactManager.m_broadPhase;
    aabb = new b2AABB();
    aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
    aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
    broadPhase.Query(WorldQueryWrapper, aabb);
  };

  b2World.prototype.RayCast = function(callback, point1, point2) {
    var RayCastWrapper, broadPhase, input, output, __this;
    RayCastWrapper = function(input, proxy) {
      var fixture, fraction, hit, point, userData;
      userData = broadPhase.GetUserData(proxy);
      fixture = (userData instanceof b2Fixture ? userData : null);
      hit = fixture.RayCast(output, input);
      if (hit) {
        fraction = output.fraction;
        point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y);
        return callback(fixture, point, output.normal, fraction);
      }
      return input.maxFraction;
    };
    __this = this;
    broadPhase = __this.m_contactManager.m_broadPhase;
    output = new b2RayCastOutput;
    input = new b2RayCastInput(point1, point2);
    broadPhase.RayCast(RayCastWrapper, input);
  };

  b2World.prototype.RayCastOne = function(point1, point2) {
    var RayCastOneWrapper, result, __this;
    RayCastOneWrapper = function(fixture, point, normal, fraction) {
      var result;
      if (fraction === void 0) {
        fraction = 0;
      }
      result = fixture;
      return fraction;
    };
    __this = this;
    result = void 0;
    __this.RayCast(RayCastOneWrapper, point1, point2);
    return result;
  };

  b2World.prototype.RayCastAll = function(point1, point2) {
    var RayCastAllWrapper, result, __this;
    RayCastAllWrapper = function(fixture, point, normal, fraction) {
      if (fraction === void 0) {
        fraction = 0;
      }
      result[result.length] = fixture;
      return 1;
    };
    __this = this;
    result = new Array();
    __this.RayCast(RayCastAllWrapper, point1, point2);
    return result;
  };

  b2World.prototype.GetBodyList = function() {
    return this.m_bodyList;
  };

  b2World.prototype.GetJointList = function() {
    return this.m_jointList;
  };

  b2World.prototype.GetContactList = function() {
    return this.m_contactList;
  };

  b2World.prototype.IsLocked = function() {
    return (this.m_flags & b2World.e_locked) > 0;
  };

  b2World.prototype.Solve = function(step) {
    var b, c, ce, controller, i, island, j, jn, other, seed, stack, stackCount, stackSize;
    b = void 0;
    controller = this.m_controllerList;
    while (controller) {
      controller.Step(step);
      controller = controller.m_next;
    }
    island = this.m_island;
    island.Initialize(this.m_bodyCount, this.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
    b = this.m_bodyList;
    while (b) {
      b.m_flags &= ~b2Body_islandFlag;
      b = b.m_next;
    }
    c = this.m_contactList;
    while (c) {
      c.m_flags &= ~b2Contact_islandFlag;
      c = c.m_next;
    }
    j = this.m_jointList;
    while (j) {
      j.m_islandFlag = false;
      j = j.m_next;
    }
    stackSize = parseInt(this.m_bodyCount);
    stack = this.s_stack;
    seed = this.m_bodyList;
    while (seed) {
      if (seed.m_flags & b2Body_islandFlag) {
        seed = seed.m_next;
        continue;
      }
      if (seed.IsAwake() === false || seed.IsActive() === false) {
        seed = seed.m_next;
        continue;
      }
      if (seed.GetType() === b2Body_staticBody) {
        seed = seed.m_next;
        continue;
      }
      island.Clear();
      stackCount = 0;
      stack[stackCount++] = seed;
      seed.m_flags |= b2Body_islandFlag;
      while (stackCount > 0) {
        b = stack[--stackCount];
        island.AddBody(b);
        if (b.IsAwake() === false) {
          b.SetAwake(true);
        }
        if (b.GetType() === b2Body_staticBody) {
          continue;
        }
        other = void 0;
        ce = b.m_contactList;
        while (ce) {
          if (ce.contact.m_flags & b2Contact_islandFlag) {
            ce = ce.next;
            continue;
          }
          if (ce.contact.IsSensor() === true || ce.contact.IsEnabled() === false || ce.contact.IsTouching() === false) {
            ce = ce.next;
            continue;
          }
          island.AddContact(ce.contact);
          ce.contact.m_flags |= b2Contact_islandFlag;
          other = ce.other;
          if (other.m_flags & b2Body_islandFlag) {
            ce = ce.next;
            continue;
          }
          stack[stackCount++] = other;
          other.m_flags |= b2Body_islandFlag;
          ce = ce.next;
        }
        jn = b.m_jointList;
        while (jn) {
          if (jn.joint.m_islandFlag === true) {
            jn = jn.next;
            continue;
          }
          other = jn.other;
          if (other.IsActive() === false) {
            jn = jn.next;
            continue;
          }
          island.AddJoint(jn.joint);
          jn.joint.m_islandFlag = true;
          if (other.m_flags & b2Body_islandFlag) {
            jn = jn.next;
            continue;
          }
          stack[stackCount++] = other;
          other.m_flags |= b2Body_islandFlag;
          jn = jn.next;
        }
      }
      island.Solve(step, this.m_gravity, this.m_allowSleep);
      i = 0;
      while (i < island.m_bodyCount) {
        b = island.m_bodies[i];
        if (b.GetType() === b2Body_staticBody) {
          b.m_flags &= ~b2Body_islandFlag;
        }
        ++i;
      }
      seed = seed.m_next;
    }
    i = 0;
    while (i < stack.length) {
      if (!stack[i]) {
        break;
      }
      stack[i] = null;
      ++i;
    }
    b = this.m_bodyList;
    while (b) {
      if (b.IsAwake() === false || b.IsActive() === false) {
        b = b.m_next;
        continue;
      }
      if (b.GetType() === b2Body_staticBody) {
        b = b.m_next;
        continue;
      }
      b.SynchronizeFixtures();
      b = b.m_next;
    }
    this.m_contactManager.FindNewContacts();
  };

  b2World.prototype.SolveTOI = function(step) {
    var b, bA, bB, c, cEdge, fA, fB, i, island, j, jEdge, minContact, minTOI, other, queue, queueSize, queueStart, seed, subStep, t0, toi;
    b = void 0;
    fA = void 0;
    fB = void 0;
    bA = void 0;
    bB = void 0;
    cEdge = void 0;
    j = void 0;
    island = this.m_island;
    island.Initialize(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
    queue = b2World.s_queue;
    b = this.m_bodyList;
    while (b) {
      b.m_flags &= ~b2Body_islandFlag;
      b.m_sweep.t0 = 0.0;
      b = b.m_next;
    }
    c = void 0;
    c = this.m_contactList;
    while (c) {
      c.m_flags &= ~(b2Contact_toiFlag | b2Contact_islandFlag);
      c = c.m_next;
    }
    j = this.m_jointList;
    while (j) {
      j.m_islandFlag = false;
      j = j.m_next;
    }
    while (true) {
      minContact = null;
      minTOI = 1.0;
      c = this.m_contactList;
      while (c) {
        if (c.IsSensor() === true || c.IsEnabled() === false || c.IsContinuous() === false) {
          continue;
        }
        toi = 1.0;
        if (c.m_flags & b2Contact_toiFlag) {
          toi = c.m_toi;
        } else {
          fA = c.m_fixtureA;
          fB = c.m_fixtureB;
          bA = fA.m_body;
          bB = fB.m_body;
          if ((bA.GetType() !== b2Body.b2_dynamicBody || bA.IsAwake() === false) && (bB.GetType() !== b2Body.b2_dynamicBody || bB.IsAwake() === false)) {
            continue;
          }
          t0 = bA.m_sweep.t0;
          if (bA.m_sweep.t0 < bB.m_sweep.t0) {
            t0 = bB.m_sweep.t0;
            bA.m_sweep.Advance(t0);
          } else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
            t0 = bA.m_sweep.t0;
            bB.m_sweep.Advance(t0);
          }
          toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
          b2Settings.b2Assert(0.0 <= toi && toi <= 1.0);
          if (toi > 0.0 && toi < 1.0) {
            toi = (1.0 - toi) * t0 + toi;
            if (toi > 1) {
              toi = 1;
            }
          }
          c.m_toi = toi;
          c.m_flags |= b2Contact_toiFlag;
        }
        if (Number.MIN_VALUE < toi && toi < minTOI) {
          minContact = c;
          minTOI = toi;
        }
        c = c.m_next;
      }
      if ((minContact == null) || 1.0 - 100.0 * Number.MIN_VALUE < minTOI) {
        break;
      }
      fA = minContact.m_fixtureA;
      fB = minContact.m_fixtureB;
      bA = fA.m_body;
      bB = fB.m_body;
      b2World.s_backupA.Set(bA.m_sweep);
      b2World.s_backupB.Set(bB.m_sweep);
      bA.Advance(minTOI);
      bB.Advance(minTOI);
      minContact.Update(this.m_contactManager.m_contactListener);
      minContact.m_flags &= ~b2Contact_toiFlag;
      if (minContact.IsSensor() === true || minContact.IsEnabled() === false) {
        bA.m_sweep.Set(b2World.s_backupA);
        bB.m_sweep.Set(b2World.s_backupB);
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
        continue;
      }
      if (minContact.IsTouching() === false) {
        continue;
      }
      seed = bA;
      if (seed.GetType() !== b2Body.b2_dynamicBody) {
        seed = bB;
      }
      island.Clear();
      queueStart = 0;
      queueSize = 0;
      queue[queueStart + queueSize++] = seed;
      seed.m_flags |= b2Body_islandFlag;
      while (queueSize > 0) {
        b = queue[queueStart++];
        --queueSize;
        island.AddBody(b);
        if (b.IsAwake() === false) {
          b.SetAwake(true);
        }
        if (b.GetType() !== b2Body.b2_dynamicBody) {
          continue;
        }
        cEdge = b.m_contactList;
        while (cEdge) {
          if (island.m_contactCount === island.m_contactCapacity) {
            break;
          }
          if (cEdge.contact.m_flags & b2Contact_islandFlag) {
            continue;
          }
          if (cEdge.contact.IsSensor() === true || cEdge.contact.IsEnabled() === false || cEdge.contact.IsTouching() === false) {
            continue;
          }
          island.AddContact(cEdge.contact);
          cEdge.contact.m_flags |= b2Contact_islandFlag;
          other = cEdge.other;
          if (other.m_flags & b2Body_islandFlag) {
            continue;
          }
          if (other.GetType() !== b2Body_staticBody) {
            other.Advance(minTOI);
            other.SetAwake(true);
          }
          queue[queueStart + queueSize] = other;
          ++queueSize;
          other.m_flags |= b2Body_islandFlag;
          cEdge = cEdge.next;
        }
        jEdge = b.m_jointList;
        while (jEdge) {
          if (island.m_jointCount === island.m_jointCapacity) {
            continue;
          }
          if (jEdge.joint.m_islandFlag === true) {
            continue;
          }
          other = jEdge.other;
          if (other.IsActive() === false) {
            continue;
          }
          island.AddJoint(jEdge.joint);
          jEdge.joint.m_islandFlag = true;
          if (other.m_flags & b2Body_islandFlag) {
            continue;
          }
          if (other.GetType() !== b2Body_staticBody) {
            other.Advance(minTOI);
            other.SetAwake(true);
          }
          queue[queueStart + queueSize] = other;
          ++queueSize;
          other.m_flags |= b2Body_islandFlag;
          jEdge = jEdge.next;
        }
      }
      subStep = b2World.s_timestep;
      subStep.warmStarting = false;
      subStep.dt = (1.0 - minTOI) * step.dt;
      subStep.inv_dt = 1.0 / subStep.dt;
      subStep.dtRatio = 0.0;
      subStep.velocityIterations = step.velocityIterations;
      subStep.positionIterations = step.positionIterations;
      island.SolveTOI(subStep);
      i = 0;
      i = 0;
      while (i < island.m_bodyCount) {
        b = island.m_bodies[i];
        b.m_flags &= ~b2Body_islandFlag;
        if (b.IsAwake() === false) {
          continue;
        }
        if (b.GetType() !== b2Body.b2_dynamicBody) {
          continue;
        }
        b.SynchronizeFixtures();
        cEdge = b.m_contactList;
        while (cEdge) {
          cEdge.contact.m_flags &= ~b2Contact_toiFlag;
          cEdge = cEdge.next;
        }
        ++i;
      }
      i = 0;
      while (i < island.m_contactCount) {
        c = island.m_contacts[i];
        c.m_flags &= ~(b2Contact_toiFlag | b2Contact_islandFlag);
        ++i;
      }
      i = 0;
      while (i < island.m_jointCount) {
        j = island.m_joints[i];
        j.m_islandFlag = false;
        ++i;
      }
      this.m_contactManager.FindNewContacts();
    }
  };

  b2World.prototype.DrawJoint = function(joint) {
    var b1, b2, color, p1, p2, pulley, s1, s2, x1, x2, xf1, xf2;
    b1 = joint.GetBodyA();
    b2 = joint.GetBodyB();
    xf1 = b1.m_xf;
    xf2 = b2.m_xf;
    x1 = xf1.position;
    x2 = xf2.position;
    p1 = joint.GetAnchorA();
    p2 = joint.GetAnchorB();
    color = b2World.s_jointColor;
    switch (joint.m_type) {
      case b2Joint.e_distanceJoint:
        this.m_debugDraw.DrawSegment(p1, p2, color);
        break;
      case b2Joint.e_pulleyJoint:
        pulley = (joint instanceof b2PulleyJoint ? joint : null);
        s1 = pulley.GetGroundAnchorA();
        s2 = pulley.GetGroundAnchorB();
        this.m_debugDraw.DrawSegment(s1, p1, color);
        this.m_debugDraw.DrawSegment(s2, p2, color);
        this.m_debugDraw.DrawSegment(s1, s2, color);
        break;
      case b2Joint.e_mouseJoint:
        this.m_debugDraw.DrawSegment(p1, p2, color);
        break;
      default:
        if (b1 !== this.m_groundBody) {
          this.m_debugDraw.DrawSegment(x1, p1, color);
        }
        this.m_debugDraw.DrawSegment(p1, p2, color);
        if (b2 !== this.m_groundBody) {
          this.m_debugDraw.DrawSegment(x2, p2, color);
        }
    }
  };

  b2World.prototype.DrawShape = function(shape, xf, color) {
    var axis, center, circle, edge, i, localVertices, poly, radius, vertexCount, vertices;
    switch (shape.m_type) {
      case b2Shape.e_circleShape:
        circle = (shape instanceof b2CircleShape ? shape : null);
        center = b2Math.MulX(xf, circle.m_p);
        radius = circle.m_radius;
        axis = xf.R.col1;
        return this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
      case b2Shape.e_polygonShape:
        i = 0;
        poly = (shape instanceof b2PolygonShape ? shape : null);
        vertexCount = parseInt(poly.GetVertexCount());
        localVertices = poly.GetVertices();
        vertices = new Array(vertexCount);
        i = 0;
        while (i < vertexCount) {
          vertices[i] = b2Math.MulX(xf, localVertices[i]);
          ++i;
        }
        return this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
      case b2Shape.e_edgeShape:
        edge = (shape instanceof b2EdgeShape ? shape : null);
        return this.m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);
    }
  };

  return b2World;

})();

//# sourceMappingURL=b2World.js.map

},{"../index":103}],59:[function(require,module,exports){
var Box2D, b2CircleShape, b2Collision, b2Contact,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

b2Collision = Box2D.Collision.b2Collision;

b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;

Box2D.Dynamics.Contacts.b2CircleContact = (function(_super) {
  __extends(b2CircleContact, _super);

  function b2CircleContact() {
    return b2CircleContact.__super__.constructor.apply(this, arguments);
  }

  b2CircleContact.Create = function(allocator) {
    return new b2CircleContact();
  };

  b2CircleContact.Destroy = function(contact, allocator) {};

  b2CircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    b2CircleContact.__super__.Reset.call(this, fixtureA, fixtureB);
  };

  b2CircleContact.prototype.Evaluate = function() {
    var bA, bB;
    bA = this.m_fixtureA.GetBody();
    bB = this.m_fixtureB.GetBody();
    b2Collision.CollideCircles(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2CircleShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
  };

  return b2CircleContact;

})(b2Contact);

//# sourceMappingURL=b2CircleContact.js.map

},{"../../index":103}],60:[function(require,module,exports){
var Box2D, b2ContactEdge, b2Manifold, b2Settings, b2TOIInput, b2TimeOfImpact;

Box2D = require('../../index');

b2Settings = Box2D.Common.b2Settings;

b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge;

b2Manifold = Box2D.Collision.b2Manifold;

b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact;

b2TOIInput = Box2D.Collision.b2TOIInput;

Box2D.Dynamics.Contacts.b2Contact = (function() {
  b2Contact.e_sensorFlag = 0x0001;

  b2Contact.e_continuousFlag = 0x0002;

  b2Contact.e_islandFlag = 0x0004;

  b2Contact.e_toiFlag = 0x0008;

  b2Contact.e_touchingFlag = 0x0010;

  b2Contact.e_enabledFlag = 0x0020;

  b2Contact.e_filterFlag = 0x0040;

  b2Contact.s_input = new b2TOIInput();

  b2Contact.prototype.m_nodeA = null;

  b2Contact.prototype.m_nodeB = null;

  b2Contact.prototype.m_manifold = null;

  b2Contact.prototype.m_oldManifold = null;

  b2Contact.prototype.m_fixtureA = null;

  b2Contact.prototype.m_fixtureB = null;

  b2Contact.prototype.m_touching = false;

  function b2Contact() {
    this.m_nodeA = new b2ContactEdge();
    this.m_nodeB = new b2ContactEdge();
    this.m_manifold = new b2Manifold();
    this.m_oldManifold = new b2Manifold();
    return;
  }

  b2Contact.prototype.GetManifold = function() {
    return this.m_manifold;
  };

  b2Contact.prototype.GetWorldManifold = function(worldManifold) {
    var bodyA, bodyB, shapeA, shapeB;
    bodyA = this.m_fixtureA.GetBody();
    bodyB = this.m_fixtureB.GetBody();
    shapeA = this.m_fixtureA.GetShape();
    shapeB = this.m_fixtureB.GetShape();
    worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
  };

  b2Contact.prototype.IsTouching = function() {
    return (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag;
  };

  b2Contact.prototype.IsContinuous = function() {
    return (this.m_flags & b2Contact.e_continuousFlag) === b2Contact.e_continuousFlag;
  };

  b2Contact.prototype.SetSensor = function(sensor) {
    if (sensor) {
      this.m_flags |= b2Contact.e_sensorFlag;
    } else {
      this.m_flags &= ~b2Contact.e_sensorFlag;
    }
  };

  b2Contact.prototype.IsSensor = function() {
    return (this.m_flags & b2Contact.e_sensorFlag) === b2Contact.e_sensorFlag;
  };

  b2Contact.prototype.SetEnabled = function(flag) {
    if (flag) {
      this.m_flags |= b2Contact.e_enabledFlag;
    } else {
      this.m_flags &= ~b2Contact.e_enabledFlag;
    }
  };

  b2Contact.prototype.IsEnabled = function() {
    return (this.m_flags & b2Contact.e_enabledFlag) === b2Contact.e_enabledFlag;
  };

  b2Contact.prototype.GetNext = function() {
    return this.m_next;
  };

  b2Contact.prototype.GetFixtureA = function() {
    return this.m_fixtureA;
  };

  b2Contact.prototype.GetFixtureB = function() {
    return this.m_fixtureB;
  };

  b2Contact.prototype.FlagForFiltering = function() {
    this.m_flags |= b2Contact.e_filterFlag;
  };

  b2Contact.prototype.Reset = function(fixtureA, fixtureB) {
    var bodyA, bodyB;
    if (fixtureA === void 0) {
      fixtureA = null;
    }
    if (fixtureB === void 0) {
      fixtureB = null;
    }
    this.m_flags = b2Contact.e_enabledFlag;
    if (!fixtureA || !fixtureB) {
      this.m_fixtureA = null;
      this.m_fixtureB = null;
      return;
    }
    if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
      this.m_flags |= b2Contact.e_sensorFlag;
    }
    bodyA = fixtureA.GetBody();
    bodyB = fixtureB.GetBody();
    if (bodyA.GetType() !== Box2D.Dynamics.b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() !== Box2D.Dynamics.b2Body.b2_dynamicBody || bodyB.IsBullet()) {
      this.m_flags |= b2Contact.e_continuousFlag;
    }
    this.m_fixtureA = fixtureA;
    this.m_fixtureB = fixtureB;
    this.m_manifold.m_pointCount = 0;
    this.m_prev = null;
    this.m_next = null;
    this.m_nodeA.contact = null;
    this.m_nodeA.prev = null;
    this.m_nodeA.next = null;
    this.m_nodeA.other = null;
    this.m_nodeB.contact = null;
    this.m_nodeB.prev = null;
    this.m_nodeB.next = null;
    this.m_nodeB.other = null;
  };

  b2Contact.prototype.Update = function(listener) {
    var aabbOverlap, bodyA, bodyB, i, id2, j, mp1, mp2, shapeA, shapeB, tManifold, touching, wasTouching, xfA, xfB;
    tManifold = this.m_oldManifold;
    this.m_oldManifold = this.m_manifold;
    this.m_manifold = tManifold;
    this.m_flags |= b2Contact.e_enabledFlag;
    touching = false;
    wasTouching = (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag;
    bodyA = this.m_fixtureA.m_body;
    bodyB = this.m_fixtureB.m_body;
    aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
    if (this.m_flags & b2Contact.e_sensorFlag) {
      if (aabbOverlap) {
        shapeA = this.m_fixtureA.GetShape();
        shapeB = this.m_fixtureB.GetShape();
        xfA = bodyA.GetTransform();
        xfB = bodyB.GetTransform();
        touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
      }
      this.m_manifold.m_pointCount = 0;
    } else {
      if (bodyA.GetType() !== Box2D.Dynamics.b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() !== Box2D.Dynamics.b2Body.b2_dynamicBody || bodyB.IsBullet()) {
        this.m_flags |= b2Contact.e_continuousFlag;
      } else {
        this.m_flags &= ~b2Contact.e_continuousFlag;
      }
      if (aabbOverlap) {
        this.Evaluate();
        touching = this.m_manifold.m_pointCount > 0;
        i = 0;
        while (i < this.m_manifold.m_pointCount) {
          mp2 = this.m_manifold.m_points[i];
          mp2.m_normalImpulse = 0.0;
          mp2.m_tangentImpulse = 0.0;
          id2 = mp2.m_id;
          j = 0;
          while (j < this.m_oldManifold.m_pointCount) {
            mp1 = this.m_oldManifold.m_points[j];
            if (mp1.m_id.key === id2.key) {
              mp2.m_normalImpulse = mp1.m_normalImpulse;
              mp2.m_tangentImpulse = mp1.m_tangentImpulse;
              break;
            }
            ++j;
          }
          ++i;
        }
      } else {
        this.m_manifold.m_pointCount = 0;
      }
      if (touching !== wasTouching) {
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
      }
    }
    if (touching) {
      this.m_flags |= b2Contact.e_touchingFlag;
    } else {
      this.m_flags &= ~b2Contact.e_touchingFlag;
    }
    if (wasTouching === false && touching === true) {
      listener.BeginContact(this);
    }
    if (wasTouching === true && touching === false) {
      listener.EndContact(this);
    }
    if ((this.m_flags & b2Contact.e_sensorFlag) === 0) {
      listener.PreSolve(this, this.m_oldManifold);
    }
  };

  b2Contact.prototype.Evaluate = function() {};

  b2Contact.prototype.ComputeTOI = function(sweepA, sweepB) {
    b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
    b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
    b2Contact.s_input.sweepA = sweepA;
    b2Contact.s_input.sweepB = sweepB;
    b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
    return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
  };

  return b2Contact;

})();

//# sourceMappingURL=b2Contact.js.map

},{"../../index":103}],61:[function(require,module,exports){
var Box2D, b2ContactConstraintPoint, b2Mat22, b2Settings, b2Vec2;

Box2D = require('../../index');

b2Settings = Box2D.Common.b2Settings;

b2ContactConstraintPoint = Box2D.Dynamics.Contacts.b2ContactConstraintPoint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

Box2D.Dynamics.Contacts.b2ContactConstraint = (function() {
  b2ContactConstraint.prototype.localPlaneNormal = null;

  b2ContactConstraint.prototype.localPoint = null;

  b2ContactConstraint.prototype.normal = null;

  b2ContactConstraint.prototype.normalMass = null;

  b2ContactConstraint.prototype.K = null;

  b2ContactConstraint.prototype.points = null;

  function b2ContactConstraint() {
    var i;
    this.localPlaneNormal = new b2Vec2();
    this.localPoint = new b2Vec2();
    this.normal = new b2Vec2();
    this.normalMass = new b2Mat22();
    this.K = new b2Mat22();
    this.points = new Array(b2Settings.b2_maxManifoldPoints);
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      this.points[i] = new b2ContactConstraintPoint();
      i++;
    }
    return;
  }

  return b2ContactConstraint;

})();

//# sourceMappingURL=b2ContactConstraint.js.map

},{"../../index":103}],62:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Contacts.b2ContactConstraintPoint = (function() {
  b2ContactConstraintPoint.prototype.localPoint = null;

  b2ContactConstraintPoint.prototype.rA = null;

  b2ContactConstraintPoint.prototype.rB = null;

  function b2ContactConstraintPoint() {
    this.localPoint = new b2Vec2();
    this.rA = new b2Vec2();
    this.rB = new b2Vec2();
    return;
  }

  return b2ContactConstraintPoint;

})();

//# sourceMappingURL=b2ContactConstraintPoint.js.map

},{"../../index":103}],63:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.Contacts.b2ContactEdge = (function() {
  function b2ContactEdge() {}

  return b2ContactEdge;

})();

//# sourceMappingURL=b2ContactEdge.js.map

},{"../../index":103}],64:[function(require,module,exports){
var Box2D, b2CircleContact, b2ContactRegister, b2EdgeAndCircleContact, b2PolyAndCircleContact, b2PolyAndEdgeContact, b2PolygonContact, b2Shape;

Box2D = require('../../index');

b2Shape = Box2D.Collision.Shapes.b2Shape;

b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister;

b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact;

b2PolyAndCircleContact = Box2D.Dynamics.Contacts.b2PolyAndCircleContact;

b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact;

b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact;

b2PolyAndEdgeContact = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact;

Box2D.Dynamics.Contacts.b2ContactFactory = (function() {
  b2ContactFactory.prototype.m_allocator = null;

  b2ContactFactory.prototype.m_registers = null;

  function b2ContactFactory(allocator) {
    this.m_allocator = allocator;
    this.InitializeRegisters();
    return;
  }

  b2ContactFactory.prototype.AddType = function(createFcn, destroyFcn, type1, type2) {
    if (type1 === void 0) {
      type1 = 0;
    }
    if (type2 === void 0) {
      type2 = 0;
    }
    this.m_registers[type1][type2].createFcn = createFcn;
    this.m_registers[type1][type2].destroyFcn = destroyFcn;
    this.m_registers[type1][type2].primary = true;
    if (type1 !== type2) {
      this.m_registers[type2][type1].createFcn = createFcn;
      this.m_registers[type2][type1].destroyFcn = destroyFcn;
      this.m_registers[type2][type1].primary = false;
    }
  };

  b2ContactFactory.prototype.InitializeRegisters = function() {
    var i, j;
    this.m_registers = new Array(b2Shape.e_shapeTypeCount);
    i = 0;
    while (i < b2Shape.e_shapeTypeCount) {
      this.m_registers[i] = new Array(b2Shape.e_shapeTypeCount);
      j = 0;
      while (j < b2Shape.e_shapeTypeCount) {
        this.m_registers[i][j] = new b2ContactRegister();
        j++;
      }
      i++;
    }
    this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
    this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
    this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
    this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
    this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
  };

  b2ContactFactory.prototype.Create = function(fixtureA, fixtureB) {
    var c, createFcn, reg, type1, type2;
    type1 = parseInt(fixtureA.GetType());
    type2 = parseInt(fixtureB.GetType());
    reg = this.m_registers[type1][type2];
    c = void 0;
    if (reg.pool) {
      c = reg.pool;
      reg.pool = c.m_next;
      reg.poolCount--;
      c.Reset(fixtureA, fixtureB);
      return c;
    }
    createFcn = reg.createFcn;
    if (createFcn != null) {
      if (reg.primary) {
        c = createFcn(this.m_allocator);
        c.Reset(fixtureA, fixtureB);
        return c;
      } else {
        c = createFcn(this.m_allocator);
        c.Reset(fixtureB, fixtureA);
        return c;
      }
    } else {
      return null;
    }
  };

  b2ContactFactory.prototype.Destroy = function(contact) {
    var destroyFcn, reg, type1, type2;
    if (contact.m_manifold.m_pointCount > 0) {
      contact.m_fixtureA.m_body.SetAwake(true);
      contact.m_fixtureB.m_body.SetAwake(true);
    }
    type1 = parseInt(contact.m_fixtureA.GetType());
    type2 = parseInt(contact.m_fixtureB.GetType());
    reg = this.m_registers[type1][type2];
    if (true) {
      reg.poolCount++;
      contact.m_next = reg.pool;
      reg.pool = contact;
    }
    destroyFcn = reg.destroyFcn;
    destroyFcn(contact, this.m_allocator);
  };

  return b2ContactFactory;

})();

//# sourceMappingURL=b2ContactFactory.js.map

},{"../../index":103}],65:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.Contacts.b2ContactRegister = (function() {
  function b2ContactRegister() {}

  return b2ContactRegister;

})();

//# sourceMappingURL=b2ContactRegister.js.map

},{"../../index":103}],66:[function(require,module,exports){
var Box2D, b2ContactID, b2Vec2;

Box2D = require('../../index');

b2ContactID = Box2D.Collision.b2ContactID;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Contacts.b2ContactResult = (function() {
  b2ContactResult.prototype.position = null;

  b2ContactResult.prototype.normal = null;

  b2ContactResult.prototype.id = null;

  function b2ContactResult() {
    this.position = new b2Vec2();
    this.normal = new b2Vec2();
    this.id = new b2ContactID();
    return;
  }

  return b2ContactResult;

})();

//# sourceMappingURL=b2ContactResult.js.map

},{"../../index":103}],67:[function(require,module,exports){
var Box2D, b2ContactConstraint, b2Math, b2PositionSolverManifold, b2Settings, b2TimeStep, b2WorldManifold;

Box2D = require('../../index');

b2Math = Box2D.Common.Math.b2Math;

b2Settings = Box2D.Common.b2Settings;

b2TimeStep = Box2D.Dynamics.b2TimeStep;

b2ContactConstraint = Box2D.Dynamics.Contacts.b2ContactConstraint;

b2WorldManifold = Box2D.Collision.b2WorldManifold;

b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold;

Box2D.Dynamics.Contacts.b2ContactSolver = (function() {
  b2ContactSolver.s_worldManifold = new b2WorldManifold();

  b2ContactSolver.s_psm = new b2PositionSolverManifold();

  b2ContactSolver.prototype.m_step = null;

  b2ContactSolver.prototype.m_constraints = null;

  b2ContactSolver.prototype.m_allocator = null;

  b2ContactSolver.prototype.m_constraintCount = 0;

  function b2ContactSolver() {
    this.m_step = new b2TimeStep();
    this.m_constraints = new Array();
    return;
  }

  b2ContactSolver.prototype.Initialize = function(step, contacts, contactCount, allocator) {
    var bodyA, bodyB, cc, ccp, ccp1, ccp2, contact, cp, fixtureA, fixtureB, friction, i, invIA, invIB, invMassA, invMassB, k, k11, k12, k22, kEqualized, kNormal, kTangent, k_maxConditionNumber, manifold, normalX, normalY, rAX, rAY, rBX, rBY, radiusA, radiusB, restitution, rn1A, rn1B, rn2A, rn2B, rnA, rnB, rtA, rtB, shapeA, shapeB, tMat, tVec, tX, tY, tangentX, tangentY, vAX, vAY, vBX, vBY, vRel, wA, wB;
    if (contactCount === void 0) {
      contactCount = 0;
    }
    contact = void 0;
    this.m_step.Set(step);
    this.m_allocator = allocator;
    i = 0;
    tVec = void 0;
    tMat = void 0;
    this.m_constraintCount = contactCount;
    while (this.m_constraints.length < this.m_constraintCount) {
      this.m_constraints[this.m_constraints.length] = new b2ContactConstraint();
    }
    i = 0;
    while (i < contactCount) {
      contact = contacts[i];
      fixtureA = contact.m_fixtureA;
      fixtureB = contact.m_fixtureB;
      shapeA = fixtureA.m_shape;
      shapeB = fixtureB.m_shape;
      radiusA = shapeA.m_radius;
      radiusB = shapeB.m_radius;
      bodyA = fixtureA.m_body;
      bodyB = fixtureB.m_body;
      manifold = contact.GetManifold();
      friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
      restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
      vAX = bodyA.m_linearVelocity.x;
      vAY = bodyA.m_linearVelocity.y;
      vBX = bodyB.m_linearVelocity.x;
      vBY = bodyB.m_linearVelocity.y;
      wA = bodyA.m_angularVelocity;
      wB = bodyB.m_angularVelocity;
      b2Settings.b2Assert(manifold.m_pointCount > 0);
      b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
      normalX = b2ContactSolver.s_worldManifold.m_normal.x;
      normalY = b2ContactSolver.s_worldManifold.m_normal.y;
      cc = this.m_constraints[i];
      cc.bodyA = bodyA;
      cc.bodyB = bodyB;
      cc.manifold = manifold;
      cc.normal.x = normalX;
      cc.normal.y = normalY;
      cc.pointCount = manifold.m_pointCount;
      cc.friction = friction;
      cc.restitution = restitution;
      cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
      cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
      cc.localPoint.x = manifold.m_localPoint.x;
      cc.localPoint.y = manifold.m_localPoint.y;
      cc.radius = radiusA + radiusB;
      cc.type = manifold.m_type;
      k = 0;
      while (k < cc.pointCount) {
        cp = manifold.m_points[k];
        ccp = cc.points[k];
        ccp.normalImpulse = cp.m_normalImpulse;
        ccp.tangentImpulse = cp.m_tangentImpulse;
        ccp.localPoint.SetV(cp.m_localPoint);
        rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
        rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
        rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
        rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
        rnA = rAX * normalY - rAY * normalX;
        rnB = rBX * normalY - rBY * normalX;
        rnA *= rnA;
        rnB *= rnB;
        kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
        ccp.normalMass = 1.0 / kNormal;
        kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
        kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
        ccp.equalizedMass = 1.0 / kEqualized;
        tangentX = normalY;
        tangentY = -normalX;
        rtA = rAX * tangentY - rAY * tangentX;
        rtB = rBX * tangentY - rBY * tangentX;
        rtA *= rtA;
        rtB *= rtB;
        kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
        ccp.tangentMass = 1.0 / kTangent;
        ccp.velocityBias = 0.0;
        tX = vBX + (-wB * rBY) - vAX - (-wA * rAY);
        tY = vBY + (wB * rBX) - vAY - (wA * rAX);
        vRel = cc.normal.x * tX + cc.normal.y * tY;
        if (vRel < (-b2Settings.b2_velocityThreshold)) {
          ccp.velocityBias += -cc.restitution * vRel;
        }
        ++k;
      }
      if (cc.pointCount === 2) {
        ccp1 = cc.points[0];
        ccp2 = cc.points[1];
        invMassA = bodyA.m_invMass;
        invIA = bodyA.m_invI;
        invMassB = bodyB.m_invMass;
        invIB = bodyB.m_invI;
        rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
        rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
        rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
        rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
        k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
        k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
        k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
        k_maxConditionNumber = 100.0;
        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
          cc.K.col1.Set(k11, k12);
          cc.K.col2.Set(k12, k22);
          cc.K.GetInverse(cc.normalMass);
        } else {
          cc.pointCount = 1;
        }
      }
      ++i;
    }
  };

  b2ContactSolver.prototype.InitVelocityConstraints = function(step) {
    var PX, PY, bodyA, bodyB, c, ccp, ccp2, i, invIA, invIB, invMassA, invMassB, j, normalX, normalY, tCount, tMat, tVec, tVec2, tX, tangentX, tangentY;
    tVec = void 0;
    tVec2 = void 0;
    tMat = void 0;
    i = 0;
    while (i < this.m_constraintCount) {
      c = this.m_constraints[i];
      bodyA = c.bodyA;
      bodyB = c.bodyB;
      invMassA = bodyA.m_invMass;
      invIA = bodyA.m_invI;
      invMassB = bodyB.m_invMass;
      invIB = bodyB.m_invI;
      normalX = c.normal.x;
      normalY = c.normal.y;
      tangentX = normalY;
      tangentY = -normalX;
      tX = 0;
      j = 0;
      tCount = 0;
      if (step.warmStarting) {
        tCount = c.pointCount;
        j = 0;
        while (j < tCount) {
          ccp = c.points[j];
          ccp.normalImpulse *= step.dtRatio;
          ccp.tangentImpulse *= step.dtRatio;
          PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
          PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
          bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
          bodyA.m_linearVelocity.x -= invMassA * PX;
          bodyA.m_linearVelocity.y -= invMassA * PY;
          bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
          bodyB.m_linearVelocity.x += invMassB * PX;
          bodyB.m_linearVelocity.y += invMassB * PY;
          ++j;
        }
      } else {
        tCount = c.pointCount;
        j = 0;
        while (j < tCount) {
          ccp2 = c.points[j];
          ccp2.normalImpulse = 0.0;
          ccp2.tangentImpulse = 0.0;
          ++j;
        }
      }
      ++i;
    }
  };

  b2ContactSolver.prototype.SolveVelocityConstraints = function() {
    var P1X, P1Y, P2X, P2Y, PX, PY, aX, aY, bX, bY, bodyA, bodyB, c, ccp, cp1, cp2, dX, dY, dv1X, dv1Y, dv2X, dv2Y, dvX, dvY, friction, i, invIA, invIB, invMassA, invMassB, j, k_errorTol, lambda, maxFriction, newImpulse, normalX, normalY, rAX, rAY, rBX, rBY, tCount, tMat, tVec, tX, tangentX, tangentY, vA, vB, vn, vn1, vn2, vt, wA, wB, xX, xY;
    j = 0;
    ccp = void 0;
    rAX = 0;
    rAY = 0;
    rBX = 0;
    rBY = 0;
    dvX = 0;
    dvY = 0;
    vn = 0;
    vt = 0;
    lambda = 0;
    maxFriction = 0;
    newImpulse = 0;
    PX = 0;
    PY = 0;
    dX = 0;
    dY = 0;
    P1X = 0;
    P1Y = 0;
    P2X = 0;
    P2Y = 0;
    tMat = void 0;
    tVec = void 0;
    i = 0;
    while (i < this.m_constraintCount) {
      c = this.m_constraints[i];
      bodyA = c.bodyA;
      bodyB = c.bodyB;
      wA = bodyA.m_angularVelocity;
      wB = bodyB.m_angularVelocity;
      vA = bodyA.m_linearVelocity;
      vB = bodyB.m_linearVelocity;
      invMassA = bodyA.m_invMass;
      invIA = bodyA.m_invI;
      invMassB = bodyB.m_invMass;
      invIB = bodyB.m_invI;
      normalX = c.normal.x;
      normalY = c.normal.y;
      tangentX = normalY;
      tangentY = -normalX;
      friction = c.friction;
      tX = 0;
      j = 0;
      while (j < c.pointCount) {
        ccp = c.points[j];
        dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
        dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
        vt = dvX * tangentX + dvY * tangentY;
        lambda = ccp.tangentMass * (-vt);
        maxFriction = friction * ccp.normalImpulse;
        newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - ccp.tangentImpulse;
        PX = lambda * tangentX;
        PY = lambda * tangentY;
        vA.x -= invMassA * PX;
        vA.y -= invMassA * PY;
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
        vB.x += invMassB * PX;
        vB.y += invMassB * PY;
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
        ccp.tangentImpulse = newImpulse;
        j++;
      }
      tCount = parseInt(c.pointCount);
      if (c.pointCount === 1) {
        ccp = c.points[0];
        dvX = vB.x + (-wB * ccp.rB.y) - vA.x - (-wA * ccp.rA.y);
        dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);
        vn = dvX * normalX + dvY * normalY;
        lambda = -ccp.normalMass * (vn - ccp.velocityBias);
        newImpulse = ccp.normalImpulse + lambda;
        newImpulse = (newImpulse > 0 ? newImpulse : 0.0);
        lambda = newImpulse - ccp.normalImpulse;
        PX = lambda * normalX;
        PY = lambda * normalY;
        vA.x -= invMassA * PX;
        vA.y -= invMassA * PY;
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
        vB.x += invMassB * PX;
        vB.y += invMassB * PY;
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
        ccp.normalImpulse = newImpulse;
      } else {
        cp1 = c.points[0];
        cp2 = c.points[1];
        aX = cp1.normalImpulse;
        aY = cp2.normalImpulse;
        dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
        dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
        dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
        dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
        vn1 = dv1X * normalX + dv1Y * normalY;
        vn2 = dv2X * normalX + dv2Y * normalY;
        bX = vn1 - cp1.velocityBias;
        bY = vn2 - cp2.velocityBias;
        tMat = c.K;
        bX -= tMat.col1.x * aX + tMat.col2.x * aY;
        bY -= tMat.col1.y * aX + tMat.col2.y * aY;
        k_errorTol = 0.001;
        while (true) {
          tMat = c.normalMass;
          xX = -(tMat.col1.x * bX + tMat.col2.x * bY);
          xY = -(tMat.col1.y * bX + tMat.col2.y * bY);
          if (xX >= 0.0 && xY >= 0.0) {
            dX = xX - aX;
            dY = xY - aY;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            P2X = dY * normalX;
            P2Y = dY * normalY;
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;
            break;
          }
          xX = -cp1.normalMass * bX;
          xY = 0.0;
          vn1 = 0.0;
          vn2 = c.K.col1.y * xX + bY;
          if (xX >= 0.0 && vn2 >= 0.0) {
            dX = xX - aX;
            dY = xY - aY;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            P2X = dY * normalX;
            P2Y = dY * normalY;
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;
            break;
          }
          xX = 0.0;
          xY = -cp2.normalMass * bY;
          vn1 = c.K.col2.x * xY + bX;
          vn2 = 0.0;
          if (xY >= 0.0 && vn1 >= 0.0) {
            dX = xX - aX;
            dY = xY - aY;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            P2X = dY * normalX;
            P2Y = dY * normalY;
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;
            break;
          }
          xX = 0.0;
          xY = 0.0;
          vn1 = bX;
          vn2 = bY;
          if (vn1 >= 0.0 && vn2 >= 0.0) {
            dX = xX - aX;
            dY = xY - aY;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            P2X = dY * normalX;
            P2Y = dY * normalY;
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;
            break;
          }
          break;
        }
      }
      bodyA.m_angularVelocity = wA;
      bodyB.m_angularVelocity = wB;
      ++i;
    }
  };

  b2ContactSolver.prototype.FinalizeVelocityConstraints = function() {
    var c, i, j, m, point1, point2;
    i = 0;
    while (i < this.m_constraintCount) {
      c = this.m_constraints[i];
      m = c.manifold;
      j = 0;
      while (j < c.pointCount) {
        point1 = m.m_points[j];
        point2 = c.points[j];
        point1.m_normalImpulse = point2.normalImpulse;
        point1.m_tangentImpulse = point2.tangentImpulse;
        ++j;
      }
      ++i;
    }
  };

  b2ContactSolver.prototype.SolvePositionConstraints = function(baumgarte) {
    var C, PX, PY, bodyA, bodyB, c, ccp, i, impulse, invIA, invIB, invMassA, invMassB, j, minSeparation, normal, point, rAX, rAY, rBX, rBY, separation;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    minSeparation = 0.0;
    i = 0;
    while (i < this.m_constraintCount) {
      c = this.m_constraints[i];
      bodyA = c.bodyA;
      bodyB = c.bodyB;
      invMassA = bodyA.m_mass * bodyA.m_invMass;
      invIA = bodyA.m_mass * bodyA.m_invI;
      invMassB = bodyB.m_mass * bodyB.m_invMass;
      invIB = bodyB.m_mass * bodyB.m_invI;
      b2ContactSolver.s_psm.Initialize(c);
      normal = b2ContactSolver.s_psm.m_normal;
      j = 0;
      while (j < c.pointCount) {
        ccp = c.points[j];
        point = b2ContactSolver.s_psm.m_points[j];
        separation = b2ContactSolver.s_psm.m_separations[j];
        rAX = point.x - bodyA.m_sweep.c.x;
        rAY = point.y - bodyA.m_sweep.c.y;
        rBX = point.x - bodyB.m_sweep.c.x;
        rBY = point.y - bodyB.m_sweep.c.y;
        minSeparation = (minSeparation < separation ? minSeparation : separation);
        C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);
        impulse = -ccp.equalizedMass * C;
        PX = impulse * normal.x;
        PY = impulse * normal.y;
        bodyA.m_sweep.c.x -= invMassA * PX;
        bodyA.m_sweep.c.y -= invMassA * PY;
        bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
        bodyA.SynchronizeTransform();
        bodyB.m_sweep.c.x += invMassB * PX;
        bodyB.m_sweep.c.y += invMassB * PY;
        bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
        bodyB.SynchronizeTransform();
        j++;
      }
      i++;
    }
    return minSeparation > (-1.5 * b2Settings.b2_linearSlop);
  };

  return b2ContactSolver;

})();

//# sourceMappingURL=b2ContactSolver.js.map

},{"../../index":103}],68:[function(require,module,exports){
var Box2D, b2Contact,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = (function(_super) {
  var b2CollideEdgeAndCircle;

  __extends(b2EdgeAndCircleContact, _super);

  function b2EdgeAndCircleContact() {
    return b2EdgeAndCircleContact.__super__.constructor.apply(this, arguments);
  }

  b2EdgeAndCircleContact.prototype.m_fixtureA = null;

  b2EdgeAndCircleContact.prototype.m_fixtureB = null;

  b2EdgeAndCircleContact.prototype.m_manifold = null;

  b2EdgeAndCircleContact.Create = function(allocator) {
    return new b2EdgeAndCircleContact();
  };

  b2EdgeAndCircleContact.Destroy = function(contact, allocator) {};

  b2EdgeAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    b2EdgeAndCircleContact.__super__.Reset.call(this, fixtureA, fixtureB);
  };

  b2EdgeAndCircleContact.prototype.Evaluate = function() {
    var bA, bB;
    bA = this.m_fixtureA.GetBody();
    bB = this.m_fixtureB.GetBody();
    this.b2CollideEdgeAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2EdgeShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
  };

  b2CollideEdgeAndCircle = function(manifold, edge, xf1, circle, xf2) {};

  return b2EdgeAndCircleContact;

})(b2Contact);

//# sourceMappingURL=b2EdgeAndCircleContact.js.map

},{"../../index":103}],69:[function(require,module,exports){
var Box2D, b2Contact,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

Box2D.Dynamics.Contacts.b2NullContact = (function(_super) {
  __extends(b2NullContact, _super);

  function b2NullContact() {
    return b2NullContact.__super__.constructor.apply(this, arguments);
  }

  b2NullContact.prototype.Evaluate = function() {};

  return b2NullContact;

})(b2Contact);

//# sourceMappingURL=b2NullContact.js.map

},{"../../index":103}],70:[function(require,module,exports){
var Box2D, b2Contact,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

Box2D.Dynamics.Contacts.b2PolyAndCircleContact = (function(_super) {
  var Evaluate;

  __extends(b2PolyAndCircleContact, _super);

  function b2PolyAndCircleContact() {
    return b2PolyAndCircleContact.__super__.constructor.apply(this, arguments);
  }

  b2PolyAndCircleContact.Create = function(allocator) {
    return new b2PolyAndCircleContact();
  };

  b2PolyAndCircleContact.Destroy = function(contact, allocator) {};

  b2PolyAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    b2PolyAndCircleContact.__super__.Reset.call(this, fixtureA, fixtureB);
    b2Settings.b2Assert(fixtureA.GetType() === b2Shape.e_polygonShape);
    b2Settings.b2Assert(fixtureB.GetType() === b2Shape.e_circleShape);
  };

  Evaluate = function() {
    var bA, bB;
    bA = this.m_fixtureA.m_body;
    bB = this.m_fixtureB.m_body;
    b2Collision.CollidePolygonAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
  };

  return b2PolyAndCircleContact;

})(b2Contact);

//# sourceMappingURL=b2PolyAndCircleContact.js.map

},{"../../index":103}],71:[function(require,module,exports){
var Box2D, b2Contact,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = (function(_super) {
  __extends(b2PolyAndEdgeContact, _super);

  function b2PolyAndEdgeContact() {
    return b2PolyAndEdgeContact.__super__.constructor.apply(this, arguments);
  }

  b2PolyAndEdgeContact.Create = function(allocator) {
    return new b2PolyAndEdgeContact();
  };

  b2PolyAndEdgeContact.Destroy = function(contact, allocator) {};

  b2PolyAndEdgeContact.prototype.Reset = function(fixtureA, fixtureB) {
    b2PolyAndEdgeContact.__super__.Reset.call(this, fixtureA, fixtureB);
    b2Settings.b2Assert(fixtureA.GetType() === b2Shape.e_polygonShape);
    b2Settings.b2Assert(fixtureB.GetType() === b2Shape.e_edgeShape);
  };

  b2PolyAndEdgeContact.prototype.Evaluate = function() {
    var bA, bB;
    bA = this.m_fixtureA.GetBody();
    bB = this.m_fixtureB.GetBody();
    this.b2CollidePolyAndEdge(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2EdgeShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
  };

  return b2PolyAndEdgeContact;

})(b2Contact);

//# sourceMappingURL=b2PolyAndEdgeContact.js.map

},{"../../index":103}],72:[function(require,module,exports){
var Box2D, b2Collision, b2Contact, b2PolygonShape,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Contact = Box2D.Dynamics.Contacts.b2Contact;

b2Collision = Box2D.Collision.b2Collision;

b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

Box2D.Dynamics.Contacts.b2PolygonContact = (function(_super) {
  __extends(b2PolygonContact, _super);

  function b2PolygonContact() {
    return b2PolygonContact.__super__.constructor.apply(this, arguments);
  }

  b2PolygonContact.Create = function(allocator) {
    return new b2PolygonContact();
  };

  b2PolygonContact.Destroy = function(contact, allocator) {};

  b2PolygonContact.prototype.Reset = function(fixtureA, fixtureB) {
    b2PolygonContact.__super__.Reset.call(this, fixtureA, fixtureB);
  };

  b2PolygonContact.prototype.Evaluate = function() {
    var bA, bB;
    bA = this.m_fixtureA.GetBody();
    bB = this.m_fixtureB.GetBody();
    b2Collision.CollidePolygons(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2PolygonShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
  };

  return b2PolygonContact;

})(b2Contact);

//# sourceMappingURL=b2PolygonContact.js.map

},{"../../index":103}],73:[function(require,module,exports){
var Box2D, Vector, b2Settings, b2Vec2;

Box2D = require('../../index');

Vector = Box2D.Vector;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Settings = Box2D.Common.b2Settings;

Box2D.Dynamics.Contacts.b2PositionSolverManifold = (function() {
  b2PositionSolverManifold.prototype.m_normal = null;

  b2PositionSolverManifold.prototype.m_separations = null;

  b2PositionSolverManifold.prototype.m_points = null;

  function b2PositionSolverManifold() {
    var i;
    this.m_normal = new b2Vec2();
    this.m_separations = new Vector(b2Settings.b2_maxManifoldPoints);
    this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
    i = 0;
    while (i < b2Settings.b2_maxManifoldPoints) {
      this.m_points[i] = new b2Vec2();
      i++;
    }
    return;
  }

  b2PositionSolverManifold.prototype.Initialize = function(cc) {
    var clipPointX, clipPointY, d, d2, dX, dY, i, planePointX, planePointY, pointAX, pointAY, pointBX, pointBY, tMat, tVec, _results;
    b2Settings.b2Assert(cc.pointCount > 0);
    i = 0;
    clipPointX = 0;
    clipPointY = 0;
    tMat = void 0;
    tVec = void 0;
    planePointX = 0;
    planePointY = 0;
    switch (cc.type) {
      case b2Manifold.e_circles:
        tMat = cc.bodyA.m_xf.R;
        tVec = cc.localPoint;
        pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tMat = cc.bodyB.m_xf.R;
        tVec = cc.points[0].localPoint;
        pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        dX = pointBX - pointAX;
        dY = pointBY - pointAY;
        d2 = dX * dX + dY * dY;
        if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
          d = Math.sqrt(d2);
          this.m_normal.x = dX / d;
          this.m_normal.y = dY / d;
        } else {
          this.m_normal.x = 1.0;
          this.m_normal.y = 0.0;
        }
        this.m_points[0].x = 0.5 * (pointAX + pointBX);
        this.m_points[0].y = 0.5 * (pointAY + pointBY);
        return this.m_separations[0] = dX * this.m_normal.x + dY * this.m_normal.y - cc.radius;
      case b2Manifold.e_faceA:
        tMat = cc.bodyA.m_xf.R;
        tVec = cc.localPlaneNormal;
        this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = cc.bodyA.m_xf.R;
        tVec = cc.localPoint;
        planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tMat = cc.bodyB.m_xf.R;
        i = 0;
        _results = [];
        while (i < cc.pointCount) {
          tVec = cc.points[i].localPoint;
          clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
          this.m_points[i].x = clipPointX;
          this.m_points[i].y = clipPointY;
          _results.push(++i);
        }
        return _results;
        break;
      case b2Manifold.e_faceB:
        tMat = cc.bodyB.m_xf.R;
        tVec = cc.localPlaneNormal;
        this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = cc.bodyB.m_xf.R;
        tVec = cc.localPoint;
        planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tMat = cc.bodyA.m_xf.R;
        i = 0;
        while (i < cc.pointCount) {
          tVec = cc.points[i].localPoint;
          clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
          this.m_points[i].Set(clipPointX, clipPointY);
          ++i;
        }
        this.m_normal.x *= -1.;
        return this.m_normal.y *= -1.;
    }
  };

  b2PositionSolverManifold.circlePointA = new b2Vec2();

  b2PositionSolverManifold.circlePointB = new b2Vec2();

  return b2PositionSolverManifold;

})();

//# sourceMappingURL=b2PositionSolverManifold.js.map

},{"../../index":103}],74:[function(require,module,exports){
var Box2D, b2Color, b2Controller, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Color = Box2D.Common.b2Color;

b2Controller = Box2D.Dynamics.Controllers.b2Controller;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Controllers.b2BuoyancyController = (function(_super) {
  __extends(b2BuoyancyController, _super);

  b2BuoyancyController.prototype.normal = null;

  b2BuoyancyController.prototype.offset = 0;

  b2BuoyancyController.prototype.density = 0;

  b2BuoyancyController.prototype.velocity = null;

  b2BuoyancyController.prototype.linearDrag = 2;

  b2BuoyancyController.prototype.angularDrag = 1;

  b2BuoyancyController.prototype.useDensity = false;

  b2BuoyancyController.prototype.useWorldGravity = true;

  b2BuoyancyController.prototype.gravity = null;

  function b2BuoyancyController() {
    b2BuoyancyController.__super__.constructor.apply(this, arguments);
    this.normal = new b2Vec2(0, -1.);
    this.velocity = new b2Vec2(0, 0);
    return;
  }

  b2BuoyancyController.prototype.Step = function(step) {
    var area, areac, body, buoyancyForce, dragForce, fixture, i, mass, massc, sarea, sc, shapeDensity;
    if (!this.m_bodyList) {
      return;
    }
    if (this.useWorldGravity) {
      this.gravity = this.GetWorld().GetGravity().Copy();
    }
    i = this.m_bodyList;
    while (i) {
      body = i.body;
      if (body.IsAwake() === false) {
        continue;
      }
      areac = new b2Vec2();
      massc = new b2Vec2();
      area = 0.0;
      mass = 0.0;
      fixture = body.GetFixtureList();
      while (fixture) {
        sc = new b2Vec2();
        sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
        area += sarea;
        areac.x += sarea * sc.x;
        areac.y += sarea * sc.y;
        shapeDensity = 0;
        if (this.useDensity) {
          shapeDensity = 1;
        } else {
          shapeDensity = 1;
        }
        mass += sarea * shapeDensity;
        massc.x += sarea * sc.x * shapeDensity;
        massc.y += sarea * sc.y * shapeDensity;
        fixture = fixture.GetNext();
      }
      areac.x /= area;
      areac.y /= area;
      massc.x /= mass;
      massc.y /= mass;
      if (area < Number.MIN_VALUE) {
        continue;
      }
      buoyancyForce = this.gravity.GetNegative();
      buoyancyForce.Multiply(this.density * area);
      body.ApplyForce(buoyancyForce, massc);
      dragForce = body.GetLinearVelocityFromWorldPoint(areac);
      dragForce.Subtract(this.velocity);
      dragForce.Multiply(-this.linearDrag * area);
      body.ApplyForce(dragForce, areac);
      body.ApplyTorque(-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag);
      i = i.nextBody;
    }
  };

  b2BuoyancyController.prototype.Draw = function(debugDraw) {
    var color, p1, p2, r;
    r = 1000;
    p1 = new b2Vec2();
    p2 = new b2Vec2();
    p1.x = this.normal.x * this.offset + this.normal.y * r;
    p1.y = this.normal.y * this.offset - this.normal.x * r;
    p2.x = this.normal.x * this.offset - this.normal.y * r;
    p2.y = this.normal.y * this.offset + this.normal.x * r;
    color = new b2Color(0, 0, 1);
    debugDraw.DrawSegment(p1, p2, color);
  };

  return b2BuoyancyController;

})(b2Controller);

//# sourceMappingURL=b2BuoyancyController.js.map

},{"../../index":103}],75:[function(require,module,exports){
var Box2D, b2Controller, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Controller = Box2D.Dynamics.Controllers.b2Controller;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Controllers.b2ConstantAccelController = (function(_super) {
  __extends(b2ConstantAccelController, _super);

  b2ConstantAccelController.prototype.A = null;

  function b2ConstantAccelController() {
    b2ConstantAccelController.__super__.constructor.apply(this, arguments);
    this.A = new b2Vec2(0, 0);
    return;
  }

  b2ConstantAccelController.prototype.Step = function(step) {
    var body, i, smallA;
    smallA = new b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
    i = this.m_bodyList;
    while (i) {
      body = i.body;
      if (!body.IsAwake()) {
        continue;
      }
      body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y));
      i = i.nextBody;
    }
  };

  return b2ConstantAccelController;

})(b2Controller);

//# sourceMappingURL=b2ConstantAccelController.js.map

},{"../../index":103}],76:[function(require,module,exports){
var Box2D, b2Controller, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Controller = Box2D.Dynamics.Controllers.b2Controller;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Controllers.b2ConstantForceController = (function(_super) {
  __extends(b2ConstantForceController, _super);

  b2ConstantForceController.prototype.F = null;

  function b2ConstantForceController() {
    b2ConstantForceController.__super__.constructor.apply(this, arguments);
    this.F = new b2Vec2(0, 0);
    return;
  }

  b2ConstantForceController.prototype.Step = function(step) {
    var body, i;
    i = this.m_bodyList;
    while (i) {
      body = i.body;
      if (!body.IsAwake()) {
        continue;
      }
      body.ApplyForce(this.F, body.GetWorldCenter());
      i = i.nextBody;
    }
  };

  return b2ConstantForceController;

})(b2Controller);

//# sourceMappingURL=b2ConstantForceController.js.map

},{"../../index":103}],77:[function(require,module,exports){
var Box2D, b2ControllerEdge;

Box2D = require('../../index');

b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge;

Box2D.Dynamics.Controllers.b2Controller = (function() {
  function b2Controller() {}

  b2Controller.prototype.m_bodyList = null;

  b2Controller.prototype.m_bodyCount = 0;

  b2Controller.prototype.m_next = null;

  b2Controller.prototype.m_world = null;

  b2Controller.prototype.Step = function(step) {};

  b2Controller.prototype.Draw = function(debugDraw) {};

  b2Controller.prototype.AddBody = function(body) {
    var edge;
    edge = new b2ControllerEdge();
    edge.controller = this;
    edge.body = body;
    edge.nextBody = this.m_bodyList;
    edge.prevBody = null;
    this.m_bodyList = edge;
    if (edge.nextBody) {
      edge.nextBody.prevBody = edge;
    }
    this.m_bodyCount++;
    edge.nextController = body.m_controllerList;
    edge.prevController = null;
    body.m_controllerList = edge;
    if (edge.nextController) {
      edge.nextController.prevController = edge;
    }
    body.m_controllerCount++;
  };

  b2Controller.prototype.RemoveBody = function(body) {
    var edge;
    edge = body.m_controllerList;
    while (edge && edge.controller !== this) {
      edge = edge.nextController;
    }
    if (edge.prevBody) {
      edge.prevBody.nextBody = edge.nextBody;
    }
    if (edge.nextBody) {
      edge.nextBody.prevBody = edge.prevBody;
    }
    if (edge.nextController) {
      edge.nextController.prevController = edge.prevController;
    }
    if (edge.prevController) {
      edge.prevController.nextController = edge.nextController;
    }
    if (this.m_bodyList === edge) {
      this.m_bodyList = edge.nextBody;
    }
    if (body.m_controllerList === edge) {
      body.m_controllerList = edge.nextController;
    }
    body.m_controllerCount--;
    this.m_bodyCount--;
  };

  b2Controller.prototype.Clear = function() {
    while (this.m_bodyList) {
      this.RemoveBody(this.m_bodyList.body);
    }
  };

  b2Controller.prototype.GetNext = function() {
    return this.m_next;
  };

  b2Controller.prototype.GetWorld = function() {
    return this.m_world;
  };

  b2Controller.prototype.GetBodyList = function() {
    return this.m_bodyList;
  };

  return b2Controller;

})();

//# sourceMappingURL=b2Controller.js.map

},{"../../index":103}],78:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.Controllers.b2ControllerEdge = (function() {
  function b2ControllerEdge() {}

  return b2ControllerEdge;

})();

//# sourceMappingURL=b2ControllerEdge.js.map

},{"../../index":103}],79:[function(require,module,exports){
var Box2D, b2Controller, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Controller = Box2D.Dynamics.Controllers.b2Controller;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Controllers.b2GravityController = (function(_super) {
  __extends(b2GravityController, _super);

  b2GravityController.prototype.G = 1;

  b2GravityController.prototype.invSqr = true;

  function b2GravityController() {
    b2GravityController.__super__.constructor.apply(this, arguments);
    this.G = 1;
    this.invSqr = true;
    return;
  }

  b2GravityController.prototype.Step = function(step) {
    var body1, body2, dx, dy, f, i, j, mass1, p1, p2, r2;
    i = null;
    body1 = null;
    p1 = null;
    mass1 = 0;
    j = null;
    body2 = null;
    p2 = null;
    dx = 0;
    dy = 0;
    r2 = 0;
    f = null;
    if (this.invSqr) {
      i = this.m_bodyList;
      while (i) {
        body1 = i.body;
        p1 = body1.GetWorldCenter();
        mass1 = body1.GetMass();
        j = this.m_bodyList;
        while (j !== i) {
          body2 = j.body;
          p2 = body2.GetWorldCenter();
          dx = p2.x - p1.x;
          dy = p2.y - p1.y;
          r2 = dx * dx + dy * dy;
          if (r2 < Number.MIN_VALUE) {
            continue;
          }
          f = new b2Vec2(dx, dy);
          f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
          if (body1.IsAwake()) {
            body1.ApplyForce(f, p1);
          }
          f.Multiply(-1.);
          if (body2.IsAwake()) {
            body2.ApplyForce(f, p2);
          }
          j = j.nextBody;
        }
        i = i.nextBody;
      }
    } else {
      i = this.m_bodyList;
      while (i) {
        body1 = i.body;
        p1 = body1.GetWorldCenter();
        mass1 = body1.GetMass();
        j = this.m_bodyList;
        while (j !== i) {
          body2 = j.body;
          p2 = body2.GetWorldCenter();
          dx = p2.x - p1.x;
          dy = p2.y - p1.y;
          r2 = dx * dx + dy * dy;
          if (r2 < Number.MIN_VALUE) {
            continue;
          }
          f = new b2Vec2(dx, dy);
          f.Multiply(this.G / r2 * mass1 * body2.GetMass());
          if (body1.IsAwake()) {
            body1.ApplyForce(f, p1);
          }
          f.Multiply(-1.);
          if (body2.IsAwake()) {
            body2.ApplyForce(f, p2);
          }
          j = j.nextBody;
        }
        i = i.nextBody;
      }
    }
  };

  return b2GravityController;

})(b2Controller);

//# sourceMappingURL=b2GravityController.js.map

},{"../../index":103}],80:[function(require,module,exports){
var Box2D, b2Controller, b2Mat22, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Controller = Box2D.Dynamics.Controllers.b2Controller;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

Box2D.Dynamics.Controllers.b2TensorDampingController = (function(_super) {
  __extends(b2TensorDampingController, _super);

  b2TensorDampingController.prototype.T = null;

  b2TensorDampingController.prototype.maxTimestep = 0;

  function b2TensorDampingController() {
    b2TensorDampingController.__super__.constructor.apply(this, arguments);
    this.T = new b2Mat22();
    this.maxTimestep = 0;
    return;
  }

  b2TensorDampingController.prototype.SetAxisAligned = function(xDamping, yDamping) {
    if (xDamping === void 0) {
      xDamping = 0;
    }
    if (yDamping === void 0) {
      yDamping = 0;
    }
    this.T.col1.x = -xDamping;
    this.T.col1.y = 0;
    this.T.col2.x = 0;
    this.T.col2.y = -yDamping;
    if (xDamping > 0 || yDamping > 0) {
      this.maxTimestep = 1 / Math.max(xDamping, yDamping);
    } else {
      this.maxTimestep = 0;
    }
  };

  b2TensorDampingController.prototype.Step = function(step) {
    var body, damping, i, timestep;
    timestep = step.dt;
    if (timestep <= Number.MIN_VALUE) {
      return;
    }
    if (timestep > this.maxTimestep && this.maxTimestep > 0) {
      timestep = this.maxTimestep;
    }
    i = this.m_bodyList;
    while (i) {
      body = i.body;
      if (!body.IsAwake()) {
        continue;
      }
      damping = body.GetWorldVector(b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
      body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep));
      i = i.nextBody;
    }
  };

  return b2TensorDampingController;

})(b2Controller);

//# sourceMappingURL=b2TensorDampingController.js.map

},{"../../index":103}],81:[function(require,module,exports){
var Box2D, b2Joint, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Joints.b2DistanceJoint = (function(_super) {
  __extends(b2DistanceJoint, _super);

  b2DistanceJoint.prototype.m_u = null;

  b2DistanceJoint.prototype.m_length = 0;

  b2DistanceJoint.prototype.m_frequencyHz = 0;

  b2DistanceJoint.prototype.m_dampingRatio = 0;

  b2DistanceJoint.prototype.m_impulse = 0.0;

  b2DistanceJoint.prototype.m_gamma = 0.0;

  b2DistanceJoint.prototype.m_bias = 0.0;

  function b2DistanceJoint(def) {
    b2DistanceJoint.__super__.constructor.call(this, def);
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_u = new b2Vec2();
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_length = def.length;
    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;
    return;
  }

  b2DistanceJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2DistanceJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2DistanceJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
  };

  b2DistanceJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return 0.0;
  };

  b2DistanceJoint.prototype.GetLength = function() {
    return this.m_length;
  };

  b2DistanceJoint.prototype.SetLength = function(length) {
    if (length === void 0) {
      length = 0;
    }
    this.m_length = length;
  };

  b2DistanceJoint.prototype.GetFrequency = function() {
    return this.m_frequencyHz;
  };

  b2DistanceJoint.prototype.SetFrequency = function(hz) {
    if (hz === void 0) {
      hz = 0;
    }
    this.m_frequencyHz = hz;
  };

  b2DistanceJoint.prototype.GetDampingRatio = function() {
    return this.m_dampingRatio;
  };

  b2DistanceJoint.prototype.SetDampingRatio = function(ratio) {
    if (ratio === void 0) {
      ratio = 0;
    }
    this.m_dampingRatio = ratio;
  };

  b2DistanceJoint.prototype.InitVelocityConstraints = function(step) {
    var C, PX, PY, bA, bB, cr1u, cr2u, d, invMass, k, length, omega, r1X, r1Y, r2X, r2Y, tMat, tX;
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
    if (length > b2Settings.b2_linearSlop) {
      this.m_u.Multiply(1.0 / length);
    } else {
      this.m_u.SetZero();
    }
    cr1u = r1X * this.m_u.y - r1Y * this.m_u.x;
    cr2u = r2X * this.m_u.y - r2Y * this.m_u.x;
    invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;
    this.m_mass = (invMass !== 0.0 ? 1.0 / invMass : 0.0);
    if (this.m_frequencyHz > 0.0) {
      C = length - this.m_length;
      omega = 2.0 * Math.PI * this.m_frequencyHz;
      d = 2.0 * this.m_mass * this.m_dampingRatio * omega;
      k = this.m_mass * omega * omega;
      this.m_gamma = step.dt * (d + step.dt * k);
      this.m_gamma = (this.m_gamma !== 0.0 ? 1 / this.m_gamma : 0.0);
      this.m_bias = C * step.dt * k * this.m_gamma;
      this.m_mass = invMass + this.m_gamma;
      this.m_mass = (this.m_mass !== 0.0 ? 1.0 / this.m_mass : 0.0);
    }
    if (step.warmStarting) {
      this.m_impulse *= step.dtRatio;
      PX = this.m_impulse * this.m_u.x;
      PY = this.m_impulse * this.m_u.y;
      bA.m_linearVelocity.x -= bA.m_invMass * PX;
      bA.m_linearVelocity.y -= bA.m_invMass * PY;
      bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
      bB.m_linearVelocity.x += bB.m_invMass * PX;
      bB.m_linearVelocity.y += bB.m_invMass * PY;
      bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
    } else {
      this.m_impulse = 0.0;
    }
  };

  b2DistanceJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, PX, PY, bA, bB, impulse, r1X, r1Y, r2X, r2Y, tMat, tX, v1X, v1Y, v2X, v2Y;
    tMat = void 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
    v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
    v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
    v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
    Cdot = this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y);
    impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
    this.m_impulse += impulse;
    PX = impulse * this.m_u.x;
    PY = impulse * this.m_u.y;
    bA.m_linearVelocity.x -= bA.m_invMass * PX;
    bA.m_linearVelocity.y -= bA.m_invMass * PY;
    bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
    bB.m_linearVelocity.x += bB.m_invMass * PX;
    bB.m_linearVelocity.y += bB.m_invMass * PY;
    bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
  };

  b2DistanceJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C, PX, PY, bA, bB, dX, dY, impulse, length, r1X, r1Y, r2X, r2Y, tMat, tX;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    tMat = void 0;
    if (this.m_frequencyHz > 0.0) {
      return true;
    }
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    length = Math.sqrt(dX * dX + dY * dY);
    dX /= length;
    dY /= length;
    C = length - this.m_length;
    C = b2Math.Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
    impulse = -this.m_mass * C;
    this.m_u.Set(dX, dY);
    PX = impulse * this.m_u.x;
    PY = impulse * this.m_u.y;
    bA.m_sweep.c.x -= bA.m_invMass * PX;
    bA.m_sweep.c.y -= bA.m_invMass * PY;
    bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
    bB.m_sweep.c.x += bB.m_invMass * PX;
    bB.m_sweep.c.y += bB.m_invMass * PY;
    bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return b2Math.Abs(C) < b2Settings.b2_linearSlop;
  };

  return b2DistanceJoint;

})(b2Joint);

//# sourceMappingURL=b2DistanceJoint.js.map

},{"../../index":103}],82:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2DistanceJointDef = (function(_super) {
  __extends(b2DistanceJointDef, _super);

  b2DistanceJointDef.prototype.type = 0;

  b2DistanceJointDef.prototype.localAnchorA = null;

  b2DistanceJointDef.prototype.localAnchorB = null;

  b2DistanceJointDef.prototype.userData = null;

  b2DistanceJointDef.prototype.bodyA = null;

  b2DistanceJointDef.prototype.bodyB = null;

  b2DistanceJointDef.prototype.length = 1.0;

  b2DistanceJointDef.prototype.frequencyHz = 0.0;

  b2DistanceJointDef.prototype.dampingRatio = 0.0;

  function b2DistanceJointDef() {
    this.localAnchorA = new b2Vec2();
    this.type = b2Joint.e_distanceJoint;
    return;
  }

  b2DistanceJointDef.prototype.Initialize = function(bA, bB, anchorA, anchorB) {
    var dX, dY;
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
    dX = anchorB.x - anchorA.x;
    dY = anchorB.y - anchorA.y;
    this.length = Math.sqrt(dX * dX + dY * dY);
    this.frequencyHz = 0.0;
    this.dampingRatio = 0.0;
  };

  return b2DistanceJointDef;

})(b2JointDef);

//# sourceMappingURL=b2DistanceJointDef.js.map

},{"../../index":103}],83:[function(require,module,exports){
var Box2D, b2Joint, b2Mat22, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

Box2D.Dynamics.Joints.b2FrictionJoint = (function(_super) {
  __extends(b2FrictionJoint, _super);

  b2FrictionJoint.prototype.m_linearMass = null;

  b2FrictionJoint.prototype.m_linearImpulse = null;

  b2FrictionJoint.prototype.m_angularMass = 0.0;

  b2FrictionJoint.prototype.m_angularImpulse = 0.0;

  b2FrictionJoint.prototype.m_maxForce = 0;

  b2FrictionJoint.prototype.m_maxTorque = 0;

  function b2FrictionJoint(def) {
    b2FrictionJoint.__super__.constructor.call(this, def);
    this.m_localAnchorA = new b2Vec2();
    this.m_localAnchorB = new b2Vec2();
    this.m_linearMass = new b2Mat22();
    this.m_linearImpulse = new b2Vec2();
    this.m_localAnchorA.SetV(def.localAnchorA);
    this.m_localAnchorB.SetV(def.localAnchorB);
    this.m_linearMass.SetZero();
    this.m_linearImpulse.SetZero();
    this.m_maxForce = def.maxForce;
    this.m_maxTorque = def.maxTorque;
    return;
  }

  b2FrictionJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
  };

  b2FrictionJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
  };

  b2FrictionJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
  };

  b2FrictionJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return inv_dt * this.m_angularImpulse;
  };

  b2FrictionJoint.prototype.SetMaxForce = function(force) {
    if (force === void 0) {
      force = 0;
    }
    this.m_maxForce = force;
  };

  b2FrictionJoint.prototype.GetMaxForce = function() {
    return this.m_maxForce;
  };

  b2FrictionJoint.prototype.SetMaxTorque = function(torque) {
    if (torque === void 0) {
      torque = 0;
    }
    this.m_maxTorque = torque;
  };

  b2FrictionJoint.prototype.GetMaxTorque = function() {
    return this.m_maxTorque;
  };

  b2FrictionJoint.prototype.InitVelocityConstraints = function(step) {
    var K, P, bA, bB, iA, iB, mA, mB, rAX, rAY, rBX, rBY, tMat, tX;
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
    rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
    rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
    rAX = tX;
    tMat = bB.m_xf.R;
    rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
    rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
    rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
    rBX = tX;
    mA = bA.m_invMass;
    mB = bB.m_invMass;
    iA = bA.m_invI;
    iB = bB.m_invI;
    K = new b2Mat22();
    K.col1.x = mA + mB;
    K.col2.x = 0.0;
    K.col1.y = 0.0;
    K.col2.y = mA + mB;
    K.col1.x += iA * rAY * rAY;
    K.col2.x += -iA * rAX * rAY;
    K.col1.y += -iA * rAX * rAY;
    K.col2.y += iA * rAX * rAX;
    K.col1.x += iB * rBY * rBY;
    K.col2.x += -iB * rBX * rBY;
    K.col1.y += -iB * rBX * rBY;
    K.col2.y += iB * rBX * rBX;
    K.GetInverse(this.m_linearMass);
    this.m_angularMass = iA + iB;
    if (this.m_angularMass > 0.0) {
      this.m_angularMass = 1.0 / this.m_angularMass;
    }
    if (step.warmStarting) {
      this.m_linearImpulse.x *= step.dtRatio;
      this.m_linearImpulse.y *= step.dtRatio;
      this.m_angularImpulse *= step.dtRatio;
      P = this.m_linearImpulse;
      bA.m_linearVelocity.x -= mA * P.x;
      bA.m_linearVelocity.y -= mA * P.y;
      bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);
      bB.m_linearVelocity.x += mB * P.x;
      bB.m_linearVelocity.y += mB * P.y;
      bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
    } else {
      this.m_linearImpulse.SetZero();
      this.m_angularImpulse = 0.0;
    }
  };

  b2FrictionJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, CdotX, CdotY, bA, bB, iA, iB, impulse, impulseV, mA, mB, maxImpulse, oldImpulse, oldImpulseV, rAX, rAY, rBX, rBY, tMat, tX, vA, vB, wA, wB;
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    vA = bA.m_linearVelocity;
    wA = bA.m_angularVelocity;
    vB = bB.m_linearVelocity;
    wB = bB.m_angularVelocity;
    mA = bA.m_invMass;
    mB = bB.m_invMass;
    iA = bA.m_invI;
    iB = bB.m_invI;
    tMat = bA.m_xf.R;
    rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
    rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
    rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
    rAX = tX;
    tMat = bB.m_xf.R;
    rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
    rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
    rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
    rBX = tX;
    maxImpulse = 0;
    Cdot = wB - wA;
    impulse = -this.m_angularMass * Cdot;
    oldImpulse = this.m_angularImpulse;
    maxImpulse = step.dt * this.m_maxTorque;
    this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
    impulse = this.m_angularImpulse - oldImpulse;
    wA -= iA * impulse;
    wB += iB * impulse;
    CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
    CdotY = vB.y + wB * rBX - vA.y - wA * rAX;
    impulseV = b2Math.MulMV(this.m_linearMass, new b2Vec2(-CdotX, -CdotY));
    oldImpulseV = this.m_linearImpulse.Copy();
    this.m_linearImpulse.Add(impulseV);
    maxImpulse = step.dt * this.m_maxForce;
    if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
      this.m_linearImpulse.Normalize();
      this.m_linearImpulse.Multiply(maxImpulse);
    }
    impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);
    vA.x -= mA * impulseV.x;
    vA.y -= mA * impulseV.y;
    wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);
    vB.x += mB * impulseV.x;
    vB.y += mB * impulseV.y;
    wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
    bA.m_angularVelocity = wA;
    bB.m_angularVelocity = wB;
  };

  b2FrictionJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    return true;
  };

  return b2FrictionJoint;

})(b2Joint);

//# sourceMappingURL=b2FrictionJoint.js.map

},{"../../index":103}],84:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2FrictionJointDef = (function(_super) {
  __extends(b2FrictionJointDef, _super);

  function b2FrictionJointDef() {
    return b2FrictionJointDef.__super__.constructor.apply(this, arguments);
  }

  b2FrictionJointDef.prototype.type = b2Joint.e_frictionJoint;

  b2FrictionJointDef.prototype.localAnchorA = null;

  b2FrictionJointDef.prototype.localAnchorB = null;

  b2FrictionJointDef.prototype.maxForce = 0.0;

  b2FrictionJointDef.prototype.maxTorque = 0.0;

  return b2FrictionJointDef;

})(b2JointDef);

({
  constructor: function() {
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
  },
  Initialize: function(bA, bB, anchor) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
  }
});

//# sourceMappingURL=b2FrictionJointDef.js.map

},{"../../index":103}],85:[function(require,module,exports){
var Box2D, b2Jacobian, b2Joint, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian;

Box2D.Dynamics.Joints.b2GearJoint = (function(_super) {
  __extends(b2GearJoint, _super);

  b2GearJoint.prototype.m_groundAnchor1 = null;

  b2GearJoint.prototype.m_groundAnchor2 = null;

  b2GearJoint.prototype.m_J = null;

  b2GearJoint.prototype.m_revolute1 = null;

  b2GearJoint.prototype.m_prismatic1 = null;

  b2GearJoint.prototype.m_revolute2 = null;

  b2GearJoint.prototype.m_prismatic2 = null;

  b2GearJoint.prototype.m_ground1 = null;

  b2GearJoint.prototype.m_ground2 = null;

  b2GearJoint.prototype.m_ratio = 0;

  b2GearJoint.prototype.m_constant = 0;

  b2GearJoint.prototype.m_impulse = 0.0;

  function b2GearJoint(def) {
    var coordinate1, coordinate2, type1, type2;
    b2GearJoint.__super__.constructor.call(this, def);
    this.m_groundAnchor1 = new b2Vec2();
    this.m_groundAnchor2 = new b2Vec2();
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_J = new b2Jacobian();
    type1 = parseInt(def.joint1.m_type);
    type2 = parseInt(def.joint2.m_type);
    coordinate1 = 0;
    coordinate2 = 0;
    this.m_ground1 = def.joint1.GetBodyA();
    this.m_bodyA = def.joint1.GetBodyB();
    if (type1 === b2Joint.e_revoluteJoint) {
      this.m_revolute1 = (def.joint1 instanceof b2RevoluteJoint ? def.joint1 : null);
      this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
      this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
      coordinate1 = this.m_revolute1.GetJointAngle();
    } else {
      this.m_prismatic1 = (def.joint1 instanceof b2PrismaticJoint ? def.joint1 : null);
      this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
      this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
      coordinate1 = this.m_prismatic1.GetJointTranslation();
    }
    this.m_ground2 = def.joint2.GetBodyA();
    this.m_bodyB = def.joint2.GetBodyB();
    if (type2 === b2Joint.e_revoluteJoint) {
      this.m_revolute2 = (def.joint2 instanceof b2RevoluteJoint ? def.joint2 : null);
      this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
      this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
      coordinate2 = this.m_revolute2.GetJointAngle();
    } else {
      this.m_prismatic2 = (def.joint2 instanceof b2PrismaticJoint ? def.joint2 : null);
      this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
      this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
      coordinate2 = this.m_prismatic2.GetJointTranslation();
    }
    this.m_ratio = def.ratio;
    this.m_constant = coordinate1 + this.m_ratio * coordinate2;
    return;
  }

  b2GearJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2GearJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2GearJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
  };

  b2GearJoint.prototype.GetReactionTorque = function(inv_dt) {
    var PX, PY, rX, rY, tMat, tX;
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    tMat = this.m_bodyB.m_xf.R;
    rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
    rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rX + tMat.col2.x * rY;
    rY = tMat.col1.y * rX + tMat.col2.y * rY;
    rX = tX;
    PX = this.m_impulse * this.m_J.linearB.x;
    PY = this.m_impulse * this.m_J.linearB.y;
    return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
  };

  b2GearJoint.prototype.GetRatio = function() {
    return this.m_ratio;
  };

  b2GearJoint.prototype.SetRatio = function(ratio) {
    if (ratio === void 0) {
      ratio = 0;
    }
    this.m_ratio = ratio;
  };

  b2GearJoint.prototype.InitVelocityConstraints = function(step) {
    var K, bA, bB, crug, g1, g2, rX, rY, tMat, tVec, tX, ugX, ugY;
    g1 = this.m_ground1;
    g2 = this.m_ground2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    ugX = 0;
    ugY = 0;
    rX = 0;
    rY = 0;
    tMat = void 0;
    tVec = void 0;
    crug = 0;
    tX = 0;
    K = 0.0;
    this.m_J.SetZero();
    if (this.m_revolute1) {
      this.m_J.angularA = -1.0;
      K += bA.m_invI;
    } else {
      tMat = g1.m_xf.R;
      tVec = this.m_prismatic1.m_localXAxis1;
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = bA.m_xf.R;
      rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      crug = rX * ugY - rY * ugX;
      this.m_J.linearA.Set(-ugX, -ugY);
      this.m_J.angularA = -crug;
      K += bA.m_invMass + bA.m_invI * crug * crug;
    }
    if (this.m_revolute2) {
      this.m_J.angularB = -this.m_ratio;
      K += this.m_ratio * this.m_ratio * bB.m_invI;
    } else {
      tMat = g2.m_xf.R;
      tVec = this.m_prismatic2.m_localXAxis1;
      ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = bB.m_xf.R;
      rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      crug = rX * ugY - rY * ugX;
      this.m_J.linearB.Set(-this.m_ratio * ugX, -this.m_ratio * ugY);
      this.m_J.angularB = -this.m_ratio * crug;
      K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
    }
    this.m_mass = (K > 0.0 ? 1.0 / K : 0.0);
    if (step.warmStarting) {
      bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
      bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
      bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
      bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
      bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
      bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
    } else {
      this.m_impulse = 0.0;
    }
  };

  b2GearJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, bA, bB, impulse;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    Cdot = this.m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity);
    impulse = -this.m_mass * Cdot;
    this.m_impulse += impulse;
    bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_angularVelocity += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_angularVelocity += bB.m_invI * impulse * this.m_J.angularB;
  };

  b2GearJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C, bA, bB, coordinate1, coordinate2, impulse, linearError;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    linearError = 0.0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    coordinate1 = 0;
    coordinate2 = 0;
    if (this.m_revolute1) {
      coordinate1 = this.m_revolute1.GetJointAngle();
    } else {
      coordinate1 = this.m_prismatic1.GetJointTranslation();
    }
    if (this.m_revolute2) {
      coordinate2 = this.m_revolute2.GetJointAngle();
    } else {
      coordinate2 = this.m_prismatic2.GetJointTranslation();
    }
    C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);
    impulse = -this.m_mass * C;
    bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
    bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;
    bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
    bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
    bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;
    bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return linearError < b2Settings.b2_linearSlop;
  };

  return b2GearJoint;

})(b2Joint);

//# sourceMappingURL=b2GearJoint.js.map

},{"../../index":103}],86:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2GearJointDef = (function(_super) {
  __extends(b2GearJointDef, _super);

  function b2GearJointDef() {
    return b2GearJointDef.__super__.constructor.apply(this, arguments);
  }

  b2GearJointDef.prototype.type = b2Joint.e_gearJoint;

  b2GearJointDef.prototype.joint1 = null;

  b2GearJointDef.prototype.joint2 = null;

  b2GearJointDef.prototype.ratio = 1.0;

  return b2GearJointDef;

})(b2JointDef);

//# sourceMappingURL=b2GearJointDef.js.map

},{"../../index":103}],87:[function(require,module,exports){
var Box2D, b2Vec2;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Joints.b2Jacobian = (function() {
  b2Jacobian.prototype.linearA = null;

  b2Jacobian.prototype.linearB = null;

  b2Jacobian.prototype.angularA = null;

  b2Jacobian.prototype.angularB = null;

  function b2Jacobian() {
    this.linearA = new b2Vec2();
    this.linearB = new b2Vec2();
    return;
  }

  b2Jacobian.prototype.SetZero = function() {
    this.linearA.SetZero();
    this.angularA = 0.0;
    this.linearB.SetZero();
    this.angularB = 0.0;
  };

  b2Jacobian.prototype.Set = function(x1, a1, x2, a2) {
    if (a1 === void 0) {
      a1 = 0;
    }
    if (a2 === void 0) {
      a2 = 0;
    }
    this.linearA.SetV(x1);
    this.angularA = a1;
    this.linearB.SetV(x2);
    this.angularB = a2;
  };

  b2Jacobian.prototype.Compute = function(x1, a1, x2, a2) {
    if (a1 === void 0) {
      a1 = 0;
    }
    if (a2 === void 0) {
      a2 = 0;
    }
    return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
  };

  return b2Jacobian;

})();

//# sourceMappingURL=b2Jacobian.js.map

},{"../../index":103}],88:[function(require,module,exports){
var Box2D, b2DistanceJoint, b2DistanceJointDef, b2FrictionJoint, b2FrictionJointDef, b2GearJoint, b2GearJointDef, b2JointEdge, b2LineJoint, b2LineJointDef, b2MouseJoint, b2MouseJointDef, b2PrismaticJoint, b2PrismaticJointDef, b2PulleyJoint, b2PulleyJointDef, b2RevoluteJoint, b2RevoluteJointDef, b2Vec2, b2WeldJoint, b2WeldJointDef;

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge;

b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint;

b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint;

b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint;

b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint;

b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint;

b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint;

b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint;

b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint;

b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint;

b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef;

b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;

b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef;

b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef;

b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef;

b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef;

b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef;

b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef;

b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef;

Box2D.Dynamics.Joints.b2Joint = (function() {
  b2Joint.e_unknownJoint = 0;

  b2Joint.e_revoluteJoint = 1;

  b2Joint.e_prismaticJoint = 2;

  b2Joint.e_distanceJoint = 3;

  b2Joint.e_pulleyJoint = 4;

  b2Joint.e_mouseJoint = 5;

  b2Joint.e_gearJoint = 6;

  b2Joint.e_lineJoint = 7;

  b2Joint.e_weldJoint = 8;

  b2Joint.e_frictionJoint = 9;

  b2Joint.e_inactiveLimit = 0;

  b2Joint.e_atLowerLimit = 1;

  b2Joint.e_atUpperLimit = 2;

  b2Joint.e_equalLimits = 3;

  b2Joint.prototype.m_type = b2Joint.e_unknownJoint;

  b2Joint.prototype.m_edgeA = null;

  b2Joint.prototype.m_edgeB = null;

  b2Joint.prototype.m_localAnchor1 = null;

  b2Joint.prototype.m_localAnchor2 = null;

  b2Joint.prototype.m_localCenterA = null;

  b2Joint.prototype.m_localCenterB = null;

  b2Joint.prototype.m_prev = null;

  b2Joint.prototype.m_next = null;

  b2Joint.prototype.m_bodyA = null;

  b2Joint.prototype.m_bodyB = null;

  b2Joint.prototype.m_collideConnected = null;

  b2Joint.prototype.m_islandFlag = null;

  b2Joint.prototype.m_userData = null;

  function b2Joint(def) {
    this.m_edgeA = new b2JointEdge();
    this.m_edgeB = new b2JointEdge();
    this.m_localCenterA = new b2Vec2();
    this.m_localCenterB = new b2Vec2();
    b2Settings.b2Assert(def.bodyA !== def.bodyB);
    this.m_type = def.type;
    this.m_prev = null;
    this.m_next = null;
    this.m_bodyA = def.bodyA;
    this.m_bodyB = def.bodyB;
    this.m_collideConnected = def.collideConnected;
    this.m_islandFlag = false;
    this.m_userData = def.userData;
    return;
  }

  b2Joint.prototype.GetType = function() {
    return this.m_type;
  };

  b2Joint.prototype.GetAnchorA = function() {
    return null;
  };

  b2Joint.prototype.GetAnchorB = function() {
    return null;
  };

  b2Joint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return null;
  };

  b2Joint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return 0.0;
  };

  b2Joint.prototype.GetBodyA = function() {
    return this.m_bodyA;
  };

  b2Joint.prototype.GetBodyB = function() {
    return this.m_bodyB;
  };

  b2Joint.prototype.GetNext = function() {
    return this.m_next;
  };

  b2Joint.prototype.GetUserData = function() {
    return this.m_userData;
  };

  b2Joint.prototype.SetUserData = function(data) {
    this.m_userData = data;
  };

  b2Joint.prototype.IsActive = function() {
    return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
  };

  b2Joint.Create = function(def, allocator) {
    var joint;
    joint = null;
    switch (def.type) {
      case b2Joint.e_distanceJoint:
        joint = new b2DistanceJoint((def instanceof b2DistanceJointDef ? def : null));
        break;
      case b2Joint.e_mouseJoint:
        joint = new b2MouseJoint((def instanceof b2MouseJointDef ? def : null));
        break;
      case b2Joint.e_prismaticJoint:
        joint = new b2PrismaticJoint((def instanceof b2PrismaticJointDef ? def : null));
        break;
      case b2Joint.e_revoluteJoint:
        joint = new b2RevoluteJoint((def instanceof b2RevoluteJointDef ? def : null));
        break;
      case b2Joint.e_pulleyJoint:
        joint = new b2PulleyJoint((def instanceof b2PulleyJointDef ? def : null));
        break;
      case b2Joint.e_gearJoint:
        joint = new b2GearJoint((def instanceof b2GearJointDef ? def : null));
        break;
      case b2Joint.e_lineJoint:
        joint = new b2LineJoint((def instanceof b2LineJointDef ? def : null));
        break;
      case b2Joint.e_weldJoint:
        joint = new b2WeldJoint((def instanceof b2WeldJointDef ? def : null));
        break;
      case b2Joint.e_frictionJoint:
        joint = new b2FrictionJoint((def instanceof b2FrictionJointDef ? def : null));
        break;
    }
    return joint;
  };

  b2Joint.Destroy = function(joint, allocator) {};

  b2Joint.prototype.InitVelocityConstraints = function(step) {};

  b2Joint.prototype.SolveVelocityConstraints = function(step) {};

  b2Joint.prototype.FinalizeVelocityConstraints = function() {};

  b2Joint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    return false;
  };

  return b2Joint;

})();

//# sourceMappingURL=b2Joint.js.map

},{"../../index":103}],89:[function(require,module,exports){
var Box2D, b2Joint;

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

Box2D.Dynamics.Joints.b2JointDef = (function() {
  function b2JointDef() {}

  b2JointDef.prototype.type = b2Joint.e_unknownJoint;

  b2JointDef.prototype.userData = null;

  b2JointDef.prototype.bodyA = null;

  b2JointDef.prototype.bodyB = null;

  b2JointDef.prototype.collideConnected = null;

  return b2JointDef;

})();

//# sourceMappingURL=b2JointDef.js.map

},{"../../index":103}],90:[function(require,module,exports){
var Box2D;

Box2D = require('../../index');

Box2D.Dynamics.Joints.b2JointEdge = (function() {
  function b2JointEdge() {}

  return b2JointEdge;

})();

//# sourceMappingURL=b2JointEdge.js.map

},{"../../index":103}],91:[function(require,module,exports){
var Box2D, b2Joint, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

Box2D.Dynamics.Joints.b2LineJoint = (function(_super) {
  __extends(b2LineJoint, _super);

  b2LineJoint.prototype.m_localXAxis1 = null;

  b2LineJoint.prototype.m_localYAxis1 = null;

  b2LineJoint.prototype.m_axis = null;

  b2LineJoint.prototype.m_perp = null;

  b2LineJoint.prototype.m_K = null;

  b2LineJoint.prototype.m_impulse = null;

  b2LineJoint.prototype.m_motorMass = 0.0;

  b2LineJoint.prototype.m_motorImpulse = 0.0;

  b2LineJoint.prototype.m_lowerTranslation = 0.0;

  b2LineJoint.prototype.m_upperTranslation = 0.0;

  b2LineJoint.prototype.m_maxMotorForce = 0.0;

  b2LineJoint.prototype.m_motorSpeed = 0.0;

  b2LineJoint.prototype.m_enableLimit = false;

  b2LineJoint.prototype.m_enableMotor = false;

  b2LineJoint.prototype.m_limitState = b2Joint.e_inactiveLimit;

  function b2LineJoint(def) {
    b2LineJoint.__super__.constructor.call(this, def);
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_localXAxis1 = new b2Vec2();
    this.m_localYAxis1 = new b2Vec2();
    this.m_axis = new b2Vec2();
    this.m_perp = new b2Vec2();
    this.m_K = new b2Mat22();
    this.m_impulse = new b2Vec2();
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_localXAxis1.SetV(def.localAxisA);
    this.m_localYAxis1.x = -this.m_localXAxis1.y;
    this.m_localYAxis1.y = this.m_localXAxis1.x;
    this.m_impulse.SetZero();
    this.m_lowerTranslation = def.lowerTranslation;
    this.m_upperTranslation = def.upperTranslation;
    this.m_maxMotorForce = def.maxMotorForce;
    this.m_motorSpeed = def.motorSpeed;
    this.m_enableLimit = def.enableLimit;
    this.m_enableMotor = def.enableMotor;
    this.m_axis.SetZero();
    this.m_perp.SetZero();
    return;
  }

  b2LineJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2LineJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2LineJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y));
  };

  b2LineJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return inv_dt * this.m_impulse.y;
  };

  b2LineJoint.prototype.GetJointTranslation = function() {
    var axis, bA, bB, dX, dY, p1, p2, tMat, translation;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    p1 = bA.GetWorldPoint(this.m_localAnchor1);
    p2 = bB.GetWorldPoint(this.m_localAnchor2);
    dX = p2.x - p1.x;
    dY = p2.y - p1.y;
    axis = bA.GetWorldVector(this.m_localXAxis1);
    translation = axis.x * dX + axis.y * dY;
    return translation;
  };

  b2LineJoint.prototype.GetJointSpeed = function() {
    var axis, bA, bB, dX, dY, p1X, p1Y, p2X, p2Y, r1X, r1Y, r2X, r2Y, speed, tMat, tX, v1, v2, w1, w2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    p1X = bA.m_sweep.c.x + r1X;
    p1Y = bA.m_sweep.c.y + r1Y;
    p2X = bB.m_sweep.c.x + r2X;
    p2Y = bB.m_sweep.c.y + r2Y;
    dX = p2X - p1X;
    dY = p2Y - p1Y;
    axis = bA.GetWorldVector(this.m_localXAxis1);
    v1 = bA.m_linearVelocity;
    v2 = bB.m_linearVelocity;
    w1 = bA.m_angularVelocity;
    w2 = bB.m_angularVelocity;
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
    return speed;
  };

  b2LineJoint.prototype.IsLimitEnabled = function() {
    return this.m_enableLimit;
  };

  b2LineJoint.prototype.EnableLimit = function(flag) {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_enableLimit = flag;
  };

  b2LineJoint.prototype.GetLowerLimit = function() {
    return this.m_lowerTranslation;
  };

  b2LineJoint.prototype.GetUpperLimit = function() {
    return this.m_upperTranslation;
  };

  b2LineJoint.prototype.SetLimits = function(lower, upper) {
    if (lower === void 0) {
      lower = 0;
    }
    if (upper === void 0) {
      upper = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_lowerTranslation = lower;
    this.m_upperTranslation = upper;
  };

  b2LineJoint.prototype.IsMotorEnabled = function() {
    return this.m_enableMotor;
  };

  b2LineJoint.prototype.EnableMotor = function(flag) {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_enableMotor = flag;
  };

  b2LineJoint.prototype.SetMotorSpeed = function(speed) {
    if (speed === void 0) {
      speed = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_motorSpeed = speed;
  };

  b2LineJoint.prototype.GetMotorSpeed = function() {
    return this.m_motorSpeed;
  };

  b2LineJoint.prototype.SetMaxMotorForce = function(force) {
    if (force === void 0) {
      force = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_maxMotorForce = force;
  };

  b2LineJoint.prototype.GetMaxMotorForce = function() {
    return this.m_maxMotorForce;
  };

  b2LineJoint.prototype.GetMotorForce = function() {
    return this.m_motorImpulse;
  };

  b2LineJoint.prototype.InitVelocityConstraints = function(step) {
    var L1, L2, PX, PY, bA, bB, dX, dY, i1, i2, jointTransition, m1, m2, r1X, r1Y, r2X, r2Y, tMat, tX, xf1, xf2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tX = 0;
    this.m_localCenterA.SetV(bA.GetLocalCenter());
    this.m_localCenterB.SetV(bB.GetLocalCenter());
    xf1 = bA.GetTransform();
    xf2 = bB.GetTransform();
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
    r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
    r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    this.m_invMassA = bA.m_invMass;
    this.m_invMassB = bB.m_invMass;
    this.m_invIA = bA.m_invI;
    this.m_invIB = bB.m_invI;
    this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
    this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
    this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
    this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
    this.m_motorMass = (this.m_motorMass > Number.MIN_VALUE ? 1.0 / this.m_motorMass : 0.0);
    this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
    this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
    this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
    m1 = this.m_invMassA;
    m2 = this.m_invMassB;
    i1 = this.m_invIA;
    i2 = this.m_invIB;
    this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
    this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
    this.m_K.col2.x = this.m_K.col1.y;
    this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
    if (this.m_enableLimit) {
      jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
      if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
        this.m_limitState = b2Joint.e_equalLimits;
      } else if (jointTransition <= this.m_lowerTranslation) {
        if (this.m_limitState !== b2Joint.e_atLowerLimit) {
          this.m_limitState = b2Joint.e_atLowerLimit;
          this.m_impulse.y = 0.0;
        }
      } else if (jointTransition >= this.m_upperTranslation) {
        if (this.m_limitState !== b2Joint.e_atUpperLimit) {
          this.m_limitState = b2Joint.e_atUpperLimit;
          this.m_impulse.y = 0.0;
        }
      } else {
        this.m_limitState = b2Joint.e_inactiveLimit;
        this.m_impulse.y = 0.0;
      }
    } else {
      this.m_limitState = b2Joint.e_inactiveLimit;
    }
    if (this.m_enableMotor === false) {
      this.m_motorImpulse = 0.0;
    }
    if (step.warmStarting) {
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      this.m_motorImpulse *= step.dtRatio;
      PX = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x;
      PY = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y;
      L1 = this.m_impulse.x * this.m_s1 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a1;
      L2 = this.m_impulse.x * this.m_s2 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a2;
      bA.m_linearVelocity.x -= this.m_invMassA * PX;
      bA.m_linearVelocity.y -= this.m_invMassA * PY;
      bA.m_angularVelocity -= this.m_invIA * L1;
      bB.m_linearVelocity.x += this.m_invMassB * PX;
      bB.m_linearVelocity.y += this.m_invMassB * PY;
      bB.m_angularVelocity += this.m_invIB * L2;
    } else {
      this.m_impulse.SetZero();
      this.m_motorImpulse = 0.0;
    }
  };

  b2LineJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, Cdot1, Cdot2, L1, L2, PX, PY, b, bA, bB, df, df2, f1, f2r, impulse, maxImpulse, oldImpulse, v1, v2, w1, w2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    v1 = bA.m_linearVelocity;
    w1 = bA.m_angularVelocity;
    v2 = bB.m_linearVelocity;
    w2 = bB.m_angularVelocity;
    PX = 0;
    PY = 0;
    L1 = 0;
    L2 = 0;
    if (this.m_enableMotor && this.m_limitState !== b2Joint.e_equalLimits) {
      Cdot = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
      impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
      oldImpulse = this.m_motorImpulse;
      maxImpulse = step.dt * this.m_maxMotorForce;
      this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.m_motorImpulse - oldImpulse;
      PX = impulse * this.m_axis.x;
      PY = impulse * this.m_axis.y;
      L1 = impulse * this.m_a1;
      L2 = impulse * this.m_a2;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    }
    Cdot1 = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
    if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
      Cdot2 = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
      f1 = this.m_impulse.Copy();
      df = this.m_K.Solve(new b2Vec2(), -Cdot1, -Cdot2);
      this.m_impulse.Add(df);
      if (this.m_limitState === b2Joint.e_atLowerLimit) {
        this.m_impulse.y = b2Math.Max(this.m_impulse.y, 0.0);
      } else {
        if (this.m_limitState === b2Joint.e_atUpperLimit) {
          this.m_impulse.y = b2Math.Min(this.m_impulse.y, 0.0);
        }
      }
      b = (-Cdot1) - (this.m_impulse.y - f1.y) * this.m_K.col2.x;
      f2r = 0;
      if (this.m_K.col1.x !== 0.0) {
        f2r = b / this.m_K.col1.x + f1.x;
      } else {
        f2r = f1.x;
      }
      this.m_impulse.x = f2r;
      df.x = this.m_impulse.x - f1.x;
      df.y = this.m_impulse.y - f1.y;
      PX = df.x * this.m_perp.x + df.y * this.m_axis.x;
      PY = df.x * this.m_perp.y + df.y * this.m_axis.y;
      L1 = df.x * this.m_s1 + df.y * this.m_a1;
      L2 = df.x * this.m_s2 + df.y * this.m_a2;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    } else {
      df2 = 0;
      if (this.m_K.col1.x !== 0.0) {
        df2 = (-Cdot1) / this.m_K.col1.x;
      } else {
        df2 = 0.0;
      }
      this.m_impulse.x += df2;
      PX = df2 * this.m_perp.x;
      PY = df2 * this.m_perp.y;
      L1 = df2 * this.m_s1;
      L2 = df2 * this.m_s2;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    }
    bA.m_linearVelocity.SetV(v1);
    bA.m_angularVelocity = w1;
    bB.m_linearVelocity.SetV(v2);
    bB.m_angularVelocity = w2;
  };

  b2LineJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C1, C2, L1, L2, PX, PY, R1, R2, a1, a2, active, angularError, bA, bB, c1, c2, dX, dY, i1, i2, impulse, impulse1, k11, limitC, linearError, m1, m2, oldLimitImpulse, r1X, r1Y, r2X, r2Y, tMat, tX, translation;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    limitC = 0;
    oldLimitImpulse = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    c1 = bA.m_sweep.c;
    a1 = bA.m_sweep.a;
    c2 = bB.m_sweep.c;
    a2 = bB.m_sweep.a;
    tMat = void 0;
    tX = 0;
    m1 = 0;
    m2 = 0;
    i1 = 0;
    i2 = 0;
    linearError = 0.0;
    angularError = 0.0;
    active = false;
    C2 = 0.0;
    R1 = b2Mat22.FromAngle(a1);
    R2 = b2Mat22.FromAngle(a2);
    tMat = R1;
    r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
    r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = R2;
    r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
    r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    dX = c2.x + r2X - c1.x - r1X;
    dY = c2.y + r2Y - c1.y - r1Y;
    if (this.m_enableLimit) {
      this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
      this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
      this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
      translation = this.m_axis.x * dX + this.m_axis.y * dY;
      if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
        C2 = b2Math.Clamp(translation, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
        linearError = b2Math.Abs(translation);
        active = true;
      } else if (translation <= this.m_lowerTranslation) {
        C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
        linearError = this.m_lowerTranslation - translation;
        active = true;
      } else if (translation >= this.m_upperTranslation) {
        C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
        linearError = translation - this.m_upperTranslation;
        active = true;
      }
    }
    this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
    this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
    this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
    impulse = new b2Vec2();
    C1 = this.m_perp.x * dX + this.m_perp.y * dY;
    linearError = b2Math.Max(linearError, b2Math.Abs(C1));
    angularError = 0.0;
    if (active) {
      m1 = this.m_invMassA;
      m2 = this.m_invMassB;
      i1 = this.m_invIA;
      i2 = this.m_invIB;
      this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
      this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
      this.m_K.col2.x = this.m_K.col1.y;
      this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
      this.m_K.Solve(impulse, -C1, -C2);
    } else {
      m1 = this.m_invMassA;
      m2 = this.m_invMassB;
      i1 = this.m_invIA;
      i2 = this.m_invIB;
      k11 = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
      impulse1 = 0;
      if (k11 !== 0.0) {
        impulse1 = (-C1) / k11;
      } else {
        impulse1 = 0.0;
      }
      impulse.x = impulse1;
      impulse.y = 0.0;
    }
    PX = impulse.x * this.m_perp.x + impulse.y * this.m_axis.x;
    PY = impulse.x * this.m_perp.y + impulse.y * this.m_axis.y;
    L1 = impulse.x * this.m_s1 + impulse.y * this.m_a1;
    L2 = impulse.x * this.m_s2 + impulse.y * this.m_a2;
    c1.x -= this.m_invMassA * PX;
    c1.y -= this.m_invMassA * PY;
    a1 -= this.m_invIA * L1;
    c2.x += this.m_invMassB * PX;
    c2.y += this.m_invMassB * PY;
    a2 += this.m_invIB * L2;
    bA.m_sweep.a = a1;
    bB.m_sweep.a = a2;
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
  };

  return b2LineJoint;

})(b2Joint);

//# sourceMappingURL=b2LineJoint.js.map

},{"../../index":103}],92:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2LineJointDef = (function(_super) {
  __extends(b2LineJointDef, _super);

  b2LineJointDef.prototype.type = b2Joint.e_lineJoint;

  b2LineJointDef.prototype.localAnchorA = null;

  b2LineJointDef.prototype.localAnchorB = null;

  b2LineJointDef.prototype.localAxisA = null;

  b2LineJointDef.prototype.enableLimit = false;

  b2LineJointDef.prototype.lowerTranslation = 0.0;

  b2LineJointDef.prototype.upperTranslation = 0.0;

  b2LineJointDef.prototype.enableMotor = false;

  b2LineJointDef.prototype.maxMotorForce = 0.0;

  b2LineJointDef.prototype.motorSpeed = 0.0;

  function b2LineJointDef() {
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.localAxisA = new b2Vec2();
    this.localAxisA.Set(1.0, 0.0);
    return;
  }

  b2LineJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.localAxisA = this.bodyA.GetLocalVector(axis);
  };

  return b2LineJointDef;

})(b2JointDef);

//# sourceMappingURL=b2LineJointDef.js.map

},{"../../index":103}],93:[function(require,module,exports){
var Box2D, b2Joint, b2Mat22, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

Box2D.Dynamics.Joints.b2MouseJoint = (function(_super) {
  __extends(b2MouseJoint, _super);

  b2MouseJoint.prototype.K = null;

  b2MouseJoint.prototype.K1 = null;

  b2MouseJoint.prototype.K2 = null;

  b2MouseJoint.prototype.m_localAnchor = null;

  b2MouseJoint.prototype.m_target = null;

  b2MouseJoint.prototype.m_impulse = null;

  b2MouseJoint.prototype.m_mass = null;

  b2MouseJoint.prototype.m_C = null;

  b2MouseJoint.prototype.m_frequencyHz = 0.0;

  b2MouseJoint.prototype.m_dampingRatio = 0.0;

  b2MouseJoint.prototype.m_beta = 0.0;

  b2MouseJoint.prototype.m_gamma = 0.0;

  function b2MouseJoint(def) {
    var tMat, tX, tY;
    b2MouseJoint.__super__.constructor.call(this, def);
    this.K = new b2Mat22();
    this.K1 = new b2Mat22();
    this.K2 = new b2Mat22();
    this.m_localAnchor = new b2Vec2();
    this.m_target = new b2Vec2();
    this.m_impulse = new b2Vec2();
    this.m_mass = new b2Mat22();
    this.m_C = new b2Vec2();
    this.m_target.SetV(def.target);
    tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
    tY = this.m_target.y - this.m_bodyB.m_xf.position.y;
    tMat = this.m_bodyB.m_xf.R;
    this.m_localAnchor.x = tX * tMat.col1.x + tY * tMat.col1.y;
    this.m_localAnchor.y = tX * tMat.col2.x + tY * tMat.col2.y;
    this.m_maxForce = def.maxForce;
    this.m_impulse.SetZero();
    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;
    return;
  }

  b2MouseJoint.prototype.GetAnchorA = function() {
    return this.m_target;
  };

  b2MouseJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
  };

  b2MouseJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
  };

  b2MouseJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return 0.0;
  };

  b2MouseJoint.prototype.GetTarget = function() {
    return this.m_target;
  };

  b2MouseJoint.prototype.SetTarget = function(target) {
    if (this.m_bodyB.IsAwake() === false) {
      this.m_bodyB.SetAwake(true);
    }
    this.m_target = target;
  };

  b2MouseJoint.prototype.GetMaxForce = function() {
    return this.m_maxForce;
  };

  b2MouseJoint.prototype.SetMaxForce = function(maxForce) {
    if (maxForce === void 0) {
      maxForce = 0;
    }
    this.m_maxForce = maxForce;
  };

  b2MouseJoint.prototype.GetFrequency = function() {
    return this.m_frequencyHz;
  };

  b2MouseJoint.prototype.SetFrequency = function(hz) {
    if (hz === void 0) {
      hz = 0;
    }
    this.m_frequencyHz = hz;
  };

  b2MouseJoint.prototype.GetDampingRatio = function() {
    return this.m_dampingRatio;
  };

  b2MouseJoint.prototype.SetDampingRatio = function(ratio) {
    if (ratio === void 0) {
      ratio = 0;
    }
    this.m_dampingRatio = ratio;
  };

  b2MouseJoint.prototype.InitVelocityConstraints = function(step) {
    var b, d, invI, invMass, k, mass, omega, rX, rY, tMat, tX;
    b = this.m_bodyB;
    mass = b.GetMass();
    omega = 2.0 * Math.PI * this.m_frequencyHz;
    d = 2.0 * mass * this.m_dampingRatio * omega;
    k = mass * omega * omega;
    this.m_gamma = step.dt * (d + step.dt * k);
    this.m_gamma = (this.m_gamma !== 0 ? 1 / this.m_gamma : 0.0);
    this.m_beta = step.dt * k * this.m_gamma;
    tMat = void 0;
    tMat = b.m_xf.R;
    rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
    rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
    tX = tMat.col1.x * rX + tMat.col2.x * rY;
    rY = tMat.col1.y * rX + tMat.col2.y * rY;
    rX = tX;
    invMass = b.m_invMass;
    invI = b.m_invI;
    this.K1.col1.x = invMass;
    this.K1.col2.x = 0.0;
    this.K1.col1.y = 0.0;
    this.K1.col2.y = invMass;
    this.K2.col1.x = invI * rY * rY;
    this.K2.col2.x = -invI * rX * rY;
    this.K2.col1.y = -invI * rX * rY;
    this.K2.col2.y = invI * rX * rX;
    this.K.SetM(this.K1);
    this.K.AddM(this.K2);
    this.K.col1.x += this.m_gamma;
    this.K.col2.y += this.m_gamma;
    this.K.GetInverse(this.m_mass);
    this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
    this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;
    b.m_angularVelocity *= 0.98;
    this.m_impulse.x *= step.dtRatio;
    this.m_impulse.y *= step.dtRatio;
    b.m_linearVelocity.x += invMass * this.m_impulse.x;
    b.m_linearVelocity.y += invMass * this.m_impulse.y;
    b.m_angularVelocity += invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
  };

  b2MouseJoint.prototype.SolveVelocityConstraints = function(step) {
    var CdotX, CdotY, b, impulseX, impulseY, maxImpulse, oldImpulseX, oldImpulseY, rX, rY, tMat, tX, tY;
    b = this.m_bodyB;
    tMat = void 0;
    tX = 0;
    tY = 0;
    tMat = b.m_xf.R;
    rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
    rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
    tX = tMat.col1.x * rX + tMat.col2.x * rY;
    rY = tMat.col1.y * rX + tMat.col2.y * rY;
    rX = tX;
    CdotX = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
    CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
    tMat = this.m_mass;
    tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
    tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
    impulseX = -(tMat.col1.x * tX + tMat.col2.x * tY);
    impulseY = -(tMat.col1.y * tX + tMat.col2.y * tY);
    oldImpulseX = this.m_impulse.x;
    oldImpulseY = this.m_impulse.y;
    this.m_impulse.x += impulseX;
    this.m_impulse.y += impulseY;
    maxImpulse = step.dt * this.m_maxForce;
    if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
      this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
    }
    impulseX = this.m_impulse.x - oldImpulseX;
    impulseY = this.m_impulse.y - oldImpulseY;
    b.m_linearVelocity.x += b.m_invMass * impulseX;
    b.m_linearVelocity.y += b.m_invMass * impulseY;
    b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
  };

  b2MouseJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    return true;
  };

  return b2MouseJoint;

})(b2Joint);

//# sourceMappingURL=b2MouseJoint.js.map

},{"../../index":103}],94:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2MouseJointDef = (function(_super) {
  __extends(b2MouseJointDef, _super);

  b2MouseJointDef.prototype.type = b2Joint.e_mouseJoint;

  b2MouseJointDef.prototype.target = null;

  b2MouseJointDef.prototype.maxForce = 0.0;

  b2MouseJointDef.prototype.frequencyHz = 5.0;

  b2MouseJointDef.prototype.dampingRatio = 0.7;

  function b2MouseJointDef() {
    this.target = new b2Vec2();
    return;
  }

  return b2MouseJointDef;

})(b2JointDef);

//# sourceMappingURL=b2MouseJointDef.js.map

},{"../../index":103}],95:[function(require,module,exports){
var Box2D, b2Joint, b2Mat22, b2Math, b2Vec2, b2Vec3,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec3 = Box2D.Common.Math.b2Vec3;

b2Math = Box2D.Common.Math.b2Math;

Box2D.Dynamics.Joints.b2PrismaticJoint = (function(_super) {
  __extends(b2PrismaticJoint, _super);

  b2PrismaticJoint.prototype.m_localXAxis1 = null;

  b2PrismaticJoint.prototype.m_localYAxis1 = null;

  b2PrismaticJoint.prototype.m_axis = null;

  b2PrismaticJoint.prototype.m_perp = null;

  b2PrismaticJoint.prototype.m_K = null;

  b2PrismaticJoint.prototype.m_refAngle = null;

  b2PrismaticJoint.prototype.m_impulse = null;

  b2PrismaticJoint.prototype.m_motorMass = 0.0;

  b2PrismaticJoint.prototype.m_motorImpulse = 0.0;

  b2PrismaticJoint.prototype.m_lowerTranslation = 0.0;

  b2PrismaticJoint.prototype.m_upperTranslation = 0.0;

  b2PrismaticJoint.prototype.m_maxMotorForce = 0.0;

  b2PrismaticJoint.prototype.m_motorSpeed = 0.0;

  b2PrismaticJoint.prototype.m_enableLimit = false;

  b2PrismaticJoint.prototype.m_enableMotor = false;

  b2PrismaticJoint.prototype.m_limitState = b2Joint.e_inactiveLimit;

  function b2PrismaticJoint(def) {
    b2PrismaticJoint.__super__.constructor.apply(this, arguments);
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_localXAxis1 = new b2Vec2();
    this.m_localYAxis1 = new b2Vec2();
    this.m_axis = new b2Vec2();
    this.m_perp = new b2Vec2();
    this.m_K = new b2Mat33();
    this.m_impulse = new b2Vec3();
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_localXAxis1.SetV(def.localAxisA);
    this.m_localYAxis1.x = -this.m_localXAxis1.y;
    this.m_localYAxis1.y = this.m_localXAxis1.x;
    this.m_refAngle = def.referenceAngle;
    this.m_impulse.SetZero();
    this.m_motorMass = 0.0;
    this.m_motorImpulse = 0.0;
    this.m_lowerTranslation = def.lowerTranslation;
    this.m_upperTranslation = def.upperTranslation;
    this.m_maxMotorForce = def.maxMotorForce;
    this.m_motorSpeed = def.motorSpeed;
    this.m_enableLimit = def.enableLimit;
    this.m_enableMotor = def.enableMotor;
    this.m_limitState = b2Joint.e_inactiveLimit;
    this.m_axis.SetZero();
    this.m_perp.SetZero();
    return;
  }

  b2PrismaticJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2PrismaticJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2PrismaticJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
  };

  b2PrismaticJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return inv_dt * this.m_impulse.y;
  };

  b2PrismaticJoint.prototype.GetJointTranslation = function() {
    var axis, bA, bB, dX, dY, p1, p2, tMat, translation;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    p1 = bA.GetWorldPoint(this.m_localAnchor1);
    p2 = bB.GetWorldPoint(this.m_localAnchor2);
    dX = p2.x - p1.x;
    dY = p2.y - p1.y;
    axis = bA.GetWorldVector(this.m_localXAxis1);
    translation = axis.x * dX + axis.y * dY;
    return translation;
  };

  b2PrismaticJoint.prototype.GetJointSpeed = function() {
    var axis, bA, bB, dX, dY, p1X, p1Y, p2X, p2Y, r1X, r1Y, r2X, r2Y, speed, tMat, tX, v1, v2, w1, w2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    p1X = bA.m_sweep.c.x + r1X;
    p1Y = bA.m_sweep.c.y + r1Y;
    p2X = bB.m_sweep.c.x + r2X;
    p2Y = bB.m_sweep.c.y + r2Y;
    dX = p2X - p1X;
    dY = p2Y - p1Y;
    axis = bA.GetWorldVector(this.m_localXAxis1);
    v1 = bA.m_linearVelocity;
    v2 = bB.m_linearVelocity;
    w1 = bA.m_angularVelocity;
    w2 = bB.m_angularVelocity;
    speed = (dX * (-w1 * axis.y) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
    return speed;
  };

  b2PrismaticJoint.prototype.IsLimitEnabled = function() {
    return this.m_enableLimit;
  };

  b2PrismaticJoint.prototype.EnableLimit = function(flag) {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_enableLimit = flag;
  };

  b2PrismaticJoint.prototype.GetLowerLimit = function() {
    return this.m_lowerTranslation;
  };

  b2PrismaticJoint.prototype.GetUpperLimit = function() {
    return this.m_upperTranslation;
  };

  b2PrismaticJoint.prototype.SetLimits = function(lower, upper) {
    if (lower === void 0) {
      lower = 0;
    }
    if (upper === void 0) {
      upper = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_lowerTranslation = lower;
    this.m_upperTranslation = upper;
  };

  b2PrismaticJoint.prototype.IsMotorEnabled = function() {
    return this.m_enableMotor;
  };

  b2PrismaticJoint.prototype.EnableMotor = function(flag) {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_enableMotor = flag;
  };

  b2PrismaticJoint.prototype.SetMotorSpeed = function(speed) {
    if (speed === void 0) {
      speed = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_motorSpeed = speed;
  };

  b2PrismaticJoint.prototype.GetMotorSpeed = function() {
    return this.m_motorSpeed;
  };

  b2PrismaticJoint.prototype.SetMaxMotorForce = function(force) {
    if (force === void 0) {
      force = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_maxMotorForce = force;
  };

  b2PrismaticJoint.prototype.GetMotorForce = function() {
    return this.m_motorImpulse;
  };

  b2PrismaticJoint.prototype.InitVelocityConstraints = function(step) {
    var L1, L2, PX, PY, bA, bB, dX, dY, i1, i2, jointTransition, m1, m2, r1X, r1Y, r2X, r2Y, tMat, tX, xf1, xf2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tX = 0;
    this.m_localCenterA.SetV(bA.GetLocalCenter());
    this.m_localCenterB.SetV(bB.GetLocalCenter());
    xf1 = bA.GetTransform();
    xf2 = bB.GetTransform();
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
    r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
    r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    this.m_invMassA = bA.m_invMass;
    this.m_invMassB = bB.m_invMass;
    this.m_invIA = bA.m_invI;
    this.m_invIB = bB.m_invI;
    this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
    this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
    this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
    this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
    if (this.m_motorMass > Number.MIN_VALUE) {
      this.m_motorMass = 1.0 / this.m_motorMass;
    }
    this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
    this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
    this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
    m1 = this.m_invMassA;
    m2 = this.m_invMassB;
    i1 = this.m_invIA;
    i2 = this.m_invIB;
    this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
    this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
    this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
    this.m_K.col2.x = this.m_K.col1.y;
    this.m_K.col2.y = i1 + i2;
    this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
    this.m_K.col3.x = this.m_K.col1.z;
    this.m_K.col3.y = this.m_K.col2.z;
    this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
    if (this.m_enableLimit) {
      jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
      if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
        this.m_limitState = b2Joint.e_equalLimits;
      } else if (jointTransition <= this.m_lowerTranslation) {
        if (this.m_limitState !== b2Joint.e_atLowerLimit) {
          this.m_limitState = b2Joint.e_atLowerLimit;
          this.m_impulse.z = 0.0;
        }
      } else if (jointTransition >= this.m_upperTranslation) {
        if (this.m_limitState !== b2Joint.e_atUpperLimit) {
          this.m_limitState = b2Joint.e_atUpperLimit;
          this.m_impulse.z = 0.0;
        }
      } else {
        this.m_limitState = b2Joint.e_inactiveLimit;
        this.m_impulse.z = 0.0;
      }
    } else {
      this.m_limitState = b2Joint.e_inactiveLimit;
    }
    if (this.m_enableMotor === false) {
      this.m_motorImpulse = 0.0;
    }
    if (step.warmStarting) {
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      this.m_motorImpulse *= step.dtRatio;
      PX = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x;
      PY = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y;
      L1 = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
      L2 = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
      bA.m_linearVelocity.x -= this.m_invMassA * PX;
      bA.m_linearVelocity.y -= this.m_invMassA * PY;
      bA.m_angularVelocity -= this.m_invIA * L1;
      bB.m_linearVelocity.x += this.m_invMassB * PX;
      bB.m_linearVelocity.y += this.m_invMassB * PY;
      bB.m_angularVelocity += this.m_invIB * L2;
    } else {
      this.m_impulse.SetZero();
      this.m_motorImpulse = 0.0;
    }
  };

  b2PrismaticJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, Cdot1X, Cdot1Y, Cdot2, L1, L2, PX, PY, bA, bB, bX, bY, df, df2, f1, f2r, impulse, maxImpulse, oldImpulse, v1, v2, w1, w2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    v1 = bA.m_linearVelocity;
    w1 = bA.m_angularVelocity;
    v2 = bB.m_linearVelocity;
    w2 = bB.m_angularVelocity;
    PX = 0;
    PY = 0;
    L1 = 0;
    L2 = 0;
    if (this.m_enableMotor && this.m_limitState !== b2Joint.e_equalLimits) {
      Cdot = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
      impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
      oldImpulse = this.m_motorImpulse;
      maxImpulse = step.dt * this.m_maxMotorForce;
      this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.m_motorImpulse - oldImpulse;
      PX = impulse * this.m_axis.x;
      PY = impulse * this.m_axis.y;
      L1 = impulse * this.m_a1;
      L2 = impulse * this.m_a2;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    }
    Cdot1X = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
    Cdot1Y = w2 - w1;
    if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
      Cdot2 = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
      f1 = this.m_impulse.Copy();
      df = this.m_K.Solve33(new b2Vec3(), -Cdot1X, -Cdot1Y, -Cdot2);
      this.m_impulse.Add(df);
      if (this.m_limitState === b2Joint.e_atLowerLimit) {
        this.m_impulse.z = b2Math.Max(this.m_impulse.z, 0.0);
      } else {
        if (this.m_limitState === b2Joint.e_atUpperLimit) {
          this.m_impulse.z = b2Math.Min(this.m_impulse.z, 0.0);
        }
      }
      bX = (-Cdot1X) - (this.m_impulse.z - f1.z) * this.m_K.col3.x;
      bY = (-Cdot1Y) - (this.m_impulse.z - f1.z) * this.m_K.col3.y;
      f2r = this.m_K.Solve22(new b2Vec2(), bX, bY);
      f2r.x += f1.x;
      f2r.y += f1.y;
      this.m_impulse.x = f2r.x;
      this.m_impulse.y = f2r.y;
      df.x = this.m_impulse.x - f1.x;
      df.y = this.m_impulse.y - f1.y;
      df.z = this.m_impulse.z - f1.z;
      PX = df.x * this.m_perp.x + df.z * this.m_axis.x;
      PY = df.x * this.m_perp.y + df.z * this.m_axis.y;
      L1 = df.x * this.m_s1 + df.y + df.z * this.m_a1;
      L2 = df.x * this.m_s2 + df.y + df.z * this.m_a2;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    } else {
      df2 = this.m_K.Solve22(new b2Vec2(), -Cdot1X, -Cdot1Y);
      this.m_impulse.x += df2.x;
      this.m_impulse.y += df2.y;
      PX = df2.x * this.m_perp.x;
      PY = df2.x * this.m_perp.y;
      L1 = df2.x * this.m_s1 + df2.y;
      L2 = df2.x * this.m_s2 + df2.y;
      v1.x -= this.m_invMassA * PX;
      v1.y -= this.m_invMassA * PY;
      w1 -= this.m_invIA * L1;
      v2.x += this.m_invMassB * PX;
      v2.y += this.m_invMassB * PY;
      w2 += this.m_invIB * L2;
    }
    bA.m_linearVelocity.SetV(v1);
    bA.m_angularVelocity = w1;
    bB.m_linearVelocity.SetV(v2);
    bB.m_angularVelocity = w2;
  };

  b2PrismaticJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C1X, C1Y, C2, L1, L2, PX, PY, R1, R2, a1, a2, active, angularError, bA, bB, c1, c2, dX, dY, i1, i2, impulse, impulse1, k11, k12, k22, limitC, linearError, m1, m2, oldLimitImpulse, r1X, r1Y, r2X, r2Y, tMat, tX, translation;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    limitC = 0;
    oldLimitImpulse = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    c1 = bA.m_sweep.c;
    a1 = bA.m_sweep.a;
    c2 = bB.m_sweep.c;
    a2 = bB.m_sweep.a;
    tMat = void 0;
    tX = 0;
    m1 = 0;
    m2 = 0;
    i1 = 0;
    i2 = 0;
    linearError = 0.0;
    angularError = 0.0;
    active = false;
    C2 = 0.0;
    R1 = b2Mat22.FromAngle(a1);
    R2 = b2Mat22.FromAngle(a2);
    tMat = R1;
    r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
    r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = R2;
    r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
    r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    dX = c2.x + r2X - c1.x - r1X;
    dY = c2.y + r2Y - c1.y - r1Y;
    if (this.m_enableLimit) {
      this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
      this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
      this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
      translation = this.m_axis.x * dX + this.m_axis.y * dY;
      if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
        C2 = b2Math.Clamp(translation, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
        linearError = b2Math.Abs(translation);
        active = true;
      } else if (translation <= this.m_lowerTranslation) {
        C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
        linearError = this.m_lowerTranslation - translation;
        active = true;
      } else if (translation >= this.m_upperTranslation) {
        C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
        linearError = translation - this.m_upperTranslation;
        active = true;
      }
    }
    this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
    this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
    this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
    impulse = new b2Vec3();
    C1X = this.m_perp.x * dX + this.m_perp.y * dY;
    C1Y = a2 - a1 - this.m_refAngle;
    linearError = b2Math.Max(linearError, b2Math.Abs(C1X));
    angularError = b2Math.Abs(C1Y);
    if (active) {
      m1 = this.m_invMassA;
      m2 = this.m_invMassB;
      i1 = this.m_invIA;
      i2 = this.m_invIB;
      this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
      this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
      this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
      this.m_K.col2.x = this.m_K.col1.y;
      this.m_K.col2.y = i1 + i2;
      this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
      this.m_K.col3.x = this.m_K.col1.z;
      this.m_K.col3.y = this.m_K.col2.z;
      this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
      this.m_K.Solve33(impulse, -C1X, -C1Y, -C2);
    } else {
      m1 = this.m_invMassA;
      m2 = this.m_invMassB;
      i1 = this.m_invIA;
      i2 = this.m_invIB;
      k11 = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
      k12 = i1 * this.m_s1 + i2 * this.m_s2;
      k22 = i1 + i2;
      this.m_K.col1.Set(k11, k12, 0.0);
      this.m_K.col2.Set(k12, k22, 0.0);
      impulse1 = this.m_K.Solve22(new b2Vec2(), -C1X, -C1Y);
      impulse.x = impulse1.x;
      impulse.y = impulse1.y;
      impulse.z = 0.0;
    }
    PX = impulse.x * this.m_perp.x + impulse.z * this.m_axis.x;
    PY = impulse.x * this.m_perp.y + impulse.z * this.m_axis.y;
    L1 = impulse.x * this.m_s1 + impulse.y + impulse.z * this.m_a1;
    L2 = impulse.x * this.m_s2 + impulse.y + impulse.z * this.m_a2;
    c1.x -= this.m_invMassA * PX;
    c1.y -= this.m_invMassA * PY;
    a1 -= this.m_invIA * L1;
    c2.x += this.m_invMassB * PX;
    c2.y += this.m_invMassB * PY;
    a2 += this.m_invIB * L2;
    bA.m_sweep.a = a1;
    bB.m_sweep.a = a2;
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
  };

  return b2PrismaticJoint;

})(b2Joint);

//# sourceMappingURL=b2PrismaticJoint.js.map

},{"../../index":103}],96:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2PrismaticJointDef = (function(_super) {
  __extends(b2PrismaticJointDef, _super);

  b2PrismaticJointDef.prototype.type = b2Joint.e_prismaticJoint;

  b2PrismaticJointDef.prototype.localAnchorA = null;

  b2PrismaticJointDef.prototype.localAnchorB = null;

  b2PrismaticJointDef.prototype.localAxisA = null;

  b2PrismaticJointDef.prototype.referenceAngle = 0.0;

  b2PrismaticJointDef.prototype.enableLimit = false;

  b2PrismaticJointDef.prototype.lowerTranslation = 0.0;

  b2PrismaticJointDef.prototype.upperTranslation = 0.0;

  b2PrismaticJointDef.prototype.enableMotor = false;

  b2PrismaticJointDef.prototype.maxMotorForce = 0.0;

  b2PrismaticJointDef.prototype.motorSpeed = 0.0;

  function b2PrismaticJointDef() {
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.localAxisA = new b2Vec2();
    this.type = b2Joint.e_prismaticJoint;
    this.localAxisA.Set(1.0, 0.0);
    return;
  }

  b2PrismaticJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.localAxisA = this.bodyA.GetLocalVector(axis);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };

  return b2PrismaticJointDef;

})(b2JointDef);

//# sourceMappingURL=b2PrismaticJointDef.js.map

},{"../../index":103}],97:[function(require,module,exports){
var Box2D, b2Joint, b2Mat22, b2Math, b2Vec2, b2Vec3,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec3 = Box2D.Common.Math.b2Vec3;

b2Math = Box2D.Common.Math.b2Math;

Box2D.Dynamics.Joints.b2PulleyJoint = (function(_super) {
  __extends(b2PulleyJoint, _super);

  b2PulleyJoint.b2_minPulleyLength = 2.0;

  b2PulleyJoint.prototype.m_localXAxis1 = null;

  b2PulleyJoint.prototype.m_localYAxis1 = null;

  b2PulleyJoint.prototype.m_u1 = null;

  b2PulleyJoint.prototype.m_u2 = null;

  b2PulleyJoint.prototype.m_ground = null;

  b2PulleyJoint.prototype.m_ratio = 0.0;

  b2PulleyJoint.prototype.m_constant = 0.0;

  b2PulleyJoint.prototype.m_maxLength1 = 0.0;

  b2PulleyJoint.prototype.m_maxLength2 = 0.0;

  b2PulleyJoint.prototype.m_impulse = 0.0;

  b2PulleyJoint.prototype.m_limitImpulse1 = 0.0;

  b2PulleyJoint.prototype.m_limitImpulse2 = 0.0;

  function b2PulleyJoint(def) {
    b2PulleyJoint.__super__.constructor.call(this, def);
    this.m_groundAnchor1 = new b2Vec2();
    this.m_groundAnchor2 = new b2Vec2();
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_u1 = new b2Vec2();
    this.m_u2 = new b2Vec2();
    this.m_ground = this.m_bodyA.m_world.m_groundBody;
    this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
    this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_ratio = def.ratio;
    this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
    this.m_maxLength1 = b2Math.Min(def.maxLengthA, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
    this.m_maxLength2 = b2Math.Min(def.maxLengthB, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);
    this.m_impulse = 0.0;
    this.m_limitImpulse1 = 0.0;
    this.m_limitImpulse2 = 0.0;
    return;
  }

  b2PulleyJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2PulleyJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2PulleyJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
  };

  b2PulleyJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return 0.0;
  };

  b2PulleyJoint.prototype.GetGroundAnchorA = function() {
    var a;
    a = this.m_ground.m_xf.position.Copy();
    a.Add(this.m_groundAnchor1);
    return a;
  };

  b2PulleyJoint.prototype.GetGroundAnchorB = function() {
    var a;
    a = this.m_ground.m_xf.position.Copy();
    a.Add(this.m_groundAnchor2);
    return a;
  };

  b2PulleyJoint.prototype.GetLength1 = function() {
    var dX, dY, p, sX, sY;
    p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    dX = p.x - sX;
    dY = p.y - sY;
    return Math.sqrt(dX * dX + dY * dY);
  };

  b2PulleyJoint.prototype.GetLength2 = function() {
    var dX, dY, p, sX, sY;
    p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    dX = p.x - sX;
    dY = p.y - sY;
    return Math.sqrt(dX * dX + dY * dY);
  };

  b2PulleyJoint.prototype.GetRatio = function() {
    return this.m_ratio;
  };

  b2PulleyJoint.prototype.InitVelocityConstraints = function(step) {
    var C, P1X, P1Y, P2X, P2Y, bA, bB, cr1u1, cr2u2, length1, length2, p1X, p1Y, p2X, p2Y, r1X, r1Y, r2X, r2Y, s1X, s1Y, s2X, s2Y, tMat, tX;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    p1X = bA.m_sweep.c.x + r1X;
    p1Y = bA.m_sweep.c.y + r1Y;
    p2X = bB.m_sweep.c.x + r2X;
    p2Y = bB.m_sweep.c.y + r2Y;
    s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    this.m_u1.Set(p1X - s1X, p1Y - s1Y);
    this.m_u2.Set(p2X - s2X, p2Y - s2Y);
    length1 = this.m_u1.Length();
    length2 = this.m_u2.Length();
    if (length1 > b2Settings.b2_linearSlop) {
      this.m_u1.Multiply(1.0 / length1);
    } else {
      this.m_u1.SetZero();
    }
    if (length2 > b2Settings.b2_linearSlop) {
      this.m_u2.Multiply(1.0 / length2);
    } else {
      this.m_u2.SetZero();
    }
    C = this.m_constant - length1 - this.m_ratio * length2;
    if (C > 0.0) {
      this.m_state = b2Joint.e_inactiveLimit;
      this.m_impulse = 0.0;
    } else {
      this.m_state = b2Joint.e_atUpperLimit;
    }
    if (length1 < this.m_maxLength1) {
      this.m_limitState1 = b2Joint.e_inactiveLimit;
      this.m_limitImpulse1 = 0.0;
    } else {
      this.m_limitState1 = b2Joint.e_atUpperLimit;
    }
    if (length2 < this.m_maxLength2) {
      this.m_limitState2 = b2Joint.e_inactiveLimit;
      this.m_limitImpulse2 = 0.0;
    } else {
      this.m_limitState2 = b2Joint.e_atUpperLimit;
    }
    cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
    cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
    this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
    this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
    this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
    this.m_limitMass1 = 1.0 / this.m_limitMass1;
    this.m_limitMass2 = 1.0 / this.m_limitMass2;
    this.m_pulleyMass = 1.0 / this.m_pulleyMass;
    if (step.warmStarting) {
      this.m_impulse *= step.dtRatio;
      this.m_limitImpulse1 *= step.dtRatio;
      this.m_limitImpulse2 *= step.dtRatio;
      P1X = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.x;
      P1Y = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.y;
      P2X = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.x;
      P2Y = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.y;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    } else {
      this.m_impulse = 0.0;
      this.m_limitImpulse1 = 0.0;
      this.m_limitImpulse2 = 0.0;
    }
  };

  b2PulleyJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, P1X, P1Y, P2X, P2Y, bA, bB, impulse, oldImpulse, r1X, r1Y, r2X, r2Y, tMat, tX, v1X, v1Y, v2X, v2Y;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    v1X = 0;
    v1Y = 0;
    v2X = 0;
    v2Y = 0;
    P1X = 0;
    P1Y = 0;
    P2X = 0;
    P2Y = 0;
    Cdot = 0;
    impulse = 0;
    oldImpulse = 0;
    if (this.m_state === b2Joint.e_atUpperLimit) {
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
      Cdot = (-(this.m_u1.x * v1X + this.m_u1.y * v1Y)) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
      impulse = this.m_pulleyMass * (-Cdot);
      oldImpulse = this.m_impulse;
      this.m_impulse = b2Math.Max(0.0, this.m_impulse + impulse);
      impulse = this.m_impulse - oldImpulse;
      P1X = -impulse * this.m_u1.x;
      P1Y = -impulse * this.m_u1.y;
      P2X = -this.m_ratio * impulse * this.m_u2.x;
      P2Y = -this.m_ratio * impulse * this.m_u2.y;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }
    if (this.m_limitState1 === b2Joint.e_atUpperLimit) {
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
      Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);
      impulse = -this.m_limitMass1 * Cdot;
      oldImpulse = this.m_limitImpulse1;
      this.m_limitImpulse1 = b2Math.Max(0.0, this.m_limitImpulse1 + impulse);
      impulse = this.m_limitImpulse1 - oldImpulse;
      P1X = -impulse * this.m_u1.x;
      P1Y = -impulse * this.m_u1.y;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
    }
    if (this.m_limitState2 === b2Joint.e_atUpperLimit) {
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
      Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);
      impulse = -this.m_limitMass2 * Cdot;
      oldImpulse = this.m_limitImpulse2;
      this.m_limitImpulse2 = b2Math.Max(0.0, this.m_limitImpulse2 + impulse);
      impulse = this.m_limitImpulse2 - oldImpulse;
      P2X = -impulse * this.m_u2.x;
      P2Y = -impulse * this.m_u2.y;
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }
  };

  b2PulleyJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C, bA, bB, impulse, length1, length2, linearError, oldImpulse, oldLimitPositionImpulse, p1X, p1Y, p2X, p2Y, r1X, r1Y, r2X, r2Y, s1X, s1Y, s2X, s2Y, tMat, tX;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
    s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
    s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
    s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    r1X = 0;
    r1Y = 0;
    r2X = 0;
    r2Y = 0;
    p1X = 0;
    p1Y = 0;
    p2X = 0;
    p2Y = 0;
    length1 = 0;
    length2 = 0;
    C = 0;
    impulse = 0;
    oldImpulse = 0;
    oldLimitPositionImpulse = 0;
    tX = 0;
    linearError = 0.0;
    if (this.m_state === b2Joint.e_atUpperLimit) {
      tMat = bA.m_xf.R;
      r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      p1X = bA.m_sweep.c.x + r1X;
      p1Y = bA.m_sweep.c.y + r1Y;
      p2X = bB.m_sweep.c.x + r2X;
      p2Y = bB.m_sweep.c.y + r2Y;
      this.m_u1.Set(p1X - s1X, p1Y - s1Y);
      this.m_u2.Set(p2X - s2X, p2Y - s2Y);
      length1 = this.m_u1.Length();
      length2 = this.m_u2.Length();
      if (length1 > b2Settings.b2_linearSlop) {
        this.m_u1.Multiply(1.0 / length1);
      } else {
        this.m_u1.SetZero();
      }
      if (length2 > b2Settings.b2_linearSlop) {
        this.m_u2.Multiply(1.0 / length2);
      } else {
        this.m_u2.SetZero();
      }
      C = this.m_constant - length1 - this.m_ratio * length2;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -this.m_pulleyMass * C;
      p1X = -impulse * this.m_u1.x;
      p1Y = -impulse * this.m_u1.y;
      p2X = -this.m_ratio * impulse * this.m_u2.x;
      p2Y = -this.m_ratio * impulse * this.m_u2.y;
      bA.m_sweep.c.x += bA.m_invMass * p1X;
      bA.m_sweep.c.y += bA.m_invMass * p1Y;
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
      bB.m_sweep.c.x += bB.m_invMass * p2X;
      bB.m_sweep.c.y += bB.m_invMass * p2Y;
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
    }
    if (this.m_limitState1 === b2Joint.e_atUpperLimit) {
      tMat = bA.m_xf.R;
      r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      p1X = bA.m_sweep.c.x + r1X;
      p1Y = bA.m_sweep.c.y + r1Y;
      this.m_u1.Set(p1X - s1X, p1Y - s1Y);
      length1 = this.m_u1.Length();
      if (length1 > b2Settings.b2_linearSlop) {
        this.m_u1.x *= 1.0 / length1;
        this.m_u1.y *= 1.0 / length1;
      } else {
        this.m_u1.SetZero();
      }
      C = this.m_maxLength1 - length1;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -this.m_limitMass1 * C;
      p1X = -impulse * this.m_u1.x;
      p1Y = -impulse * this.m_u1.y;
      bA.m_sweep.c.x += bA.m_invMass * p1X;
      bA.m_sweep.c.y += bA.m_invMass * p1Y;
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
      bA.SynchronizeTransform();
    }
    if (this.m_limitState2 === b2Joint.e_atUpperLimit) {
      tMat = bB.m_xf.R;
      r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      p2X = bB.m_sweep.c.x + r2X;
      p2Y = bB.m_sweep.c.y + r2Y;
      this.m_u2.Set(p2X - s2X, p2Y - s2Y);
      length2 = this.m_u2.Length();
      if (length2 > b2Settings.b2_linearSlop) {
        this.m_u2.x *= 1.0 / length2;
        this.m_u2.y *= 1.0 / length2;
      } else {
        this.m_u2.SetZero();
      }
      C = this.m_maxLength2 - length2;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -this.m_limitMass2 * C;
      p2X = -impulse * this.m_u2.x;
      p2Y = -impulse * this.m_u2.y;
      bB.m_sweep.c.x += bB.m_invMass * p2X;
      bB.m_sweep.c.y += bB.m_invMass * p2Y;
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
      bB.SynchronizeTransform();
    }
    return linearError < b2Settings.b2_linearSlop;
  };

  return b2PulleyJoint;

})(b2Joint);

//# sourceMappingURL=b2PulleyJoint.js.map

},{"../../index":103}],98:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2PulleyJointDef = (function(_super) {
  __extends(b2PulleyJointDef, _super);

  b2PulleyJointDef.prototype.type = b2Joint.e_pulleyJoint;

  b2PulleyJointDef.prototype.groundAnchorA = null;

  b2PulleyJointDef.prototype.groundAnchorB = null;

  b2PulleyJointDef.prototype.localAnchorA = null;

  b2PulleyJointDef.prototype.localAnchorB = null;

  b2PulleyJointDef.prototype.lengthA = 0.0;

  b2PulleyJointDef.prototype.maxLengthA = 0.0;

  b2PulleyJointDef.prototype.lengthB = 0.0;

  b2PulleyJointDef.prototype.maxLengthB = 0.0;

  b2PulleyJointDef.prototype.ratio = 1.0;

  b2PulleyJointDef.prototype.collideConnected = true;

  function b2PulleyJointDef() {
    b2PulleyJointDef.__super__.constructor.apply(this, arguments);
    this.groundAnchorA = new b2Vec2();
    this.groundAnchorB = new b2Vec2();
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.groundAnchorA.Set(-1.0, 1.0);
    this.groundAnchorB.Set(1.0, 1.0);
    this.localAnchorA.Set(-1.0, 0.0);
    this.localAnchorB.Set(1.0, 0.0);
    return;
  }

  b2PulleyJointDef.prototype.Initialize = function(bA, bB, gaA, gaB, anchorA, anchorB, r) {
    var C, d1X, d1Y, d2X, d2Y;
    if (r === void 0) {
      r = 0;
    }
    this.bodyA = bA;
    this.bodyB = bB;
    this.groundAnchorA.SetV(gaA);
    this.groundAnchorB.SetV(gaB);
    this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
    d1X = anchorA.x - gaA.x;
    d1Y = anchorA.y - gaA.y;
    this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
    d2X = anchorB.x - gaB.x;
    d2Y = anchorB.y - gaB.y;
    this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
    this.ratio = r;
    C = this.lengthA + this.ratio * this.lengthB;
    this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
    this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
  };

  return b2PulleyJointDef;

})(b2JointDef);

//# sourceMappingURL=b2PulleyJointDef.js.map

},{"../../index":103}],99:[function(require,module,exports){
var Box2D, b2Joint, b2Mat22, b2Math, b2Vec2, b2Vec3,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Mat22 = Box2D.Common.Math.b2Mat22;

b2Vec3 = Box2D.Common.Math.b2Vec3;

b2Math = Box2D.Common.Math.b2Math;

Box2D.Dynamics.Joints.b2RevoluteJoint = (function(_super) {
  __extends(b2RevoluteJoint, _super);

  b2RevoluteJoint.tImpulse = new b2Vec2();

  b2RevoluteJoint.prototype.K = null;

  b2RevoluteJoint.prototype.K1 = null;

  b2RevoluteJoint.prototype.K2 = null;

  b2RevoluteJoint.prototype.K3 = null;

  b2RevoluteJoint.prototype.impulse2 = null;

  b2RevoluteJoint.prototype.impulse3 = null;

  b2RevoluteJoint.prototype.reduced = null;

  b2RevoluteJoint.prototype.m_impulse = null;

  b2RevoluteJoint.prototype.m_mass = 0.0;

  b2RevoluteJoint.prototype.m_referenceAngle = 0.0;

  b2RevoluteJoint.prototype.m_motorImpulse = 0.0;

  b2RevoluteJoint.prototype.m_lowerAngle = 0.0;

  b2RevoluteJoint.prototype.m_upperAngle = 0.0;

  b2RevoluteJoint.prototype.m_maxMotorTorque = 0.0;

  b2RevoluteJoint.prototype.m_motorSpeed = 0.0;

  b2RevoluteJoint.prototype.m_enableLimit = 0.0;

  b2RevoluteJoint.prototype.m_enableMotor = false;

  b2RevoluteJoint.prototype.m_limitState = b2Joint.e_inactiveLimit;

  function b2RevoluteJoint(def) {
    b2RevoluteJoint.__super__.constructor.call(this, def);
    this.K = new b2Mat22();
    this.K1 = new b2Mat22();
    this.K2 = new b2Mat22();
    this.K3 = new b2Mat22();
    this.impulse3 = new b2Vec3();
    this.impulse2 = new b2Vec2();
    this.reduced = new b2Vec2();
    this.m_localAnchor1 = new b2Vec2();
    this.m_localAnchor2 = new b2Vec2();
    this.m_impulse = new b2Vec3();
    this.m_mass = new b2Mat33();
    this.m_localAnchor1.SetV(def.localAnchorA);
    this.m_localAnchor2.SetV(def.localAnchorB);
    this.m_referenceAngle = def.referenceAngle;
    this.m_impulse.SetZero();
    this.m_lowerAngle = def.lowerAngle;
    this.m_upperAngle = def.upperAngle;
    this.m_maxMotorTorque = def.maxMotorTorque;
    this.m_motorSpeed = def.motorSpeed;
    this.m_enableLimit = def.enableLimit;
    this.m_enableMotor = def.enableMotor;
    return;
  }

  b2RevoluteJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };

  b2RevoluteJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };

  b2RevoluteJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
  };

  b2RevoluteJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return inv_dt * this.m_impulse.z;
  };

  b2RevoluteJoint.prototype.GetJointAngle = function() {
    return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
  };

  b2RevoluteJoint.prototype.GetJointSpeed = function() {
    return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
  };

  b2RevoluteJoint.prototype.IsLimitEnabled = function() {
    return this.m_enableLimit;
  };

  b2RevoluteJoint.prototype.EnableLimit = function(flag) {
    this.m_enableLimit = flag;
  };

  b2RevoluteJoint.prototype.GetLowerLimit = function() {
    return this.m_lowerAngle;
  };

  b2RevoluteJoint.prototype.GetUpperLimit = function() {
    return this.m_upperAngle;
  };

  b2RevoluteJoint.prototype.SetLimits = function(lower, upper) {
    if (lower === void 0) {
      lower = 0;
    }
    if (upper === void 0) {
      upper = 0;
    }
    this.m_lowerAngle = lower;
    this.m_upperAngle = upper;
  };

  b2RevoluteJoint.prototype.IsMotorEnabled = function() {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    return this.m_enableMotor;
  };

  b2RevoluteJoint.prototype.EnableMotor = function(flag) {
    this.m_enableMotor = flag;
  };

  b2RevoluteJoint.prototype.SetMotorSpeed = function(speed) {
    if (speed === void 0) {
      speed = 0;
    }
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_motorSpeed = speed;
  };

  b2RevoluteJoint.prototype.GetMotorSpeed = function() {
    return this.m_motorSpeed;
  };

  b2RevoluteJoint.prototype.SetMaxMotorTorque = function(torque) {
    if (torque === void 0) {
      torque = 0;
    }
    this.m_maxMotorTorque = torque;
  };

  b2RevoluteJoint.prototype.GetMotorTorque = function() {
    return this.m_maxMotorTorque;
  };

  b2RevoluteJoint.prototype.InitVelocityConstraints = function(step) {
    var PX, PY, bA, bB, i1, i2, jointAngle, m1, m2, r1X, r1Y, r2X, r2Y, tMat, tX;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tX = 0;

    /**
    todo: fix this
     */
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    m1 = bA.m_invMass;
    m2 = bB.m_invMass;
    i1 = bA.m_invI;
    i2 = bB.m_invI;
    this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
    this.m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2;
    this.m_mass.col3.x = (-r1Y * i1) - r2Y * i2;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
    this.m_mass.col3.y = r1X * i1 + r2X * i2;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = i1 + i2;
    this.m_motorMass = 1.0 / (i1 + i2);
    if (this.m_enableMotor === false) {
      this.m_motorImpulse = 0.0;
    }
    if (this.m_enableLimit) {
      jointAngle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
      if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop) {
        this.m_limitState = b2Joint.e_equalLimits;
      } else if (jointAngle <= this.m_lowerAngle) {
        if (this.m_limitState !== b2Joint.e_atLowerLimit) {
          this.m_impulse.z = 0.0;
        }
        this.m_limitState = b2Joint.e_atLowerLimit;
      } else if (jointAngle >= this.m_upperAngle) {
        if (this.m_limitState !== b2Joint.e_atUpperLimit) {
          this.m_impulse.z = 0.0;
        }
        this.m_limitState = b2Joint.e_atUpperLimit;
      } else {
        this.m_limitState = b2Joint.e_inactiveLimit;
        this.m_impulse.z = 0.0;
      }
    } else {
      this.m_limitState = b2Joint.e_inactiveLimit;
    }
    if (step.warmStarting) {
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      this.m_motorImpulse *= step.dtRatio;
      PX = this.m_impulse.x;
      PY = this.m_impulse.y;
      bA.m_linearVelocity.x -= m1 * PX;
      bA.m_linearVelocity.y -= m1 * PY;
      bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.m_motorImpulse + this.m_impulse.z);
      bB.m_linearVelocity.x += m2 * PX;
      bB.m_linearVelocity.y += m2 * PY;
      bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.m_motorImpulse + this.m_impulse.z);
    } else {
      this.m_impulse.SetZero();
      this.m_motorImpulse = 0.0;
    }
  };

  b2RevoluteJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot, Cdot1X, Cdot1Y, Cdot2, CdotX, CdotY, bA, bB, i1, i2, impulse, m1, m2, maxImpulse, newImpulse, oldImpulse, r1X, r1Y, r2X, r2Y, tMat, tX, v1, v2, w1, w2;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = void 0;
    tX = 0;
    newImpulse = 0;
    r1X = 0;
    r1Y = 0;
    r2X = 0;
    r2Y = 0;
    v1 = bA.m_linearVelocity;
    w1 = bA.m_angularVelocity;
    v2 = bB.m_linearVelocity;
    w2 = bB.m_angularVelocity;
    m1 = bA.m_invMass;
    m2 = bB.m_invMass;
    i1 = bA.m_invI;
    i2 = bB.m_invI;
    if (this.m_enableMotor && this.m_limitState !== b2Joint.e_equalLimits) {
      Cdot = w2 - w1 - this.m_motorSpeed;
      impulse = this.m_motorMass * (-Cdot);
      oldImpulse = this.m_motorImpulse;
      maxImpulse = step.dt * this.m_maxMotorTorque;
      this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.m_motorImpulse - oldImpulse;
      w1 -= i1 * impulse;
      w2 += i2 * impulse;
    }
    if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
      tMat = bA.m_xf.R;
      r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      Cdot1X = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y);
      Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
      Cdot2 = w2 - w1;
      this.m_mass.Solve33(this.impulse3, -Cdot1X, -Cdot1Y, -Cdot2);
      if (this.m_limitState === b2Joint.e_equalLimits) {
        this.m_impulse.Add(this.impulse3);
      } else if (this.m_limitState === b2Joint.e_atLowerLimit) {
        newImpulse = this.m_impulse.z + this.impulse3.z;
        if (newImpulse < 0.0) {
          this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
          this.impulse3.x = this.reduced.x;
          this.impulse3.y = this.reduced.y;
          this.impulse3.z = -this.m_impulse.z;
          this.m_impulse.x += this.reduced.x;
          this.m_impulse.y += this.reduced.y;
          this.m_impulse.z = 0.0;
        }
      } else if (this.m_limitState === b2Joint.e_atUpperLimit) {
        newImpulse = this.m_impulse.z + this.impulse3.z;
        if (newImpulse > 0.0) {
          this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
          this.impulse3.x = this.reduced.x;
          this.impulse3.y = this.reduced.y;
          this.impulse3.z = -this.m_impulse.z;
          this.m_impulse.x += this.reduced.x;
          this.m_impulse.y += this.reduced.y;
          this.m_impulse.z = 0.0;
        }
      }
      v1.x -= m1 * this.impulse3.x;
      v1.y -= m1 * this.impulse3.y;
      w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
      v2.x += m2 * this.impulse3.x;
      v2.y += m2 * this.impulse3.y;
      w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
    } else {
      tMat = bA.m_xf.R;
      r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      CdotX = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y);
      CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
      this.m_mass.Solve22(this.impulse2, -CdotX, -CdotY);
      this.m_impulse.x += this.impulse2.x;
      this.m_impulse.y += this.impulse2.y;
      v1.x -= m1 * this.impulse2.x;
      v1.y -= m1 * this.impulse2.y;
      w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
      v2.x += m2 * this.impulse2.x;
      v2.y += m2 * this.impulse2.y;
      w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
    }
    bA.m_linearVelocity.SetV(v1);
    bA.m_angularVelocity = w1;
    bB.m_linearVelocity.SetV(v2);
    bB.m_angularVelocity = w2;
  };

  b2RevoluteJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C, CLength, CLengthSquared, CX, CY, angle, angularError, bA, bB, impulseX, impulseY, invI1, invI2, invMass1, invMass2, k, k_allowedStretch, k_beta, limitImpulse, m, oldLimitImpulse, positionError, r1X, r1Y, r2X, r2Y, tMat, tX, uX, uY;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    oldLimitImpulse = 0;
    C = 0;
    tMat = void 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    angularError = 0.0;
    positionError = 0.0;
    tX = 0;
    impulseX = 0;
    impulseY = 0;
    if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
      angle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
      limitImpulse = 0.0;
      if (this.m_limitState === b2Joint.e_equalLimits) {
        C = b2Math.Clamp(angle - this.m_lowerAngle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
        limitImpulse = -this.m_motorMass * C;
        angularError = b2Math.Abs(C);
      } else if (this.m_limitState === b2Joint.e_atLowerLimit) {
        C = angle - this.m_lowerAngle;
        angularError = -C;
        C = b2Math.Clamp(C + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0.0);
        limitImpulse = -this.m_motorMass * C;
      } else if (this.m_limitState === b2Joint.e_atUpperLimit) {
        C = angle - this.m_upperAngle;
        angularError = C;
        C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
        limitImpulse = -this.m_motorMass * C;
      }
      bA.m_sweep.a -= bA.m_invI * limitImpulse;
      bB.m_sweep.a += bB.m_invI * limitImpulse;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
    }
    tMat = bA.m_xf.R;
    r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
    r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
    r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
    r1X = tX;
    tMat = bB.m_xf.R;
    r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
    r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
    r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
    r2X = tX;
    CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
    CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    CLengthSquared = CX * CX + CY * CY;
    CLength = Math.sqrt(CLengthSquared);
    positionError = CLength;
    invMass1 = bA.m_invMass;
    invMass2 = bB.m_invMass;
    invI1 = bA.m_invI;
    invI2 = bB.m_invI;
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
    if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
      uX = CX / CLength;
      uY = CY / CLength;
      k = invMass1 + invMass2;
      m = 1.0 / k;
      impulseX = m * (-CX);
      impulseY = m * (-CY);
      k_beta = 0.5;
      bA.m_sweep.c.x -= k_beta * invMass1 * impulseX;
      bA.m_sweep.c.y -= k_beta * invMass1 * impulseY;
      bB.m_sweep.c.x += k_beta * invMass2 * impulseX;
      bB.m_sweep.c.y += k_beta * invMass2 * impulseY;
      CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
      CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
    }
    this.K1.col1.x = invMass1 + invMass2;
    this.K1.col2.x = 0.0;
    this.K1.col1.y = 0.0;
    this.K1.col2.y = invMass1 + invMass2;
    this.K2.col1.x = invI1 * r1Y * r1Y;
    this.K2.col2.x = -invI1 * r1X * r1Y;
    this.K2.col1.y = -invI1 * r1X * r1Y;
    this.K2.col2.y = invI1 * r1X * r1X;
    this.K3.col1.x = invI2 * r2Y * r2Y;
    this.K3.col2.x = -invI2 * r2X * r2Y;
    this.K3.col1.y = -invI2 * r2X * r2Y;
    this.K3.col2.y = invI2 * r2X * r2X;
    this.K.SetM(this.K1);
    this.K.AddM(this.K2);
    this.K.AddM(this.K3);
    this.K.Solve(b2RevoluteJoint.tImpulse, -CX, -CY);
    impulseX = b2RevoluteJoint.tImpulse.x;
    impulseY = b2RevoluteJoint.tImpulse.y;
    bA.m_sweep.c.x -= bA.m_invMass * impulseX;
    bA.m_sweep.c.y -= bA.m_invMass * impulseY;
    bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX);
    bB.m_sweep.c.x += bB.m_invMass * impulseX;
    bB.m_sweep.c.y += bB.m_invMass * impulseY;
    bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX);
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
  };

  return b2RevoluteJoint;

})(b2Joint);

//# sourceMappingURL=b2RevoluteJoint.js.map

},{"../../index":103}],100:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2RevoluteJointDef = (function(_super) {
  __extends(b2RevoluteJointDef, _super);

  b2RevoluteJointDef.prototype.type = b2Joint.e_revoluteJoint;

  b2RevoluteJointDef.prototype.localAnchorA = null;

  b2RevoluteJointDef.prototype.localAnchorB = null;

  b2RevoluteJointDef.prototype.referenceAngle = 0.0;

  b2RevoluteJointDef.prototype.lowerAngle = 0.0;

  b2RevoluteJointDef.prototype.upperAngle = 0.0;

  b2RevoluteJointDef.prototype.maxMotorTorque = 0.0;

  b2RevoluteJointDef.prototype.motorSpeed = 0.0;

  b2RevoluteJointDef.prototype.enableLimit = false;

  b2RevoluteJointDef.prototype.enableMotor = false;

  function b2RevoluteJointDef() {
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    this.localAnchorA.Set(0.0, 0.0);
    this.localAnchorB.Set(0.0, 0.0);
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

})(b2JointDef);

//# sourceMappingURL=b2RevoluteJointDef.js.map

},{"../../index":103}],101:[function(require,module,exports){
var Box2D, b2Joint, b2Mat33, b2Vec2, b2Vec3,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2Vec3 = Box2D.Common.Math.b2Vec3;

b2Mat33 = Box2D.Common.Math.b2Mat33;

b2Joint = Box2D.Dynamics.Joints.b2Joint;

Box2D.Dynamics.Joints.b2WeldJoint = (function(_super) {
  __extends(b2WeldJoint, _super);

  b2WeldJoint.prototype.m_impulse = null;

  b2WeldJoint.prototype.m_mass = null;

  b2WeldJoint.prototype.m_referenceAngle = 0.0;

  function b2WeldJoint(def) {
    b2WeldJoint.__super__.constructor.call(this, def);
    this.m_localAnchorA = new b2Vec2();
    this.m_localAnchorB = new b2Vec2();
    this.m_impulse = new b2Vec3();
    this.m_mass = new b2Mat33();
    this.m_localAnchorA.SetV(def.localAnchorA);
    this.m_localAnchorB.SetV(def.localAnchorB);
    this.m_referenceAngle = def.referenceAngle;
    this.m_impulse.SetZero();
    this.m_mass = new b2Mat33();
    return;
  }

  b2WeldJoint.prototype.GetAnchorA = function() {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
  };

  b2WeldJoint.prototype.GetAnchorB = function() {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
  };

  b2WeldJoint.prototype.GetReactionForce = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
  };

  b2WeldJoint.prototype.GetReactionTorque = function(inv_dt) {
    if (inv_dt === void 0) {
      inv_dt = 0;
    }
    return inv_dt * this.m_impulse.z;
  };

  b2WeldJoint.prototype.InitVelocityConstraints = function(step) {
    var bA, bB, iA, iB, mA, mB, rAX, rAY, rBX, rBY, tMat, tX;
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
    rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
    rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
    rAX = tX;
    tMat = bB.m_xf.R;
    rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
    rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
    rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
    rBX = tX;
    mA = bA.m_invMass;
    mB = bB.m_invMass;
    iA = bA.m_invI;
    iB = bB.m_invI;
    this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
    this.m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
    this.m_mass.col3.x = (-rAY * iA) - rBY * iB;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
    this.m_mass.col3.y = rAX * iA + rBX * iB;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = iA + iB;
    if (step.warmStarting) {
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      this.m_impulse.z *= step.dtRatio;
      bA.m_linearVelocity.x -= mA * this.m_impulse.x;
      bA.m_linearVelocity.y -= mA * this.m_impulse.y;
      bA.m_angularVelocity -= iA * (rAX * this.m_impulse.y - rAY * this.m_impulse.x + this.m_impulse.z);
      bB.m_linearVelocity.x += mB * this.m_impulse.x;
      bB.m_linearVelocity.y += mB * this.m_impulse.y;
      bB.m_angularVelocity += iB * (rBX * this.m_impulse.y - rBY * this.m_impulse.x + this.m_impulse.z);
    } else {
      this.m_impulse.SetZero();
    }
  };

  b2WeldJoint.prototype.SolveVelocityConstraints = function(step) {
    var Cdot1X, Cdot1Y, Cdot2, bA, bB, iA, iB, impulse, mA, mB, rAX, rAY, rBX, rBY, tMat, tX, vA, vB, wA, wB;
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    vA = bA.m_linearVelocity;
    wA = bA.m_angularVelocity;
    vB = bB.m_linearVelocity;
    wB = bB.m_angularVelocity;
    mA = bA.m_invMass;
    mB = bB.m_invMass;
    iA = bA.m_invI;
    iB = bB.m_invI;
    tMat = bA.m_xf.R;
    rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
    rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
    rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
    rAX = tX;
    tMat = bB.m_xf.R;
    rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
    rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
    rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
    rBX = tX;
    Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
    Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;
    Cdot2 = wB - wA;
    impulse = new b2Vec3();
    this.m_mass.Solve33(impulse, -Cdot1X, -Cdot1Y, -Cdot2);
    this.m_impulse.Add(impulse);
    vA.x -= mA * impulse.x;
    vA.y -= mA * impulse.y;
    wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
    vB.x += mB * impulse.x;
    vB.y += mB * impulse.y;
    wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
    bA.m_angularVelocity = wA;
    bB.m_angularVelocity = wB;
  };

  b2WeldJoint.prototype.SolvePositionConstraints = function(baumgarte) {
    var C1X, C1Y, C2, angularError, bA, bB, iA, iB, impulse, k_allowedStretch, mA, mB, positionError, rAX, rAY, rBX, rBY, tMat, tX;
    if (baumgarte === void 0) {
      baumgarte = 0;
    }
    tMat = void 0;
    tX = 0;
    bA = this.m_bodyA;
    bB = this.m_bodyB;
    tMat = bA.m_xf.R;
    rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
    rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
    tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
    rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
    rAX = tX;
    tMat = bB.m_xf.R;
    rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
    rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
    tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
    rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
    rBX = tX;
    mA = bA.m_invMass;
    mB = bB.m_invMass;
    iA = bA.m_invI;
    iB = bB.m_invI;
    C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX;
    C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY;
    C2 = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
    k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
    positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
    angularError = b2Math.Abs(C2);
    if (positionError > k_allowedStretch) {
      iA *= 1.0;
      iB *= 1.0;
    }
    this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
    this.m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
    this.m_mass.col3.x = (-rAY * iA) - rBY * iB;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
    this.m_mass.col3.y = rAX * iA + rBX * iB;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = iA + iB;
    impulse = new b2Vec3();
    this.m_mass.Solve33(impulse, -C1X, -C1Y, -C2);
    bA.m_sweep.c.x -= mA * impulse.x;
    bA.m_sweep.c.y -= mA * impulse.y;
    bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
    bB.m_sweep.c.x += mB * impulse.x;
    bB.m_sweep.c.y += mB * impulse.y;
    bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
    bA.SynchronizeTransform();
    bB.SynchronizeTransform();
    return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
  };

  return b2WeldJoint;

})(b2Joint);

//# sourceMappingURL=b2WeldJoint.js.map

},{"../../index":103}],102:[function(require,module,exports){
var Box2D, b2Joint, b2JointDef, b2Vec2,
  __hasProp = {}.hasOwnProperty,
  __extends = function(child, parent) { for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; } function ctor() { this.constructor = child; } ctor.prototype = parent.prototype; child.prototype = new ctor(); child.__super__ = parent.prototype; return child; };

Box2D = require('../../index');

b2Joint = Box2D.Dynamics.Joints.b2Joint;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2JointDef = Box2D.Dynamics.Joints.b2JointDef;

Box2D.Dynamics.Joints.b2WeldJointDef = (function(_super) {
  __extends(b2WeldJointDef, _super);

  b2WeldJointDef.prototype.type = b2Joint.e_weldJoint;

  b2WeldJointDef.prototype.referenceAngle = 0.0;

  b2WeldJointDef.prototype.localAnchorA = null;

  b2WeldJointDef.prototype.localAnchorB = null;

  function b2WeldJointDef() {
    this.localAnchorA = new b2Vec2();
    this.localAnchorB = new b2Vec2();
    return;
  }

  b2WeldJointDef.prototype.Initialize = function(bA, bB, anchor) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };

  return b2WeldJointDef;

})(b2JointDef);

//# sourceMappingURL=b2WeldJointDef.js.map

},{"../../index":103}],103:[function(require,module,exports){

/*
 */
'use strict';
var Box2D;

module.exports = Box2D = (function() {
  function Box2D() {}

  Box2D.parseUInt = function(v) {
    return Math.abs(parseInt(v));
  };

  Box2D.Vector = function(length) {
    var i, tmp;
    if (length == null) {
      length = 0;
    }
    tmp = new Array(length);
    i = 0;
    while (i < length) {
      tmp[i] = 0;
      ++i;
    }
    return tmp;
  };

  Box2D.equals = function(o1, o2) {
    var _ref;
    if (o1 === null) {
      return false;
    }
    if ((o2 instanceof Function) && (o1 instanceof o2)) {
      return true;
    }
    if (((_ref = o1.constructor.__implements) != null ? _ref[o2.name] : void 0)) {
      return true;
    }
    return false;
  };

  Box2D.generateCallback = function(context, cb) {
    return function() {
      cb.apply(context, arguments);
    };
  };

  return Box2D;

})();

Box2D.Common = (function() {
  function Common() {}

  return Common;

})();

Box2D.Common.Math = (function() {
  function Math() {}

  return Math;

})();

require('./common/math/b2Vec2');

require('./common/math/b2Vec3');

require('./common/math/b2Mat22');

require('./common/math/b2Mat33');

require('./common/math/b2Sweep');

require('./common/math/b2Transform');

require('./common/math/b2Math');

require('./common/b2Color');

require('./common/b2Settings');

Box2D.Collision = (function() {
  function Collision() {}

  return Collision;

})();

Box2D.Collision.Shapes = (function() {
  function Shapes() {}

  return Shapes;

})();

require('./collision/shapes/b2Shape');

require('./collision/shapes/b2CircleShape');

require('./collision/shapes/b2PolygonShape');

require('./collision/shapes/b2EdgeChainDef');

require('./collision/shapes/b2EdgeShape');

require('./collision/shapes/b2MassData');

require('./collision/Features');

require('./collision/b2ContactID');

require('./collision/ClipVertex');

require('./collision/b2AABB');

require('./collision/b2SimplexCache');

require('./collision/b2Bound');

require('./collision/b2BoundValues');

require('./collision/b2ManifoldPoint');

require('./collision/b2Manifold');

require('./collision/b2Collision');

require('./collision/b2ContactPoint');

require('./collision/b2SimplexVertex');

require('./collision/b2Simplex');

require('./collision/b2Distance');

require('./collision/b2DistanceInput');

require('./collision/b2DistanceOutput');

require('./collision/b2DistanceProxy');

require('./collision/b2DynamicTreeNode');

require('./collision/b2DynamicTree');

require('./collision/b2DynamicTreePair');

require('./collision/b2DynamicTreeBroadPhase');

require('./collision/b2Point');

require('./collision/b2RayCastInput');

require('./collision/b2RayCastOutput');

require('./collision/b2Segment');

require('./collision/b2SeparationFunction');

require('./collision/b2TimeOfImpact');

require('./collision/b2TOIInput');

require('./collision/b2WorldManifold');

Box2D.Dynamics = (function() {
  function Dynamics() {}

  return Dynamics;

})();

require('./dynamics/b2FilterData');

require('./dynamics/b2TimeStep');

Box2D.Dynamics.Contacts = (function() {
  function Contacts() {}

  return Contacts;

})();

require('./dynamics/contacts/b2ContactRegister');

require('./dynamics/contacts/b2ContactResult');

require('./dynamics/contacts/b2ContactEdge');

require('./dynamics/contacts/b2ContactConstraintPoint');

require('./dynamics/contacts/b2ContactConstraint');

require('./dynamics/contacts/b2Contact');

require('./dynamics/contacts/b2CircleContact');

require('./dynamics/contacts/b2PositionSolverManifold');

require('./dynamics/contacts/b2ContactSolver');

require('./dynamics/contacts/b2EdgeAndCircleContact');

require('./dynamics/contacts/b2NullContact');

require('./dynamics/contacts/b2PolyAndCircleContact');

require('./dynamics/contacts/b2PolyAndEdgeContact');

require('./dynamics/contacts/b2PolygonContact');

require('./dynamics/contacts/b2ContactFactory');

Box2D.Dynamics.Controllers = (function() {
  function Controllers() {}

  return Controllers;

})();

require('./dynamics/controllers/b2ControllerEdge');

require('./dynamics/controllers/b2Controller');

require('./dynamics/controllers/b2BuoyancyController');

require('./dynamics/controllers/b2ConstantAccelController');

require('./dynamics/controllers/b2ConstantForceController');

require('./dynamics/controllers/b2GravityController');

require('./dynamics/controllers/b2TensorDampingController');

Box2D.Dynamics.Joints = (function() {
  function Joints() {}

  return Joints;

})();

require('./dynamics/joints/b2Joint');

require('./dynamics/joints/b2JointDef');

require('./dynamics/joints/b2JointEdge');

require('./dynamics/joints/b2Jacobian');

require('./dynamics/joints/b2DistanceJoint');

require('./dynamics/joints/b2DistanceJointDef');

require('./dynamics/joints/b2FrictionJoint');

require('./dynamics/joints/b2FrictionJointDef');

require('./dynamics/joints/b2GearJoint');

require('./dynamics/joints/b2GearJointDef');

require('./dynamics/joints/b2LineJoint');

require('./dynamics/joints/b2LineJointDef');

require('./dynamics/joints/b2MouseJoint');

require('./dynamics/joints/b2MouseJointDef');

require('./dynamics/joints/b2PrismaticJoint');

require('./dynamics/joints/b2PrismaticJointDef');

require('./dynamics/joints/b2PulleyJoint');

require('./dynamics/joints/b2PulleyJointDef');

require('./dynamics/joints/b2RevoluteJoint');

require('./dynamics/joints/b2RevoluteJointDef');

require('./dynamics/joints/b2WeldJoint');

require('./dynamics/joints/b2WeldJointDef');

require('./dynamics/b2ContactListener');

require('./dynamics/b2ContactFilter');

require('./dynamics/b2Fixture');

require('./dynamics/b2ContactManager');

require('./dynamics/b2Body');

require('./dynamics/b2DebugDraw');

require('./dynamics/b2BodyDef');

require('./dynamics/b2ContactFilter');

require('./dynamics/b2ContactImpulse');

require('./dynamics/b2FixtureDef');

require('./dynamics/b2Island');

require('./dynamics/b2World');

require('./dynamics/b2DestructionListener');

//# sourceMappingURL=index.js.map

},{"./collision/ClipVertex":1,"./collision/Features":2,"./collision/b2AABB":3,"./collision/b2Bound":4,"./collision/b2BoundValues":5,"./collision/b2Collision":6,"./collision/b2ContactID":7,"./collision/b2ContactPoint":8,"./collision/b2Distance":9,"./collision/b2DistanceInput":10,"./collision/b2DistanceOutput":11,"./collision/b2DistanceProxy":12,"./collision/b2DynamicTree":13,"./collision/b2DynamicTreeBroadPhase":14,"./collision/b2DynamicTreeNode":15,"./collision/b2DynamicTreePair":16,"./collision/b2Manifold":17,"./collision/b2ManifoldPoint":18,"./collision/b2Point":19,"./collision/b2RayCastInput":20,"./collision/b2RayCastOutput":21,"./collision/b2Segment":22,"./collision/b2SeparationFunction":23,"./collision/b2Simplex":24,"./collision/b2SimplexCache":25,"./collision/b2SimplexVertex":26,"./collision/b2TOIInput":27,"./collision/b2TimeOfImpact":28,"./collision/b2WorldManifold":29,"./collision/shapes/b2CircleShape":30,"./collision/shapes/b2EdgeChainDef":31,"./collision/shapes/b2EdgeShape":32,"./collision/shapes/b2MassData":33,"./collision/shapes/b2PolygonShape":34,"./collision/shapes/b2Shape":35,"./common/b2Color":36,"./common/b2Settings":37,"./common/math/b2Mat22":38,"./common/math/b2Mat33":39,"./common/math/b2Math":40,"./common/math/b2Sweep":41,"./common/math/b2Transform":42,"./common/math/b2Vec2":43,"./common/math/b2Vec3":44,"./dynamics/b2Body":45,"./dynamics/b2BodyDef":46,"./dynamics/b2ContactFilter":47,"./dynamics/b2ContactImpulse":48,"./dynamics/b2ContactListener":49,"./dynamics/b2ContactManager":50,"./dynamics/b2DebugDraw":51,"./dynamics/b2DestructionListener":52,"./dynamics/b2FilterData":53,"./dynamics/b2Fixture":54,"./dynamics/b2FixtureDef":55,"./dynamics/b2Island":56,"./dynamics/b2TimeStep":57,"./dynamics/b2World":58,"./dynamics/contacts/b2CircleContact":59,"./dynamics/contacts/b2Contact":60,"./dynamics/contacts/b2ContactConstraint":61,"./dynamics/contacts/b2ContactConstraintPoint":62,"./dynamics/contacts/b2ContactEdge":63,"./dynamics/contacts/b2ContactFactory":64,"./dynamics/contacts/b2ContactRegister":65,"./dynamics/contacts/b2ContactResult":66,"./dynamics/contacts/b2ContactSolver":67,"./dynamics/contacts/b2EdgeAndCircleContact":68,"./dynamics/contacts/b2NullContact":69,"./dynamics/contacts/b2PolyAndCircleContact":70,"./dynamics/contacts/b2PolyAndEdgeContact":71,"./dynamics/contacts/b2PolygonContact":72,"./dynamics/contacts/b2PositionSolverManifold":73,"./dynamics/controllers/b2BuoyancyController":74,"./dynamics/controllers/b2ConstantAccelController":75,"./dynamics/controllers/b2ConstantForceController":76,"./dynamics/controllers/b2Controller":77,"./dynamics/controllers/b2ControllerEdge":78,"./dynamics/controllers/b2GravityController":79,"./dynamics/controllers/b2TensorDampingController":80,"./dynamics/joints/b2DistanceJoint":81,"./dynamics/joints/b2DistanceJointDef":82,"./dynamics/joints/b2FrictionJoint":83,"./dynamics/joints/b2FrictionJointDef":84,"./dynamics/joints/b2GearJoint":85,"./dynamics/joints/b2GearJointDef":86,"./dynamics/joints/b2Jacobian":87,"./dynamics/joints/b2Joint":88,"./dynamics/joints/b2JointDef":89,"./dynamics/joints/b2JointEdge":90,"./dynamics/joints/b2LineJoint":91,"./dynamics/joints/b2LineJointDef":92,"./dynamics/joints/b2MouseJoint":93,"./dynamics/joints/b2MouseJointDef":94,"./dynamics/joints/b2PrismaticJoint":95,"./dynamics/joints/b2PrismaticJointDef":96,"./dynamics/joints/b2PulleyJoint":97,"./dynamics/joints/b2PulleyJointDef":98,"./dynamics/joints/b2RevoluteJoint":99,"./dynamics/joints/b2RevoluteJointDef":100,"./dynamics/joints/b2WeldJoint":101,"./dynamics/joints/b2WeldJointDef":102}]},{},[103])(103)
});