class Box2D.Dynamics.Contacts.b2ContactConstraint

  localPlaneNormal  : null
  localPoint        : null
  normal            : null
  normalMass        : null
  K                 : null
  points            : null


  constructor: ->
    @localPlaneNormal = new b2Vec2()
    @localPoint = new b2Vec2()
    @normal = new b2Vec2()
    @normalMass = new b2Mat22()
    @K = new b2Mat22()
    @points = new Array(b2Settings.b2_maxManifoldPoints)
    i = 0

    while i < b2Settings.b2_maxManifoldPoints
      @points[i] = new b2ContactConstraintPoint()
      i++
    return