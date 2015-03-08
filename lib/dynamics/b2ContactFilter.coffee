Box2D = require('../index')

b2Vec2      = Box2D.Common.Math.b2Vec2
b2Body      = Box2D.Dynamics.b2Body

class Box2D.Dynamics.b2ContactFilter
  
  
  ShouldCollide: (fixtureA, fixtureB) ->
    filter1 = fixtureA.GetFilterData()
    filter2 = fixtureB.GetFilterData()
    return filter1.groupIndex > 0  if filter1.groupIndex is filter2.groupIndex and filter1.groupIndex isnt 0
    collide = (filter1.maskBits & filter2.categoryBits) isnt 0 and (filter1.categoryBits & filter2.maskBits) isnt 0
    collide

  RayCollide: (userData, fixture) ->
    return true  unless userData
    @ShouldCollide ((if userData instanceof b2Fixture then userData else null)), fixture

  @b2_defaultFilter = new b2ContactFilter()
