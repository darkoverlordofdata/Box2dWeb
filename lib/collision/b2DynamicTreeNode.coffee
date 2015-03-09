Box2D = require('../index')

b2AABB          = Box2D.Collision.b2AABB

class Box2D.Collision.b2DynamicTreeNode

  aabb      : null
  child1    : null

  constructor: ->
    @aabb = new b2AABB()
    return

  IsLeaf: ->
    return not @child1?
