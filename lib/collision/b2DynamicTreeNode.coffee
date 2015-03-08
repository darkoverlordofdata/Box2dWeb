Box2D = require('../index')


class Box2D.Collision.b2DynamicTreeNode

  aabb      : null
  child1    : null

  constructor: ->
    @aabb = new b2AABB()
    return

  IsLeaf: ->
    return not @child1?
