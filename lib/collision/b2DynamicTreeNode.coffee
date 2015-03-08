Box2D = require('../index')


class Box2D.Collision.b2DynamicTreeNode

  constructor: ->
    @aabb = new b2AABB()
    return

  IsLeaf: ->
    not @child1?
