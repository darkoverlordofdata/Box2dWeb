Box2D = require('../index')


class Box2D.Collision.b2DynamicTree

  constructor: ->
    @m_root = null
    @m_freeList = null
    @m_path = 0
    @m_insertionCount = 0
    return

  CreateProxy: (aabb, userData) ->
    node = @AllocateNode()
    extendX = b2Settings.b2_aabbExtension
    extendY = b2Settings.b2_aabbExtension
    node.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    node.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    node.aabb.upperBound.x = aabb.upperBound.x + extendX
    node.aabb.upperBound.y = aabb.upperBound.y + extendY
    node.userData = userData
    @InsertLeaf node
    node

  DestroyProxy: (proxy) ->
    @RemoveLeaf proxy
    @FreeNode proxy
    return

  MoveProxy: (proxy, aabb, displacement) ->
    b2Settings.b2Assert proxy.IsLeaf()
    return false  if proxy.aabb.Contains(aabb)
    @RemoveLeaf proxy
    extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.x > 0 then displacement.x else (-displacement.x)))
    extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * ((if displacement.y > 0 then displacement.y else (-displacement.y)))
    proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX
    proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY
    proxy.aabb.upperBound.x = aabb.upperBound.x + extendX
    proxy.aabb.upperBound.y = aabb.upperBound.y + extendY
    @InsertLeaf proxy
    true

  Rebalance: (iterations) ->
    iterations = 0  if iterations is `undefined`
    return  unless @m_root?
    i = 0

    while i < iterations
      node = @m_root
      bit = 0
      while node.IsLeaf() is false
        node = (if (@m_path >> bit) & 1 then node.child2 else node.child1)
        bit = (bit + 1) & 31
      ++@m_path
      @RemoveLeaf node
      @InsertLeaf node
      i++
    return

  GetFatAABB: (proxy) ->
    proxy.aabb

  GetUserData: (proxy) ->
    proxy.userData

  Query: (callback, aabb) ->
    return  unless @m_root?
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      if node.aabb.TestOverlap(aabb)
        if node.IsLeaf()
          proceed = callback(node)
          return  unless proceed
        else
          stack[count++] = node.child1
          stack[count++] = node.child2
    return

  RayCast: (callback, input) ->
    return  unless @m_root?
    p1 = input.p1
    p2 = input.p2
    r = b2Math.SubtractVV(p1, p2)
    r.Normalize()
    v = b2Math.CrossFV(1.0, r)
    abs_v = b2Math.AbsV(v)
    maxFraction = input.maxFraction
    segmentAABB = new b2AABB()
    tX = 0
    tY = 0
    tX = p1.x + maxFraction * (p2.x - p1.x)
    tY = p1.y + maxFraction * (p2.y - p1.y)
    segmentAABB.lowerBound.x = Math.min(p1.x, tX)
    segmentAABB.lowerBound.y = Math.min(p1.y, tY)
    segmentAABB.upperBound.x = Math.max(p1.x, tX)
    segmentAABB.upperBound.y = Math.max(p1.y, tY)
    stack = new Vector()
    count = 0
    stack[count++] = @m_root
    while count > 0
      node = stack[--count]
      continue  if node.aabb.TestOverlap(segmentAABB) is false
      c = node.aabb.GetCenter()
      h = node.aabb.GetExtents()
      separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y
      continue  if separation > 0.0
      if node.IsLeaf()
        subInput = new b2RayCastInput()
        subInput.p1 = input.p1
        subInput.p2 = input.p2
        subInput.maxFraction = input.maxFraction
        maxFraction = callback(subInput, node)
        return  if maxFraction is 0.0
        if maxFraction > 0.0
          tX = p1.x + maxFraction * (p2.x - p1.x)
          tY = p1.y + maxFraction * (p2.y - p1.y)
          segmentAABB.lowerBound.x = Math.min(p1.x, tX)
          segmentAABB.lowerBound.y = Math.min(p1.y, tY)
          segmentAABB.upperBound.x = Math.max(p1.x, tX)
          segmentAABB.upperBound.y = Math.max(p1.y, tY)
      else
        stack[count++] = node.child1
        stack[count++] = node.child2
    return

  AllocateNode: ->
    if @m_freeList
      node = @m_freeList
      @m_freeList = node.parent
      node.parent = null
      node.child1 = null
      node.child2 = null
      return node
    new b2DynamicTreeNode()

  FreeNode: (node) ->
    node.parent = @m_freeList
    @m_freeList = node
    return

  InsertLeaf: (leaf) ->
    ++@m_insertionCount
    unless @m_root?
      @m_root = leaf
      @m_root.parent = null
      return
    center = leaf.aabb.GetCenter()
    sibling = @m_root
    if sibling.IsLeaf() is false
      loop
        child1 = sibling.child1
        child2 = sibling.child2
        norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y)
        norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y)
        if norm1 < norm2
          sibling = child1
        else
          sibling = child2
        break unless sibling.IsLeaf() is false
    node1 = sibling.parent
    node2 = @AllocateNode()
    node2.parent = node1
    node2.userData = null
    node2.aabb.Combine leaf.aabb, sibling.aabb
    if node1
      if sibling.parent.child1 is sibling
        node1.child1 = node2
      else
        node1.child2 = node2
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      loop
        break  if node1.aabb.Contains(node2.aabb)
        node1.aabb.Combine node1.child1.aabb, node1.child2.aabb
        node2 = node1
        node1 = node1.parent
        break unless node1
    else
      node2.child1 = sibling
      node2.child2 = leaf
      sibling.parent = node2
      leaf.parent = node2
      @m_root = node2
    return

  RemoveLeaf: (leaf) ->
    if leaf is @m_root
      @m_root = null
      return
    node2 = leaf.parent
    node1 = node2.parent
    sibling = undefined
    if node2.child1 is leaf
      sibling = node2.child2
    else
      sibling = node2.child1
    if node1
      if node1.child1 is node2
        node1.child1 = sibling
      else
        node1.child2 = sibling
      sibling.parent = node1
      @FreeNode node2
      while node1
        oldAABB = node1.aabb
        node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb)
        break  if oldAABB.Contains(node1.aabb)
        node1 = node1.parent
    else
      @m_root = sibling
      sibling.parent = null
      @FreeNode node2
    return

 