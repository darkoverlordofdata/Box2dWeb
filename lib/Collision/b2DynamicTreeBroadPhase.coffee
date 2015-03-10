class Box2D.Collision.b2DynamicTreeBroadPhase

  m_tree              : null
  m_moveBuffer        : null
  m_pairBuffer        : null
  m_pairCount         : 0
  @m_proxyCount       : 0

  constructor: ->
    @m_tree = new b2DynamicTree()
    @m_moveBuffer = new Array()
    @m_pairBuffer = new Array()
    return

  CreateProxy: (aabb, userData) ->
    proxy = @m_tree.CreateProxy(aabb, userData)
    ++@m_proxyCount
    @BufferMove proxy
    proxy
  
  DestroyProxy: (proxy) ->
    @UnBufferMove proxy
    --@m_proxyCount
    @m_tree.DestroyProxy proxy
    return
  
  MoveProxy: (proxy, aabb, displacement) ->
    buffer = @m_tree.MoveProxy(proxy, aabb, displacement)
    @BufferMove proxy  if buffer
    return
  
  TestOverlap: (proxyA, proxyB) ->
    aabbA = @m_tree.GetFatAABB(proxyA)
    aabbB = @m_tree.GetFatAABB(proxyB)
    aabbA.TestOverlap aabbB
  
  GetUserData: (proxy) ->
    @m_tree.GetUserData proxy
  
  GetFatAABB: (proxy) ->
    @m_tree.GetFatAABB proxy
  
  GetProxyCount: ->
    @m_proxyCount
  
  UpdatePairs: (callback) ->
    @m_pairCount = 0
    i = 0
    queryProxy = undefined
    i = 0
    while i < @m_moveBuffer.length
      QueryCallback = (proxy) =>
        return true  if proxy is queryProxy
        @m_pairBuffer[@m_pairCount] = new b2DynamicTreePair()  if @m_pairCount is @m_pairBuffer.length
        pair = @m_pairBuffer[@m_pairCount]
        pair.proxyA = (if proxy < queryProxy then proxy else queryProxy)
        pair.proxyB = (if proxy >= queryProxy then proxy else queryProxy)
        ++@m_pairCount
        true
      queryProxy = @m_moveBuffer[i]
      fatAABB = @m_tree.GetFatAABB(queryProxy)
      @m_tree.Query QueryCallback, fatAABB
      ++i
    @m_moveBuffer.length = 0
    i = 0
  
    while i < @m_pairCount
      primaryPair = @m_pairBuffer[i]
      userDataA = @m_tree.GetUserData(primaryPair.proxyA)
      userDataB = @m_tree.GetUserData(primaryPair.proxyB)
      callback userDataA, userDataB
      ++i
      while i < @m_pairCount
        pair = @m_pairBuffer[i]
        break  if pair.proxyA isnt primaryPair.proxyA or pair.proxyB isnt primaryPair.proxyB
        ++i
    return
  
  Query: (callback, aabb) ->
    @m_tree.Query callback, aabb
    return
  
  RayCast: (callback, input) ->
    @m_tree.RayCast callback, input
    return
  
  Validate: ->
  
  Rebalance: (iterations) ->
    iterations = 0  if iterations is undefined
    @m_tree.Rebalance iterations
    return
  
  BufferMove: (proxy) ->
    @m_moveBuffer[@m_moveBuffer.length] = proxy
    return
  
  UnBufferMove: (proxy) ->
    i = parseInt(@m_moveBuffer.indexOf(proxy))
    @m_moveBuffer.splice i, 1
    return
  
  ComparePairs: (pair1, pair2) ->
    return 0

  @__implements = IBroadPhase: true
