class Box2D.Collision.Features

  Object.defineProperties Features::,

    referenceEdge:
      enumerable: false
      configurable: true
      get: -> @_referenceEdge
      set: (value) ->
        value = 0  if value is undefined
        @_referenceEdge = value
        @_m_id._key = (@_m_id._key & 0xffffff00) | (@_referenceEdge & 0x000000ff)
        return

    incidentEdge:
      enumerable: false
      configurable: true
      get: -> @_incidentEdge
      set: (value) ->
        value = 0  if value is undefined
        @_incidentEdge = value
        @_m_id._key = (@_m_id._key & 0xffff00ff) | ((@_incidentEdge << 8) & 0x0000ff00)
        return

    incidentVertex:

      enumerable: false
      configurable: true
      get: -> @_incidentVertex
      set: (value) ->
        value = 0  if value is undefined
        @_incidentVertex = value
        @_m_id._key = (@_m_id._key & 0xff00ffff) | ((@_incidentVertex << 16) & 0x00ff0000)
        return

    flip:
      enumerable: false
      configurable: true
      get: -> @_flip
      set: (value) ->
        value = 0  if value is undefined
        @_flip = value
        @_m_id._key = (@_m_id._key & 0x00ffffff) | ((@_flip << 24) & 0xff000000)
        return
