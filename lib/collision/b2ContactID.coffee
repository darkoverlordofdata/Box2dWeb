Box2D = require('../index')


class Box2D.Collision.b2ContactID

  key:        null
  features:   null

  constructor: ->
    @features = new Features()
    @features._m_id = this
    return

  Set: (id) ->
    @key = id._key
    return

  Copy: ->
    id = new b2ContactID()
    id.key = @key
    id

  Object.defineProperties b2ContactID::,
    key:
      enumerable: false
      configurable: true
      get: -> @_key
      set: (value) ->
        value = 0  if value is undefined
        @_key = value
        @features._referenceEdge = @_key & 0x000000ff
        @features._incidentEdge = ((@_key & 0x0000ff00) >> 8) & 0x000000ff
        @features._incidentVertex = ((@_key & 0x00ff0000) >> 16) & 0x000000ff
        @features._flip = ((@_key & 0xff000000) >> 24) & 0x000000ff
        return
 