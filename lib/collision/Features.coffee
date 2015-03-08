Box2D = require('../index')


class Box2D.Collision.Features

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    get: ->
      @_referenceEdge

  Object.defineProperty Features::, "referenceEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_referenceEdge = value
      @_m_id._key = (@_m_id._key & 0xffffff00) | (@_referenceEdge & 0x000000ff)
      return

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    get: ->
      @_incidentEdge

  Object.defineProperty Features::, "incidentEdge",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentEdge = value
      @_m_id._key = (@_m_id._key & 0xffff00ff) | ((@_incidentEdge << 8) & 0x0000ff00)
      return

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    get: ->
      @_incidentVertex

  Object.defineProperty Features::, "incidentVertex",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_incidentVertex = value
      @_m_id._key = (@_m_id._key & 0xff00ffff) | ((@_incidentVertex << 16) & 0x00ff0000)
      return

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    get: ->
      @_flip

  Object.defineProperty Features::, "flip",
    enumerable: false
    configurable: true
    set: (value) ->
      value = 0  if value is `undefined`
      @_flip = value
      @_m_id._key = (@_m_id._key & 0x00ffffff) | ((@_flip << 24) & 0xff000000)
      return
