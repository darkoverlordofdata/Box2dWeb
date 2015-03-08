Box2D = require('../index')

class Box2D.Dynamics.b2DebugDraw
  
  SetFlags: (flags) ->
    flags = 0  if flags is `undefined`
    return

  GetFlags: ->

  AppendFlags: (flags) ->
    flags = 0  if flags is `undefined`
    return

  ClearFlags: (flags) ->
    flags = 0  if flags is `undefined`
    return

  SetSprite: (sprite) ->

  GetSprite: ->

  SetDrawScale: (drawScale) ->
    drawScale = 0  if drawScale is `undefined`
    return

  GetDrawScale: ->

  SetLineThickness: (lineThickness) ->
    lineThickness = 0  if lineThickness is `undefined`
    return

  GetLineThickness: ->

  SetAlpha: (alpha) ->
    alpha = 0  if alpha is `undefined`
    return

  GetAlpha: ->

  SetFillAlpha: (alpha) ->
    alpha = 0  if alpha is `undefined`
    return

  GetFillAlpha: ->

  SetXFormScale: (xformScale) ->
    xformScale = 0  if xformScale is `undefined`
    return

  GetXFormScale: ->

  DrawPolygon: (vertices, vertexCount, color) ->
    vertexCount = 0  if vertexCount is `undefined`
    return

  DrawSolidPolygon: (vertices, vertexCount, color) ->
    vertexCount = 0  if vertexCount is `undefined`
    return

  DrawCircle: (center, radius, color) ->
    radius = 0  if radius is `undefined`
    return

  DrawSolidCircle: (center, radius, axis, color) ->
    radius = 0  if radius is `undefined`
    return

  DrawSegment: (p1, p2, color) ->

  DrawTransform: (xf) ->

  @e_shapeBit = 0x0001
  @e_jointBit = 0x0002
  @e_aabbBit = 0x0004
  @e_pairBit = 0x0008
  @e_centerOfMassBit = 0x0010
  @e_controllerBit = 0x0020

