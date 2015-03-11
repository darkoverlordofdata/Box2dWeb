'use strict'

b2internal =  'Box2D.Common.b2internal'
IBroadPhase = 'Box2D.Collision.IBroadPhase'

parseUInt = (v) -> Math.abs(parseInt(v))

Vector = (length=0) ->
  a = new Array(length)
  i = 0

  while i < length
    a[i] = 0
    ++i
  return a

class window.Box2D


  @equals: (o1, o2) ->
    return false  if o1 is null
    return true  if (o2 instanceof Function) and (o1 instanceof o2)
    return true  if (o1.constructor.__implements?[o2.name])
    false


  @generateCallback: (context, cb) ->
    return ->
      cb.apply context, arguments
      return

  @Common:
    b2internal: b2internal
    Math: {}

  @Collision:
    IBroadPhase: IBroadPhase
    Shapes: {}

  @Dynamics:
    Contacts: {}
    Controllers: {}
    Joints: {}

