((a2j, undefined_) ->
  
  ###*
  if(!(Object.prototype.defineProperty instanceof Function)
  
  Corrected by deleteing
  10 chars.
  how many to document it?
  
  - darkoverlordofdata
  ###
  emptyFn = ->
  if (Object.defineProperty not instanceof Function) and Object::__defineGetter__ instanceof Function and Object::__defineSetter__ instanceof Function
    console.log "HOW DID I GET HERE?"
    Object.defineProperty = (obj, p, cfg) ->
      obj.__defineGetter__ p, cfg.get  if cfg.get instanceof Function
      obj.__defineSetter__ p, cfg.set  if cfg.set instanceof Function
      return
  a2j.inherit = (cls, base) ->
    tmpCtr = cls
    emptyFn:: = base::
    cls:: = new emptyFn
    cls::constructor = tmpCtr
    return

  a2j.generateCallback = generateCallback = (context, cb) ->
    ->
      cb.apply context, arguments
      return

  a2j.NVector = NVector = (length) ->
    length = 0  if length is `undefined`
    tmp = new Array(length or 0)
    i = 0

    while i < length
      tmp[i] = 0
      ++i
    tmp

  a2j.is = is = (o1, o2) ->
    return false  if o1 is null
    return true  if (o2 instanceof Function) and (o1 instanceof o2)
    return true  if (o1.constructor.__implements?) and (o1.constructor.__implements[o2])
    false

  a2j.parseUInt = (v) ->
    Math.abs parseInt(v)

  return
) Box2D
