(->
  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
  b2DebugDraw.b2DebugDraw = ->
    @m_drawScale = 1.0
    @m_lineThickness = 1.0
    @m_alpha = 1.0
    @m_fillAlpha = 1.0
    @m_xformScale = 1.0
    __this = this
    
    ##WORKAROUND
    @m_sprite = graphics:
      clear: ->
        __this.m_ctx.clearRect 0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height
        return

    return

  b2DebugDraw::_color = (color, alpha) ->
    "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")"

  b2DebugDraw::b2DebugDraw = ->
    @m_drawFlags = 0
    return

  b2DebugDraw::SetFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags = flags
    return

  b2DebugDraw::GetFlags = ->
    @m_drawFlags

  b2DebugDraw::AppendFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags |= flags
    return

  b2DebugDraw::ClearFlags = (flags) ->
    flags = 0  if flags is `undefined`
    @m_drawFlags &= ~flags
    return

  b2DebugDraw::SetSprite = (sprite) ->
    @m_ctx = sprite
    return

  b2DebugDraw::GetSprite = ->
    @m_ctx

  b2DebugDraw::SetDrawScale = (drawScale) ->
    drawScale = 0  if drawScale is `undefined`
    @m_drawScale = drawScale
    return

  b2DebugDraw::GetDrawScale = ->
    @m_drawScale

  b2DebugDraw::SetLineThickness = (lineThickness) ->
    lineThickness = 0  if lineThickness is `undefined`
    @m_lineThickness = lineThickness
    @m_ctx.strokeWidth = lineThickness
    return

  b2DebugDraw::GetLineThickness = ->
    @m_lineThickness

  b2DebugDraw::SetAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    @m_alpha = alpha
    return

  b2DebugDraw::GetAlpha = ->
    @m_alpha

  b2DebugDraw::SetFillAlpha = (alpha) ->
    alpha = 0  if alpha is `undefined`
    @m_fillAlpha = alpha
    return

  b2DebugDraw::GetFillAlpha = ->
    @m_fillAlpha

  b2DebugDraw::SetXFormScale = (xformScale) ->
    xformScale = 0  if xformScale is `undefined`
    @m_xformScale = xformScale
    return

  b2DebugDraw::GetXFormScale = ->
    @m_xformScale

  b2DebugDraw::DrawPolygon = (vertices, vertexCount, color) ->
    return  unless vertexCount
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.moveTo vertices[0].x * drawScale, vertices[0].y * drawScale
    i = 1

    while i < vertexCount
      s.lineTo vertices[i].x * drawScale, vertices[i].y * drawScale
      i++
    s.lineTo vertices[0].x * drawScale, vertices[0].y * drawScale
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawSolidPolygon = (vertices, vertexCount, color) ->
    return  unless vertexCount
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.fillStyle = @_color(color.color, @m_fillAlpha)
    s.moveTo vertices[0].x * drawScale, vertices[0].y * drawScale
    i = 1

    while i < vertexCount
      s.lineTo vertices[i].x * drawScale, vertices[i].y * drawScale
      i++
    s.lineTo vertices[0].x * drawScale, vertices[0].y * drawScale
    s.closePath()
    s.fill()
    s.stroke()
    return

  b2DebugDraw::DrawCircle = (center, radius, color) ->
    return  unless radius
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.arc center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawSolidCircle = (center, radius, axis, color) ->
    return  unless radius
    s = @m_ctx
    drawScale = @m_drawScale
    cx = center.x * drawScale
    cy = center.y * drawScale
    s.moveTo 0, 0
    s.beginPath()
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.fillStyle = @_color(color.color, @m_fillAlpha)
    s.arc cx, cy, radius * drawScale, 0, Math.PI * 2, true
    s.moveTo cx, cy
    s.lineTo (center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale
    s.closePath()
    s.fill()
    s.stroke()
    return

  b2DebugDraw::DrawSegment = (p1, p2, color) ->
    s = @m_ctx
    drawScale = @m_drawScale
    s.strokeStyle = @_color(color.color, @m_alpha)
    s.beginPath()
    s.moveTo p1.x * drawScale, p1.y * drawScale
    s.lineTo p2.x * drawScale, p2.y * drawScale
    s.closePath()
    s.stroke()
    return

  b2DebugDraw::DrawTransform = (xf) ->
    s = @m_ctx
    drawScale = @m_drawScale
    s.beginPath()
    s.strokeStyle = @_color(0xff0000, @m_alpha)
    s.moveTo xf.position.x * drawScale, xf.position.y * drawScale
    s.lineTo (xf.position.x + @m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + @m_xformScale * xf.R.col1.y) * drawScale
    s.strokeStyle = @_color(0xff00, @m_alpha)
    s.moveTo xf.position.x * drawScale, xf.position.y * drawScale
    s.lineTo (xf.position.x + @m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + @m_xformScale * xf.R.col2.y) * drawScale
    s.closePath()
    s.stroke()
    return

  return
)()
