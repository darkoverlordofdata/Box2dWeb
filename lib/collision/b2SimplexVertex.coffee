Box2D = require('../index')


class Box2D.Collision.b2SimplexVertex

  Set: (other) ->
    @wA.SetV other.wA
    @wB.SetV other.wB
    @w.SetV other.w
    @a = other.a
    @indexA = other.indexA
    @indexB = other.indexB
    return
