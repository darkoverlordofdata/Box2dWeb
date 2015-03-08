Box2D = require('../index')


class Box2D.Collision.b2TimeOfImpact

  @TimeOfImpact: (input) ->
    ++b2TimeOfImpact.b2_toiCalls
    proxyA = input.proxyA
    proxyB = input.proxyB
    sweepA = input.sweepA
    sweepB = input.sweepB
    b2Settings.b2Assert sweepA.t0 is sweepB.t0
    b2Settings.b2Assert 1.0 - sweepA.t0 > Number.MIN_VALUE
    radius = proxyA.m_radius + proxyB.m_radius
    tolerance = input.tolerance
    alpha = 0.0
    k_maxIterations = 1000
    iter = 0
    target = 0.0
    b2TimeOfImpact.s_cache.count = 0
    b2TimeOfImpact.s_distanceInput.useRadii = false
    loop
      sweepA.GetTransform b2TimeOfImpact.s_xfA, alpha
      sweepB.GetTransform b2TimeOfImpact.s_xfB, alpha
      b2TimeOfImpact.s_distanceInput.proxyA = proxyA
      b2TimeOfImpact.s_distanceInput.proxyB = proxyB
      b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA
      b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB
      b2Distance.Distance b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput
      if b2TimeOfImpact.s_distanceOutput.distance <= 0.0
        alpha = 1.0
        break
      b2TimeOfImpact.s_fcn.Initialize b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB
      separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if separation <= 0.0
        alpha = 1.0
        break
      if iter is 0
        if separation > radius
          target = b2Math.Max(radius - tolerance, 0.75 * radius)
        else
          target = b2Math.Max(separation - tolerance, 0.02 * radius)
      if separation - target < 0.5 * tolerance
        if iter is 0
          alpha = 1.0
          break
        break
      newAlpha = alpha
      x1 = alpha
      x2 = 1.0
      f1 = separation
      sweepA.GetTransform b2TimeOfImpact.s_xfA, x2
      sweepB.GetTransform b2TimeOfImpact.s_xfB, x2
      f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
      if f2 >= target
        alpha = 1.0
        break
      rootIterCount = 0
      loop
        x = 0
        if rootIterCount & 1
          x = x1 + (target - f1) * (x2 - x1) / (f2 - f1)
        else
          x = 0.5 * (x1 + x2)
        sweepA.GetTransform b2TimeOfImpact.s_xfA, x
        sweepB.GetTransform b2TimeOfImpact.s_xfB, x
        f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB)
        if b2Math.Abs(f - target) < 0.025 * tolerance
          newAlpha = x
          break
        if f > target
          x1 = x
          f1 = f
        else
          x2 = x
          f2 = f
        ++rootIterCount
        ++b2TimeOfImpact.b2_toiRootIters
        break  if rootIterCount is 50
      b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount)
      break  if newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha
      alpha = newAlpha
      iter++
      ++b2TimeOfImpact.b2_toiIters
      break  if iter is k_maxIterations
    b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(b2TimeOfImpact.b2_toiMaxIters, iter)
    alpha

  @b2_toiCalls = 0
  @b2_toiIters = 0
  @b2_toiMaxIters = 0
  @b2_toiRootIters = 0
  @b2_toiMaxRootIters = 0
  @s_cache = new b2SimplexCache()
  @s_distanceInput = new b2DistanceInput()
  @s_xfA = new b2Transform()
  @s_xfB = new b2Transform()
  @s_fcn = new b2SeparationFunction()
  @s_distanceOutput = new b2DistanceOutput()
