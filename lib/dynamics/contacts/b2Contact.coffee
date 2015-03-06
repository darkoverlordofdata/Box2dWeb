Box2D = require('../../index')

class Box2D.Dynamics.b2Contact

  constructor: (fixtureA, fixtureB, touching) ->
    @m_fixtureA = fixtureA
    @m_fixtureB = fixtureB
    @m_touching = touching
    return

  GetFixtureA: ->
    @m_fixtureA

  GetFixtureB: ->
    @m_fixtureB

  IsTouching: ->
    @m_touching

