Box2D = require('../../index')

class Box2D.Dynamics.b2Contact

  m_fixtureA: null
  m_fixtureB: null
  m_touching: false


  constructor: (fixtureA, fixtureB, touching) ->
    @m_fixtureA = fixtureA
    @m_fixtureB = fixtureB
    @m_touching = touching
    return

  GetFixtureA: ->
    return @m_fixtureA

  GetFixtureB: ->
    return @m_fixtureB

  IsTouching: ->
    return @m_touching

