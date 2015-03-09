#!/usr/bin/env coffee

fs = require('fs')

s = String(fs.readFileSync('./web/box2d_web.js'))

eval(s)

simple = ['inherit', 'generateCallback', 'NVector', 'is', 'parseUInt']
namespace = ['Box2D']
source = []

process = (o) ->

  return unless o?
  for name, value of o
    if 'function' is typeof value
      source.push({namespace: namespace.join('.'), name: name, src:value})
    else
      namespace.push(name)
      for k, v of value
        if 'object' is typeof v
          namespace.push(k)
          process(v)
        else
          source.push({namespace: namespace.join('.'), name: k, src:v})
      namespace.pop()
  namespace.pop()


process Box2D
for o in source
#  console.log(o.namespace+'.'+o.name)
  src = o.src[o.name]
  x = Box2D
  y = o.namespace.split('.')
  z = y.shift()
  while (z = y.shift())
    x = x[z]
  if 'undefined' is typeof src # not a class
    if 'function' is typeof x[o.name]
      console.log 'Function '+x[o.name].name
    else
      console.log typeof(x[o.name]) + " " + x[o.name]



  else # this is a class
    console.log "class " + o.name
#    console.log x[o.name].toString()

    # constructor
    ctor = o.src[o.name]
    c = ctor.toString().replace('function () {','').replace(/}$/, '').split(/\n/)
    c.pop()
    c.shift()

    # initializer
    args = ''
    d = []
    if o.src.prototype[o.name]
      init = o.src.prototype[o.name]
      t = init.toString()
      i = t.indexOf('(')
      j = t.indexOf(')')
      args = t.substring(i+1,j)
      t = t.replace(args, '')
      t = t.replace('function () {','').replace(/}$/, '')

      d = t.split(/\n/)
      d.pop()
      d.shift()

    c = c.concat(d)
    if c.length > 0
      console.log '-------------------------'
      console.log 'function '+o.name+'('+args+'){'
      if o.src.prototype.__super
        console.log '\tthis.__super.apply(this, arguments);'
      console.log c.join('\n')
      console.log '}'
      console.log '-------------------------'


