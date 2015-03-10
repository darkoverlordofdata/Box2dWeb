fs = require('fs')
path = require('path')
{exec} = require('child_process')

###
 * Build the coffeescript classes
###
task 'build', 'build box2d', ->

  src = [String(fs.readFileSync('./lib/Box2D.coffee'))]
  filelist = String(fs.readFileSync('./filelist'))

  for f in filelist.split('\n')
    src.push(path.basename(f) + " = " + String(fs.readFileSync(f+'.coffee')))

  fs.writeFileSync("./tmp/Box2D.coffee", src.join('\n'))
  exec "coffee --output ./web -c ./tmp/Box2D.coffee"


###
 * parse the javascript library
###
namespace = ['Box2D']
classes = []

task 'convert', 'convert js', ->

  ###
   * Load box2d
  ###
  s = String(fs.readFileSync('./web/box2d_web.js'))
  eval(s)

  namespace = ['Box2D']
  classes = []
  process Box2D
  for klass in classes
    pathname = './'+((klass.namespace+'.'+klass.name).replace(/\./g,'/'))+'.js'
    src = klass.src[klass.name]

    node = Box2D
    list = klass.namespace.split('.')
    next = list.shift()
    while (next = list.shift())
      node = node[next]

    if 'function' is typeof src
      console.log "class " + klass.name

      # constructor
      #
      # function b2ClassName.b2ClassName() {
      #
      ctor = klass.src[klass.name]
      ctor = ctor.toString().replace('function () {','').replace(/}$/, '').split(/\n/)
      ctor.pop()
      ctor.shift()
      #
      # remove
      # Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
      #


      # initialization
      #
      # function b2ClassName.prototype.b2ClassName($0, $1...) {
      #
      args = ''
      init = []
      if klass.src.prototype[klass.name]
        t = klass.src.prototype[klass.name].toString()
        i = t.indexOf('(')
        j = t.indexOf(')')
        args = t.substring(i+1,j)
        t = t.replace(args, '')
        t = t.replace('function () {','').replace(/}$/, '')

        init = t.split(/\n/)
        init.pop()
        init.shift()
        #
        # replace
        # this.__super.b2Shape.call(this)
        #
      ctor = ctor.concat(init)
      code = []
      if ctor.length > 0
        code.push 'function '+klass.name+'('+args+'){'
        code.push ctor.join('\n')
        code.push '}'

      for method, v of klass.src.prototype
        if method isnt '__super' and method isnt 'constructor'
          code.push '//' + method
          vs = v.toString()
          vs = klass.name + '.prototype.' + method + ' = ' + vs + ';'
          code.push vs


      fs.writeFileSync(pathname, code.join('\n'))

###
 * Traverse pojs object tree
###
process = (obj) ->

  return unless obj?
  for name, member of obj
    if 'function' is typeof member # this is a class
      classes.push({namespace: namespace.join('.'), name: name, src: member})
    else
      namespace.push(name)
      for key, val of member
        if 'object' is typeof val
          namespace.push(key)
          process(val)
        else
          classes.push({namespace: namespace.join('.'), name: key, src: val})
      namespace.pop()
  namespace.pop()


