fs = require('fs')
path = require('path')
{exec} = require('child_process')

###> ========================================================================
    Build the coffeescript classes
======================================================================== <###
#task 'buildcoffee', 'build box2d', ->
#
#  src = [String(fs.readFileSync('./coffee/Box2D.coffee'))]
#  filelist = String(fs.readFileSync('./coffee/filelist'))
#
#  for f in filelist.split('\n')
#    src.push(path.basename(f) + " = " + String(fs.readFileSync(f+'.coffee')))
#
#  fs.writeFileSync("./tmp/Box2D.coffee", src.join('\n'))
#  exec "coffee --output ./web -c ./tmp/Box2D.coffee"

###> ========================================================================
    Build the javascript classes
======================================================================== <###
task 'buildjs', 'build box2d', ->   _buildjs()

###> ========================================================================
    perf testing
======================================================================== <###
task 'perfc', 'performance test', ->
  `'use strict;'`


  obj = {x: 0};
  y = 42

  d1 = Date.now()
  for i in [0...100000000]
    `obj.x = y || 0;`


  d2 = Date.now()
  for i in [0...100000000]
    `if (y === undefined) obj.x=y;`


  d3 = Date.now()

  console.log "if 1 #{d2 - d1}"
  console.log "if 2 #{d3 - d2}"


task 'perfa', 'performance test', ->

  Box2D = require('box2dweb')
  Redux = require('./web/packages/box2d/Box2D.js')

  d1 = Date.now()
  _tester Redux
  d2 = Date.now()

  console.log "Redux #{d2 - d1}"

task 'perfb', 'performance test', ->

  Box2D = require('box2dweb')

  d1 = Date.now()
  _tester Box2D
  d2 = Date.now()

  console.log "Box2D #{d2 - d1}"


_tester = (Box2D) ->
  b2World               = Box2D.Dynamics.b2World
  b2Body                = Box2D.Dynamics.b2Body
  b2BodyDef             = Box2D.Dynamics.b2BodyDef
  b2CircleShape         = Box2D.Collision.Shapes.b2CircleShape
  b2FixtureDef          = Box2D.Dynamics.b2FixtureDef
  b2Vec2                = Box2D.Common.Math.b2Vec2

  world = new b2World(new b2Vec2(0, 0), true)
  bullets = []
  for i in [0...10000]
    ###
     * Bullet simulation
    ###
    bodyDef = new b2BodyDef()
    bodyDef.type = b2Body.b2_dynamicBody
    bodyDef.fixedRotation = true
    bodyDef.position.x = 100
    bodyDef.position.y = 100
    bodyDef.linearVelocity.Set(.01, .01)
    bodyDef.angularVelocity = 0

    fixDef = new b2FixtureDef()
    fixDef.density = 1.0
    fixDef.friction = 0.0
    fixDef.restitution = 0.2
    fixDef.shape = new b2CircleShape(0)

    body = world.CreateBody(bodyDef)
    body.CreateFixture(fixDef)
    bullets.push body





###> ========================================================================
    parse the javascript library
======================================================================== <###

namespace = ['Box2D']
classes = []
option '-o', '--override', 'use overrides'
task 'redux', 'Redux the box2sweb library', (options) ->

  namespace = ['Box2D']   # namespace root
  classes = []            # list of classes
  tab = '   '             # 3 spaces per tab
  ###
   *
   * Load modified Box2dWeb-2.1.a.3.js:
   *
   *  Crashes in node.js:
   *
   *     22   if(!(Object.prototype.defineProperty instanceof Function)
   *
   *    should be
   *
   *     22   if(!(Object.defineProperty instanceof Function)
   *
   *  Comment out so we can process the postDefs:
   *
   *  10866   //delete Box2D.postDefs;
   *
   *
  ###
  eval(String(fs.readFileSync('./src/box2d_web.js')))
  loadClasses Box2D

  ###
   * Hold your bits - mikolalysenko has already done this!
  ###
#  Box2D = require('box2dweb')

  postDefs = []
  statics = {}
  for postDef in Box2D.postDefs
    def = postDef.toString().split('\n')
    def.pop()
    def.shift()
    for line, ix in def
      def[ix] = line.replace('      Box2D.', 'Box2D.')


    for line in def
      val = line.split(/\s*=\s*/)
      ns = val[0].split('.')
      key = ns[ns.length-2]
      statics[key] ?= []
      statics[key].push val



  ###
   * re-write each class
  ###
  for klass in classes

    continue if klass.namespace is 'Box2D.postDefs'

    pathname = './'+(klass.namespace.replace(/\./g,'/').replace('Box2D','lib'))+'/'
    src = klass.src[klass.name]
    code = [] # output
    foundSuper = false


    if 'function' is typeof src
      ###
       *  constructor:
       *
       *  Combine constructor and initializers
       *
       *  function b2ClassName.b2ClassName() {
      ###
      ctor = klass.src[klass.name]
      ctor = ctor.toString().split(/\n/)
      ctor.pop()    # remove start block
      ctor.shift()  # remove end block
      ###
      # remove this super
      ###
      if klass.src.prototype.__super?
        uber = klass.src.prototype.__super.constructor
        m1 = klass.namespace + '.' + uber.name + '.' + uber.name + '.apply(this, arguments);'
        for line, ix in ctor
          if line.indexOf(m1) >= 0
            ctor[ix] = ''


      ###
       * initialization
       *
       * transform super call
      ###
      args = ''
      init = []
      if klass.src.prototype[klass.name]
        func = klass.src.prototype[klass.name].toString()
        args = func.substring(func.indexOf('(')+1, func.indexOf(')'))
        func = func.replace(args, '')
        func = func.replace('function () {','').replace(/}$/, '')

        init = func.split('\n')
        init.pop()    # remove start block
        init.shift()  # remove end block
        ###
        # replace super
        ###
        if klass.src.prototype.__super?
          uber = klass.src.prototype.__super.constructor
          m2 = 'this.__super.' + uber.name + '.call(this'
          for line, ix in init
            if line.indexOf(m2) >= 0
              init[ix] = line.replace(m2, klass.name+'.__super__.constructor.call(this')
              foundSuper = true

      if klass.src.prototype.__super? and not foundSuper
        init.unshift tab+tab+klass.name+'.__super__.constructor.apply(this, arguments);'


      ###
       *
       *  Static Fields
       *
      ###
      if statics[klass.name]?
        code.push ''
        for val in statics[klass.name]
          ns = val[0].split('.')
          if val[1].indexOf(klass.name) >= 0
            postDefs.push val
          else
            code.push tab+klass.name+'.'+ns[ns.length-1]+' = '+val[1]


      ###
       *
       *  Instance Fields
       *
      ###
      if (fs.existsSync(pathname+klass.name+'.properties.js'))
        props = String(fs.readFileSync(pathname+klass.name+'.properties.js'))
        code.push ''
        for prop in props.split('\n')
          code.push tab+klass.name+'.prototype.'+prop


      ###
       * The constructor
       *
      ###
      if (options.override and fs.existsSync(pathname+klass.name+'_ctor.js'))
        code.push String(fs.readFileSync(pathname+klass.name+'_ctor.js'))
      else
        code.push ''
        code.push '   /**'
        code.push '    * Constructor'
        code.push '    *'
        for param in args.split(/\s*,\s*/)
          code.push '    * @param ' + param
        code.push '    *'
        code.push '    */'
        code.push tab + 'function '+klass.name+'('+args+'){'
        code.push _opt(ctor.join('\n'))
        code.push _opt(init.join('\n'))
        code.push tab + '}'

      ###
       * Static Members
      ###
      for methodName, method of klass.src
        if methodName isnt klass.name
          if 'function' is typeof method
            func = method.toString()
            args = func.substring(func.indexOf('(')+1, func.indexOf(')'))
            func = func.split('\n')

            code.push ''
            code.push '   /**'
            code.push '    * Static '+methodName
            code.push '    *'
            for param in args.split(/\s*,\s*/)
              code.push '    * @param ' + param
            code.push '    *'
            code.push '    */'
            code.push tab + klass.name + '.' + methodName + ' = ' + _opt(func.join('\n')) + ';'


      ###
       * The remaining methods
       * Transform super calls
      ###
      for methodName, method of klass.src.prototype
        if methodName isnt '__super' and methodName isnt 'constructor' and methodName isnt klass.name
          func = method.toString()
          args = func.substring(func.indexOf('(')+1, func.indexOf(')'))

          func = method.toString().split('\n')
          if klass.src.prototype.__super?
            m3 = 'this.__super.' + methodName + '.call(this'

            for line, ix in func
              if line.indexOf(m3) >= 0
                func[ix] = line.replace(m3, klass.name+'.__super__.'+methodName+'.call(this')

          code.push ''
          code.push '   /**'
          code.push '    * '+methodName
          code.push '    *'
          for param in args.split(/\s*,\s*/)
            code.push '    * @param ' + param
          code.push '    *'
          code.push '    */'
          code.push tab + klass.name + '.prototype.' + methodName + ' = ' + _opt(func.join('\n')) + ';'


      prog = []
      prog.push '/**'
      prog.push ' * Class '+klass.name
      prog.push ' *'
      prog.push ' *'
      prog.push ' */'
      if klass.src.prototype.__super?
        uber = klass.src.prototype.__super.constructor
        prog.push klass.name + ' = ' + klass.namespace + '.' + klass.name + ' = (function(superClass) {'
        prog.push '\'use strict;\''
        prog.push tab+'extend(' + klass.name + ', superClass);'
        prog.push _opt(code.join('\n'))
        while (val = postDefs.pop())
          ns = val[0].split('.')
          prog.push tab+klass.name+'.'+ns[ns.length-1]+' = '+val[1]
        prog.push tab+'return ' + klass.name + ';'
        prog.push  '})('+uber.name+');'
      else
        prog.push  klass.namespace + '.' + klass.name + ' = ' + klass.name + ' = (function() {'
        prog.push '\'use strict;\''
        prog.push _opt(code.join('\n'))
        while (val = postDefs.pop())
          ns = val[0].split('.')
          prog.push tab+klass.name+'.'+ns[ns.length-1]+' = '+val[1]
        prog.push tab+'return ' + klass.name + ';'
        prog.push '})();'

      fs.writeFileSync(pathname+klass.name+'.js', prog.join('\n'))

  _buildjs()


###
 * Traverse pojs object tree
###
_buildjs = ->


  src = []
  filelist = String(fs.readFileSync('./lib/filelist'))

  for f in filelist.split('\n')
    src.push(String(fs.readFileSync(f+'.js')))

  template = String(fs.readFileSync('./lib/template.js'))
  .replace("'{% Box2D %}'", src.join('\n'))

  fs.writeFileSync("./web/packages/box2d/Box2D.js", template)


###
 * Traverse pojs object tree
###
loadClasses = (obj) ->

  return unless obj?
  for name, member of obj
    if 'function' is typeof member # this is a class
      classes.push({namespace: namespace.join('.'), name: name, src: member})
    else
      namespace.push(name)
      for key, val of member
        if 'object' is typeof val
          #
          # recursively process nodes
          #
          namespace.push(key)
          loadClasses(val)
        else
          classes.push({namespace: namespace.join('.'), name: key, src: val})
      namespace.pop()
  namespace.pop()
  return


###
 * Optimizations
###
_opt = (s) ->


  ###
   * Rule 1
   *
   *  > if (x === undefined) x = 0;
   *  < x = x || 0;
   *
  ###
  op1 = /if\s*\(\s*([A-Za-z0-9._]*)\s*===\s*undefined\s*\)\s*([A-Za-z0-9._]*)\s*=\s*([A-Za-z0-9._]*)\s*;/

  re1 = ($0, $1, $2, $3) ->
    if $1 is $2
      "#{$1} = #{$1} || #{$3};"
    else
      $0


  if 'string' is typeof s
    if s.indexOf('\n')
      s1 = s.split('\n')
      for s2, index in s1
        if op1.test(s2)
          s1[index] = s2.replace(op1, re1)
      s = s1.join('\n')

    else
      if op1.test(s)
        s = s.replace(op1, re1)
  else
    for s1, index in s
      if op1.test(s)
        s[index] = s1.replace(op1, re1)


  return s

###> ========================================================================
    test the optimization regexp
======================================================================== <###
task 'test', 'test', ->

  x = "if (x_ === undefined) x_ = 0;"
  console.log x
  console.log _opt(x)

  x = [
    "if (x_ === undefined) x_ = 0;"
    "if (y_ === undefined) y_ = 0;"
  ]
  console.log x
  console.log _opt(x)

  x = "if (x_ === undefined) x_ = 0;\nif (y_ === undefined) y_ = 0;"
  console.log x
  console.log _opt(x)

