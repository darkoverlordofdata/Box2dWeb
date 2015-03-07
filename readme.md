# Box2DWeb

## Work in progress


### The original Box2dWeb

    Issue 41: Box2D.js overwrite original Object.defineProperty
    https://code.google.com/p/box2dweb/issues/detail?id=41

    This issue has been open for over a year, and it's just a faulty polyfill.
    It causes other code, dependant on Object.defineProperty, to fail.
    I need a corrected version for my projects, so here it is.


### Cocoon

    ./cocoon - cocoon_box2d.js port
        ready to test
    ./bod2d_web - cocoon_box2d.js API, but for the browser
        not ready