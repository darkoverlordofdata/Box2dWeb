# Box2DWeb

## Box2DWeb Redux

An redux of mikolalysenko's npm listing of Uli Hecht's port of Box2DFlash which is the flash port of Erin Catto's box2d library.

Try to say that in one breath!

Box2DWeb 2.1alpha has a ton of great code wrapped up in some dated pre-html5 bindings.
[Mikolalysenko] (https://www.npmjs.com/package/box2dweb) fixed the Object.defineProperty issue,
and exposed the postDefs object, so we can use this version to fully automate the process.

Phase I: automated recompilation of the code to use modern coffeescript inspired bindings.

    Completed: This cuts the function call overhead in half when allocating new framework objects.

Phase II: define scalar properties on the prototype, and remove initialization from the constructor.

    In Progress: This should also reduce overhead.

Classes are written out in 1 class per file format in the ./lib folder to enable maintenance going forward.


## Install

$ git clone https://github.com/darkoverlordofdata/Box2dWeb.git
$ cd Box2dWeb
$ npm install
$ cake redux