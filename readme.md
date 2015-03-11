# Box2DWeb

## Box2DWeb ReGenerated

Box2DWeb 2.1alpha has a ton of great code wrapped up in some dated pre-html5 bindings.
The Object.defineProperty issue was fixed by mikolalysenko, so I'm using that version (https://www.npmjs.com/package/box2dweb) as my base.

Phase I: automated recompilation of the code to use modern coffeescript inspired bindings.

    Completed: This cuts the function call overhead in half when allocating new framework objects.

Phase II: define scalar properties on the prototype, and remove initialization from the constructor.

    In Progress: This should also reduce overhead.

Classes are written out in 1 class per file format in the ./lib folder to enable maintenance going forward.


Also included is a redux of CocconJS box2d bindings done in coffeescript (see coffee folder)
