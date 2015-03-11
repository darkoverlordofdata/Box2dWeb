# Box2DWeb

## Box2DWeb Recompiled

Box2D has a ton of great code wrapped up in some dated pre-html5 bindings.
I've automated a recompile of the code to modern coffeescript inspired bindings.

This cuts the function call overhead in half when allocating new framework objects.

Classes are written out in 1 class per file format in the ./lib folder to enable maintenance going forward.


        *
        * Using modified Box2dWeb-2.1.a.3.js:
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
