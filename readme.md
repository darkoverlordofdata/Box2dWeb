# Box2DWeb

## Box2DWeb Recompiled

Box2D has a ton of great code wrapped up in some dated pre-html5 bindings.
I've automated a recompile of the code to modern coffeescript inspired bindings.

This cuts the function call overhead in half when allocating new framework objects.

Classes are written out in 1 class per file format in the ./lib folder to enable maintenance going forward.