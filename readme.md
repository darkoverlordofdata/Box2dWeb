# Box2DWeb

## Box2DWeb Redux

An redux of mikolalysenko's npm listing of Uli Hecht's port of Box2DFlash which is the flash port of Erin Catto's box2d library.

Try to say that in one breath!

Box2DWeb 2.1alpha has a ton of code wrapped up in some dated pre-html5 bindings.
[Mikolalysenko] (https://www.npmjs.com/package/box2dweb) fixed the Object.defineProperty issue,
and exposed the postDefs object, so we can use this version to fully automate the process.

Phase I: automated recompilation of the code to use modern coffeescript inspired bindings.

    Complete.
    The redux process iterates over the live Box2D object, extracting the source code for all methods.
    This code is used to generate new class definitions, 1 class per file. Multiple initializations are
    reduced to 1 constructor function. Then it's all packed back up with a spiffy new header.
    This cuts the function call overhead in half when allocating new framework objects.


Phase II: move scalar property definitions to the prototype and remove them from the constructor

    In Progress.
    Allows manually written overrides for constructors to be merged in during the redux process.
    This should also reduce overhead:
    1. Avoids unnecessary initialization code
    2. Results in fewer hidden classes


## Performace

Performance is a slippery slope. I've created a simple test, taken from Asteroids,
that creates 10,000 bullets. Typical results:
Box2D - (5269, 5113, 5167, 5488, 5336) = 26373 ms
Redux - (3167, 3745, 4314, 4171, 3894) = 19966 ms

For the most part, I see up to a 25% increase.


## Install

```bash
$ git clone https://github.com/darkoverlordofdata/Box2dWeb.git
$ cd Box2dWeb
$ npm install
$ cake redux
```
