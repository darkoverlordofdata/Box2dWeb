#!/bin/sh

uglifyjs --mangle --output box2d_web.min.js box2d_web.js
uglifyjs --mangle --output chrysalis_box2d.min.js chrysalis_box2d.js
