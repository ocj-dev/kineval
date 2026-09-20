/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Cloth Simulation in HTML5 Canvas | Javascript Infrastructure Methods

    Javascript support functions for the Canvas cloth-simulation environment.
      Includes initialization of the canvas/cloth/mouse handlers from the
      URL parameters parsed in cloth_canvas.html, and the mouse event wiring
      that feeds cloth.js's applyMouseInteraction().

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see cloth_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

//////////////////////////////////////////////////
/////     INITIALIZATION FUNCTIONS
//////////////////////////////////////////////////

function init() {

    canvas = document.getElementById('myCanvas');
    ctx = canvas.getContext('2d');
    canvas_width = canvas.width;
    canvas_height = canvas.height;

    var b_rect = canvas.getBoundingClientRect();
    canvas_side_off = b_rect.left;
    canvas_top_off = b_rect.top;

    // no ground plane by default; scenes (e.g. the blob-simulation slide
    // demo) can push an additional horizontal boundary onto this array
    if (typeof ground_planes === 'undefined') ground_planes = [];

    initCloth();
    initMouseHandlers();

    cur_time = Date.now();
    animate();
}

function initCloth() {
    cloth = new Cloth();
}

// (re)build the cloth from scratch, e.g. after a mouse-up sets a new goal-free
// restart, or when a slide-embedded demo changes a parameter live
function restartCloth() {
    initCloth();
}

//////////////////////////////////////////////////
/////     MOUSE HANDLING
//////////////////////////////////////////////////

// #region mouse-handlers
function initMouseHandlers() {

    canvas.onmousemove = function (e) {
        var rect = canvas.getBoundingClientRect();
        mouse.px = mouse.x;
        mouse.py = mouse.y;
        mouse.x = e.clientX - rect.left;
        mouse.y = e.clientY - rect.top;
    };

    canvas.onmousedown = function (e) {
        mouse.button = e.which;
        mouse.down = true;
    };

    canvas.onmouseup = function () {
        mouse.down = false;
    };

    // right-click is used for cutting, so suppress the context menu over the canvas
    canvas.oncontextmenu = function (e) { e.preventDefault(); };
}
// #endregion mouse-handlers
