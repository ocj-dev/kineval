/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Pendularm Dynamical Simulation | three.js Scene

    A faithful port of the upstream kineval-stencil's own createScene() (from
    project_pendularm/pendularm1.html and pendularm2.html): the same 4-leg
    table stand (legs, sidebars, crossbar), the same pendulum.geom /
    pendulum_link / pendulum_mass / pendulum2_link / pendulum2_mass mesh
    hierarchy and one-time rotateOnAxis() setup calls, and the same per-frame
    drive -- pendulum.geom.rotation.y = angle[0], and for the double pendulum,
    pendulum_mass.rotation.z = angle[1] (see the stencil's own header comment:
    "second arm of pendulum must be in world coordinates, not parent link
    coordinates"). Modernized to ES-module imports and idiomatic
    .position.set() calls (equivalent to the stencil's own `mesh.position =
    {x,y,z}` -- three.js's matrix composition only ever reads position.x/y/z,
    so both forms produce the same transform); a locally vendored current
    three.js release replaces the stencil's pinned non-module r92 build, and
    OrbitControls plus a responsive renderer/ground-plane are added for a
    proper orbit-able view -- but the pendulum and stand geometry itself, and
    its swing axis, are unchanged from the original.

    A cleaner, non-colliding alternative rig lives in scene_altdraw.js and is
    used by pendularm_altdraw.html (see that file's own header for why).

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see pendularm.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// #region create-scene
// The stand (legs/sidebars/crossbar) is built at a fixed size, exactly as in
// the original -- it does not scale with the pendulum's own length/mass
// parameters, only the arm itself does (in buildPendulumRig() below).
function createScene(container, pendulum) {

    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f5f5);

    var camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 200);
    camera.position.set(6, 4, 7);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(renderer.domElement);

    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 1.5, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.update();

    scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.1));
    var sun = new THREE.DirectionalLight(0xffffff, 0.9);
    sun.position.set(6, 10, 4);
    scene.add(sun);

    var ground = new THREE.Mesh(
        new THREE.PlaneGeometry(30, 30),
        new THREE.MeshStandardMaterial({ color: 0xe4e6ea, roughness: 1 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -1.75;
    scene.add(ground);
    scene.add(new THREE.GridHelper(30, 30, 0xc7cad1, 0xdddfe4).translateY(-1.75));

    buildStand(scene);
    var rig = buildPendulumRig(scene, pendulum);

    var gravityArrow = makeArrow(0x1a73e8), controlArrow = makeArrow(0xe37400);
    scene.add(gravityArrow, controlArrow);

    return {
        scene: scene, camera: camera, renderer: renderer, controls: controls,
        pendulumGeom: rig.pendulumGeom, pendulumMass: rig.pendulumMass, tipMesh: rig.tipMesh,
        gravityArrow: gravityArrow, controlArrow: controlArrow
    };
}
// #endregion create-scene

// #region build-stand
// The original's translucent white 4-leg table: legs 1-4 at the four
// corners, two sidebars (each a child of one leg) running lengthwise, and a
// crossbar (a child of sidebar1) running across -- ported call-for-call from
// the stencil's own createScene(), including which mesh is whose child.
function buildStand(scene) {

    var legMaterial = new THREE.MeshLambertMaterial({ color: 0xffffff, transparent: true, opacity: 0.3 });

    var legGeom = new THREE.CylinderGeometry(0.2, 0.2, 3.5, 20, 20, false);
    var leg1 = new THREE.Mesh(legGeom, legMaterial);
    var leg2 = new THREE.Mesh(legGeom, legMaterial);
    var leg3 = new THREE.Mesh(legGeom, legMaterial);
    var leg4 = new THREE.Mesh(legGeom, legMaterial);
    leg1.position.set(2, 0, 1);
    leg2.position.set(-2, 0, 1);
    leg3.position.set(-2, 0, -1);
    leg4.position.set(2, 0, -1);
    scene.add(leg1, leg2, leg3, leg4);

    var sidebarGeom = new THREE.CylinderGeometry(0.2, 0.2, 4.0, 20, 20, false);
    var sidebar1 = new THREE.Mesh(sidebarGeom, legMaterial);
    sidebar1.rotateOnAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    sidebar1.position.set(-2, 1.5, 0);
    leg1.add(sidebar1);
    var sidebar2 = new THREE.Mesh(sidebarGeom, legMaterial);
    sidebar2.rotateOnAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    sidebar2.position.set(2, 1.5, 0);
    leg3.add(sidebar2);

    var crossbarGeom = new THREE.CylinderGeometry(0.2, 0.2, 2.0, 20, 20, false);
    var crossbar = new THREE.Mesh(crossbarGeom, legMaterial);
    crossbar.rotateOnAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    crossbar.position.set(0, 0, -1);
    sidebar1.add(crossbar);
}
// #endregion build-stand

// #region build-pendulum-rig
// pendulum.geom (a short red disk standing in for the motor housing) is
// added directly to the scene -- NOT nested under the stand -- at world
// position (0, 1.5, 0), exactly as the original leaves it (its own
// `crossbar.add(pendulum.geom)` line is commented out). The link and mass
// hang from it via the same chain of one-time rotateOnAxis() calls the
// original uses to orient a Y-aligned cylinder into a horizontal arm before
// the per-frame rotation.y swing takes over. The second link (double
// pendulum only) hangs from pendulum_mass the same way the original's
// pendulum2_link/pendulum2_mass do.
function buildPendulumRig(scene, pendulum) {

    var redMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 });

    var pendulumGeom = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 0.3, 20, 20, false), redMaterial);
    pendulumGeom.rotateOnAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    pendulumGeom.position.set(0, 1.5, 0);
    scene.add(pendulumGeom);

    var length0 = pendulum.length[0];
    var pendulumLink = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, length0, 20, 20, false), redMaterial);
    pendulumLink.rotateOnAxis(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    pendulumLink.position.set(0, 0, length0 / 2);
    pendulumGeom.add(pendulumLink);

    var massRadius0 = Math.sqrt(pendulum.mass[0] * 0.1);
    var pendulumMass = new THREE.Mesh(new THREE.SphereGeometry(massRadius0), redMaterial);
    pendulumMass.position.set(0, -length0 / 2, 0);
    pendulumLink.add(pendulumMass);

    var tipMesh = pendulumMass;

    if (pendulum.links === 2) {
        var length1 = pendulum.length[1];
        var pendulum2Link = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, length1, 20, 20, false), redMaterial);
        pendulum2Link.rotateOnAxis(new THREE.Vector3(0, 0, 1), Math.PI);
        pendulum2Link.position.set(0, -length1 / 2, 0);
        pendulumMass.add(pendulum2Link);

        var massRadius1 = Math.sqrt(pendulum.mass[1] * 0.1);
        var pendulum2Mass = new THREE.Mesh(new THREE.SphereGeometry(massRadius1), redMaterial);
        pendulum2Mass.position.set(0, length1 / 2, 0);
        pendulum2Link.add(pendulum2Mass);

        tipMesh = pendulum2Mass;
    }

    return { pendulumGeom: pendulumGeom, pendulumMass: pendulumMass, tipMesh: tipMesh };
}
// #endregion build-pendulum-rig

function makeArrow(color) {
    var arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.001, color, 0.2, 0.12);
    arrow.visible = false;
    return arrow;
}

// #region update-pendulum-meshes
// pendulum.geom.rotation.y drives the whole assembly (world-space, since
// pendulum.geom sits directly on the scene, not nested under any other
// rotated parent); for the double pendulum, pendulum_mass.rotation.z drives
// the second link on top of that -- exactly the original's own two
// animate()-time assignments, `pendulum.geom.rotation.y = pendulum.angle[0]`
// and `pendulum_mass.rotation.z = pendulum.angle[1]`.
function updatePendulumMeshes(rig, pendulum) {

    rig.pendulumGeom.rotation.y = pendulum.angle[0];
    if (pendulum.links === 2) rig.pendulumMass.rotation.z = pendulum.angle[1];

    var tipWorld = new THREE.Vector3();
    rig.tipMesh.getWorldPosition(tipWorld);

    var gravityTorque = -(pendulum.mass[0]) * pendulum.gravity * pendulum.length[0] * Math.sin(pendulum.angle[0]);
    var controlTorque = pendulum.control[0];

    updateArrow(rig.gravityArrow, tipWorld, gravityTorque);
    updateArrow(rig.controlArrow, tipWorld, controlTorque);
}

function updateArrow(arrow, origin, signedTorque) {
    var mag = Math.min(2.5, Math.abs(signedTorque) * 0.01);
    if (mag < 0.02) { arrow.visible = false; return; }
    arrow.visible = true;
    arrow.position.copy(origin);
    arrow.setDirection(new THREE.Vector3(signedTorque >= 0 ? 1 : -1, 0, 0));
    arrow.setLength(mag, mag * 0.35, mag * 0.22);
}
// #endregion update-pendulum-meshes

// #region resize-renderer
function resizeRenderer(rig, container) {
    var w = container.clientWidth, h = container.clientHeight;
    rig.camera.aspect = w / h;
    rig.camera.updateProjectionMatrix();
    rig.renderer.setSize(w, h);
}
// #endregion resize-renderer

export { createScene, updatePendulumMeshes, resizeRenderer };
