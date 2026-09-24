/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Pendularm Dynamical Simulation | three.js Scene

    Builds and updates the orbit-able 3D scene: a simple stand, the pendulum
    arm(s) as a nested pivot hierarchy (so the double-pendulum's second link
    renders correctly even though its equations of motion use an ABSOLUTE
    angle from vertical, not an angle relative to link 1 -- see
    updatePendulumMeshes() below), and vector-arrow overlays for the
    gravity-restoring and motor-control torques. A current three.js release
    is vendored locally (vendor/three/) as ES modules, in place of the
    upstream stencil's pinned non-module r92 build.

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

var PIVOT_HEIGHT = 4.2;   // world-space height of the top pivot above the ground plane
var ROD_RADIUS = 0.06;
var MASS_COLOR = 0xB3261E;
var ROD_COLOR = 0x3a3a3a;
var STAND_COLOR = 0x9aa0a6;

// #region create-scene
// A minimal orbit-able stand: one vertical post and a short crossbar holding
// the pivot at PIVOT_HEIGHT, deliberately simpler than the upstream
// stencil's 4-leg table rig -- the physics is 1D/2D-planar regardless, so
// the stand exists only to give the eye a fixed spatial reference while
// orbiting, not to imply any structural role.
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
    controls.target.set(0, PIVOT_HEIGHT * 0.6, 0);
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
    scene.add(ground);
    scene.add(new THREE.GridHelper(30, 30, 0xc7cad1, 0xdddfe4));

    var standMaterial = new THREE.MeshStandardMaterial({ color: STAND_COLOR, roughness: 0.6 });
    var post = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.16, PIVOT_HEIGHT, 16), standMaterial);
    post.position.y = PIVOT_HEIGHT / 2;
    scene.add(post);
    var crossbar = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 1.2, 12), standMaterial);
    crossbar.rotation.z = Math.PI / 2;
    crossbar.position.set(0.5, PIVOT_HEIGHT, 0);
    scene.add(crossbar);

    var pivot = new THREE.Group();
    pivot.position.set(0.5, PIVOT_HEIGHT, 0);
    scene.add(pivot);

    var links = buildLinks(pivot, pendulum);

    return { scene: scene, camera: camera, renderer: renderer, controls: controls, pivot: pivot, links: links,
        gravityArrow: makeArrow(0x1a73e8), controlArrow: makeArrow(0xe37400) };
}
// #endregion create-scene

function makeArrow(color) {
    var arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.001, color, 0.2, 0.12);
    arrow.visible = false;
    return arrow;
}

// #region build-links
// Link 2 (when present) is nested as a CHILD of link 1's group so it moves
// with link 1's swing, but pendulum.angle[1] is an ABSOLUTE angle from
// vertical (matching doublePendulumAcceleration()'s own convention), not an
// angle relative to link 1. Three.js composes a child's rotation with its
// parent's, so link 2's group is instead updated every frame to
// (angle[1] - angle[0]) -- the relative rotation that, composed with link
// 1's absolute rotation, reproduces link 2's true absolute angle in world
// space. See updatePendulumMeshes() below.
function buildLinks(pivot, pendulum) {

    var out = [];
    var parent = pivot;

    for (var i = 0; i < pendulum.links; i++) {

        var group = new THREE.Group();
        parent.add(group);

        var length = pendulum.length[i];
        var rodMaterial = new THREE.MeshStandardMaterial({ color: ROD_COLOR, roughness: 0.5 });
        var rod = new THREE.Mesh(new THREE.CylinderGeometry(ROD_RADIUS, ROD_RADIUS, length, 16), rodMaterial);
        rod.position.y = -length / 2;
        group.add(rod);

        var massRadius = Math.max(0.12, Math.sqrt(pendulum.mass[i]) * 0.16);
        var mass = new THREE.Mesh(
            new THREE.SphereGeometry(massRadius, 24, 16),
            new THREE.MeshStandardMaterial({ color: MASS_COLOR, roughness: 0.35 })
        );
        mass.position.y = -length;
        group.add(mass);

        var childAnchor = new THREE.Group();
        childAnchor.position.y = -length;
        group.add(childAnchor);

        out.push({ group: group, length: length });
        parent = childAnchor;
    }

    return out;
}
// #endregion build-links

// #region update-pendulum-meshes
// Applies pendulum.angle (absolute, from the downward vertical) to the
// nested link-group hierarchy, and points the gravity/control torque arrows
// from each link's mass position, scaled by the magnitude of the torque
// they represent so a bigger swing/harder correction is visibly a bigger
// arrow.
function updatePendulumMeshes(rig, pendulum) {

    for (var i = 0; i < rig.links.length; i++) {
        var relative = (i === 0) ? pendulum.angle[0] : pendulum.angle[i] - pendulum.angle[i - 1];
        rig.links[i].group.rotation.z = relative;
    }

    var tipWorld = new THREE.Vector3();
    rig.links[rig.links.length - 1].group.children[1].getWorldPosition(tipWorld);

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
