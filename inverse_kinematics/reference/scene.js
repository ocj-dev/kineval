/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Inverse Kinematics | three.js Scene

    Based directly on ../../forward_kinematics/reference/scene.js -- same
    rendering approach for the same reason (see that file's header):
    deliberately NOT using three.js's own parent-child Object3D nesting to
    pose the robot, so nothing here silently redoes, in the engine, the
    matrix-stack composition kinematics.js is responsible for. Every link
    mesh sits FLAT in the scene with matrixAutoUpdate disabled, posed every
    frame straight from the already-computed robot.links[name].matrix.

    Three differences from the forward_kinematics/ version:

      1. Meshes default to VISIBLE immediately (forward_kinematics/'s
         buildRobotMeshes hides them for progressive DFS-order reveal; this
         module has no such traversal-order stepping -- IK's "stepping" is
         through Jacobian iterations, not tree order, so the whole arm is
         always on screen, converging toward the target).

      2. buildRobotMeshes takes an optional `linksGeom` map of pre-built
         THREE geometry, keyed by link name -- needed for robots/robot_mr2.js
         (copied verbatim from ../../kineval/robots/robot_mr2.js), which
         builds its own THREE.BoxGeometry per link directly rather than the
         plain {type,size,offset} descriptors urdf_example.js/
         mobile_arm.urdf.js use. When a link name is present in linksGeom,
         that pre-built geometry wins; otherwise makeLinkGeometry runs on the
         link's plain descriptor, same as forward_kinematics/.

      3. instantiateThreeRobot (new) exists so this file can stay the ONLY
         one importing three.js, even for robot_mr2.js's createRobotMr2(THREE)
         factory -- which (ported faithfully, see robots/robot_mr2.js's own
         header) expects a THREE namespace handed to it as an argument rather
         than importing one itself. infrastructure.js's ROBOT_FACTORIES holds
         the bare createRobotMr2 function reference; the HTML page's module
         script calls scene.js's exported instantiateThreeRobot(createRobotMr2)
         to run it, never importing THREE on its own.

      4. createTargetMarker/updateTargetMarker and
         createEndeffectorMarker/updateEndeffectorMarker (new) -- small
         sphere markers for the IK target position and the end-effector's
         current world position, so convergence is visible as the orange
         end-effector marker sliding toward the magenta target marker.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see inverse_kinematics.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// #region make-link-geometry
// Bakes a link's fixed visual offset (xyz translation, rpy fixed-axis
// rotation -- same Rz*Ry*Rx order as matrix_from_rpy in kinematics.js) into
// the geometry itself, once, at load time -- same role as
// forward_kinematics/reference/scene.js's own makeLinkGeometry.
function makeLinkGeometry(desc) {
    var geom;
    if (desc.type === 'box') {
        geom = new THREE.BoxGeometry(desc.size[0], desc.size[1], desc.size[2]);
    } else if (desc.type === 'cylinder') {
        geom = new THREE.CylinderGeometry(desc.size[0], desc.size[0], desc.size[1], 24);
    } else {
        geom = new THREE.SphereGeometry(desc.size ? desc.size[0] : 0.1, 16, 16);
    }

    var rpy = desc.offset.rpy;
    geom.rotateX(rpy[0]);
    geom.rotateY(rpy[1]);
    geom.rotateZ(rpy[2]);
    geom.translate(desc.offset.xyz[0], desc.offset.xyz[1], desc.offset.xyz[2]);

    return geom;
}
// #endregion make-link-geometry

// #region instantiate-three-robot
// Runs a robot_mr2.js-style factory (which expects a THREE namespace handed
// in as an argument) without that factory, or anything else outside this
// file, ever importing three.js itself. See this file's header, point 3.
function instantiateThreeRobot(factoryFn) {
    return factoryFn(THREE);
}
// #endregion instantiate-three-robot

// #region create-viewer
// One-time setup: camera/renderer/controls/lighting/ground. Does not know
// about any particular robot -- buildRobotMeshes/removeRobotMeshes below are
// called separately (and re-called whenever ?robot= switches between
// urdf_example/mobile_arm/mr2) against this same viewer.
function createViewer(container) {

    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f5f5);

    var camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.05, 200);
    camera.position.set(2.2, 2.0, 2.6);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(renderer.domElement);

    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.3, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.update();

    scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.1));
    var sun = new THREE.DirectionalLight(0xffffff, 0.9);
    sun.position.set(3, 5, 2);
    scene.add(sun);

    var ground = new THREE.Mesh(
        new THREE.PlaneGeometry(20, 20),
        new THREE.MeshStandardMaterial({ color: 0xe4e6ea, roughness: 1 })
    );
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);
    scene.add(new THREE.GridHelper(20, 20, 0xc7cad1, 0xdddfe4));
    scene.add(new THREE.AxesHelper(0.4));

    return { scene: scene, camera: camera, renderer: renderer, controls: controls };
}

function buildRobotMeshes(viewer, robot, linksGeom) {
    var linkMeshes = {};

    var name;
    for (name in robot.links) {
        var link = robot.links[name];
        var material = new THREE.MeshLambertMaterial({ color: link.color !== undefined ? link.color : 0x8a8f98 });
        var geom = (linksGeom && linksGeom[name]) ? linksGeom[name] : makeLinkGeometry(link.geometry);
        var mesh = new THREE.Mesh(geom, material);
        mesh.matrixAutoUpdate = false;
        mesh.visible = true; // IK has no DFS traversal to progressively reveal -- the whole arm is always shown
        viewer.scene.add(mesh);
        linkMeshes[name] = mesh;
    }

    viewer.linkMeshes = linkMeshes;
    return viewer;
}

function removeRobotMeshes(viewer) {
    var name;
    for (name in viewer.linkMeshes) {
        var mesh = viewer.linkMeshes[name];
        viewer.scene.remove(mesh);
        mesh.geometry.dispose();
        mesh.material.dispose();
    }
    viewer.linkMeshes = {};
}
// #endregion create-viewer

// #region update-robot-meshes
// Copies each link's already-fully-composed world matrix (computed by
// traverseFKLink in kinematics.js) directly onto its mesh. THREE.Matrix4.set
// takes its sixteen arguments in ROW-MAJOR order -- the same convention
// matrix.js uses for every matrix (m[row][col]) -- so this is a direct,
// unmodified copy, not a transpose.
function updateRobotMeshes(viewer, robot) {
    var name;
    for (name in robot.links) {
        var m = robot.links[name].matrix;
        viewer.linkMeshes[name].matrix.set(
            m[0][0], m[0][1], m[0][2], m[0][3],
            m[1][0], m[1][1], m[1][2], m[1][3],
            m[2][0], m[2][1], m[2][2], m[2][3],
            m[3][0], m[3][1], m[3][2], m[3][3]
        );
    }
}
// #endregion update-robot-meshes

// #region target-marker
// Small sphere markers for the IK target position (magenta, fixed by the
// user) and the end-effector's current world position (orange, slides as
// iterateIK converges) -- the one new visualization this module adds beyond
// forward_kinematics/'s scene.js, per this module's lab brief.
function createTargetMarker(viewer) {
    var marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.05, 16, 16),
        new THREE.MeshLambertMaterial({ color: 0xe8349a })
    );
    marker.matrixAutoUpdate = true;
    viewer.scene.add(marker);
    return marker;
}

function updateTargetMarker(marker, position) {
    marker.position.set(position[0], position[1], position[2]);
}

function createEndeffectorMarker(viewer) {
    var marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.045, 16, 16),
        new THREE.MeshLambertMaterial({ color: 0xe8710a })
    );
    marker.matrixAutoUpdate = true;
    viewer.scene.add(marker);
    return marker;
}

function updateEndeffectorMarker(marker, position) {
    marker.position.set(position[0], position[1], position[2]);
}
// #endregion target-marker

// #region resize-renderer
function resizeRenderer(viewer, container) {
    var w = container.clientWidth, h = container.clientHeight;
    viewer.camera.aspect = w / h;
    viewer.camera.updateProjectionMatrix();
    viewer.renderer.setSize(w, h);
}
// #endregion resize-renderer

export {
    createViewer, buildRobotMeshes, removeRobotMeshes, updateRobotMeshes, resizeRenderer,
    instantiateThreeRobot, createTargetMarker, updateTargetMarker, createEndeffectorMarker, updateEndeffectorMarker
};
