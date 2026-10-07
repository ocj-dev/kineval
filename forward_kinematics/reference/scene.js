/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | three.js Scene

    Deliberately does NOT use three.js's own parent-child Object3D nesting to
    pose the robot (the way simulation/reference/scene.js nests pendulum
    meshes and lets three.js compose their transforms for you) -- that would
    silently redo, in the engine, the exact matrix-stack composition this
    module's lesson is about doing yourself. Instead every link mesh is
    added FLAT, directly to the scene, with matrixAutoUpdate disabled, and
    updateRobotMeshes() below copies the already-fully-composed 4x4 world
    matrix kinematics.js computed for that link (robot.links[name].matrix)
    straight onto the mesh. The one thing three.js's engine *is* trusted
    for is baking each link's own fixed visual offset into its geometry
    once, at creation time (makeLinkGeometry below) -- exactly how the
    upstream stencil's own robot files (e.g. robot_urdf_example.js) use
    geometry.applyMatrix() for the same purpose.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see forward_kinematics.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// #region make-link-geometry
// Bakes a link's fixed visual offset (xyz translation, rpy fixed-axis
// rotation -- same Rz*Ry*Rx order as matrix_from_rpy in kinematics.js) into
// the geometry itself, once, at load time. The DFS traversal and the
// per-frame mesh update below only ever deal with each link's/joint's own
// VARIABLE world transform on top of this.
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

// #region create-viewer
// One-time setup: camera/renderer/controls/lighting/ground. Does not know
// about any particular robot -- buildRobotMeshes/removeRobotMeshes below
// are called separately (and re-called whenever the ?robot= test case
// switches between mobile_arm and urdf_example) against this same viewer.
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

function buildRobotMeshes(viewer, robot) {
    var linkMeshes = {};
    var highlightMaterial = new THREE.MeshLambertMaterial({ color: 0xffd34d });

    var name;
    for (name in robot.links) {
        var link = robot.links[name];
        var material = new THREE.MeshLambertMaterial({ color: link.color !== undefined ? link.color : 0x8a8f98 });
        var mesh = new THREE.Mesh(makeLinkGeometry(link.geometry), material);
        mesh.matrixAutoUpdate = false;
        mesh.visible = false; // revealed progressively as the DFS step-through reaches each link
        viewer.scene.add(mesh);
        linkMeshes[name] = mesh;
    }

    viewer.linkMeshes = linkMeshes;
    viewer.highlightMaterial = highlightMaterial;
    viewer.defaultMaterials = collectMaterials(linkMeshes);
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

function collectMaterials(linkMeshes) {
    var materials = {}, name;
    for (name in linkMeshes) materials[name] = linkMeshes[name].material;
    return materials;
}
// #endregion create-viewer

// #region update-robot-meshes
// Copies each link's already-fully-composed world matrix (computed by
// traverseFKLink in kinematics.js) directly onto its mesh. THREE.Matrix4.set
// takes its sixteen arguments in ROW-MAJOR order -- the same convention
// matrix.js uses for every matrix (m[row][col]) -- so this is a direct,
// unmodified copy, not a transpose.
function updateRobotMeshes(rig, robot) {
    var name;
    for (name in robot.links) {
        var m = robot.links[name].matrix;
        rig.linkMeshes[name].matrix.set(
            m[0][0], m[0][1], m[0][2], m[0][3],
            m[1][0], m[1][1], m[1][2], m[1][3],
            m[2][0], m[2][1], m[2][2], m[2][3],
            m[3][0], m[3][1], m[3][2], m[3][3]
        );
    }
}

// Drives the step-through DFS visualization: a link mesh becomes visible
// the moment its own 'link' entry in the traversal order has been reached
// (so the robot visibly assembles itself base-outward as the steps
// advance), and the single most-recently-reached link is swapped to the
// highlight material -- everything already revealed keeps its normal
// material, matching how MatrixStackPanel.vue highlights the deck's own
// pseudocode panel one line at a time rather than leaving prior lines lit.
function revealLinksUpTo(rig, order, stepIndex) {
    var mostRecentLink = null;
    var i;
    for (i = 0; i <= stepIndex && i < order.length; i++) {
        if (order[i].type === 'link') {
            rig.linkMeshes[order[i].name].visible = true;
            mostRecentLink = order[i].name;
        }
    }

    var linkName;
    for (linkName in rig.linkMeshes) {
        rig.linkMeshes[linkName].material = rig.defaultMaterials[linkName];
    }
    if (mostRecentLink) {
        rig.linkMeshes[mostRecentLink].material = rig.highlightMaterial;
    }
}
// #endregion update-robot-meshes

// #region resize-renderer
function resizeRenderer(rig, container) {
    var w = container.clientWidth, h = container.clientHeight;
    rig.camera.aspect = w / h;
    rig.camera.updateProjectionMatrix();
    rig.renderer.setSize(w, h);
}
// #endregion resize-renderer

export { createViewer, buildRobotMeshes, removeRobotMeshes, updateRobotMeshes, revealLinksUpTo, resizeRenderer };
