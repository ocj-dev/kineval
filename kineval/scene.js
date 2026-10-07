/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Full KinEval Reference Viewer | three.js Scene

    Ports the rendering approach from the upstream kineval-stencil's own
    kineval_threejs.js + kineval.js#robotDraw: each link gets a mesh stored
    directly on robot.links[x].geom, each joint gets a small marker sphere
    stored on robot.joints[x].geom (upstream shows joint *axes*; this
    viewer shows a highlightable marker instead, same role -- indicating
    the currently-active joint, j/k/l/h-navigable per userinput.js), and
    the end-effector gets its own marker from robot.endeffector.frame/
    position, matching upstream's own endeffector-drawing block in
    robotDraw(). Like forward_kinematics/reference/scene.js, every mesh is
    posed directly from its already-computed .xform (kinematics.js) rather
    than via three.js's own parent-child scene-graph nesting.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see index.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// #region create-viewer
function createViewer(container) {
    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f5f5);

    var camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.05, 300);
    camera.position.set(4, 3, 5);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(renderer.domElement);

    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.5, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.update();

    scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.1));
    var sun = new THREE.DirectionalLight(0xffffff, 0.9);
    sun.position.set(4, 8, 3);
    scene.add(sun);

    var ground = new THREE.Mesh(
        new THREE.PlaneGeometry(40, 40),
        new THREE.MeshStandardMaterial({ color: 0xe4e6ea, roughness: 1 })
    );
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);
    scene.add(new THREE.GridHelper(40, 40, 0xc7cad1, 0xdddfe4));

    return { scene: scene, camera: camera, renderer: renderer, controls: controls };
}
// #endregion create-viewer

// #region build-robot-meshes
// Attaches a three.js mesh directly onto robot.links[x].geom (link) and
// robot.joints[x].geom (a small marker sphere per joint) -- matching
// upstream's own robot.links[x].geom/robot.joints[x].geom convention in
// kineval_threejs.js/kineval.js#robotDraw, so this file and
// userinput.js/controls.js read/write the exact fields upstream does.
function buildRobotMeshes(viewer, robot, links_geom) {
    var linkMaterial = new THREE.MeshLambertMaterial({ color: 0x8a8f98 });
    var jointMaterial = new THREE.MeshLambertMaterial({ color: 0x202b4e });
    var activeJointMaterial = new THREE.MeshLambertMaterial({ color: 0xffd34d });
    var endeffectorMaterial = new THREE.MeshLambertMaterial({ color: 0xe8710a });

    var x;
    for (x in robot.links) {
        var mesh = new THREE.Mesh(links_geom[x], linkMaterial.clone());
        mesh.matrixAutoUpdate = false;
        viewer.scene.add(mesh);
        robot.links[x].geom = mesh;
    }

    for (x in robot.joints) {
        var marker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 12, 12), jointMaterial.clone());
        marker.matrixAutoUpdate = false;
        viewer.scene.add(marker);
        robot.joints[x].geom = marker;
    }

    var endeffectorMarker = new THREE.Mesh(new THREE.SphereGeometry(0.08, 12, 12), endeffectorMaterial);
    endeffectorMarker.matrixAutoUpdate = false;
    viewer.scene.add(endeffectorMarker);

    return { linkMaterial: linkMaterial, jointMaterial: jointMaterial, activeJointMaterial: activeJointMaterial, endeffectorMarker: endeffectorMarker };
}

function removeRobotMeshes(viewer, robot, rig) {
    var x;
    for (x in robot.links) {
        viewer.scene.remove(robot.links[x].geom);
        robot.links[x].geom.geometry.dispose();
        robot.links[x].geom.material.dispose();
    }
    for (x in robot.joints) {
        viewer.scene.remove(robot.joints[x].geom);
        robot.joints[x].geom.geometry.dispose();
        robot.joints[x].geom.material.dispose();
    }
    viewer.scene.remove(rig.endeffectorMarker);
    rig.endeffectorMarker.geometry.dispose();
    rig.endeffectorMarker.material.dispose();
}
// #endregion build-robot-meshes

// #region update-robot-meshes
function matrix4FromArray(m) {
    var out = new THREE.Matrix4();
    out.set(
        m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3]
    );
    return out;
}

// Poses every link mesh and joint marker from its already-computed .xform,
// highlights kineval.params.active_joint's marker, and places the
// end-effector marker at robot.endeffector.frame's xform times its local
// offset -- the same three pieces of upstream's own robotDraw() this
// viewer's "forward kinematics aspect" needs (link display, active-joint
// display, end-effector display); the link/joint/collision/axes toggle
// switches upstream's robotDraw() also has are out of scope here.
function updateRobotMeshes(robot, rig, activeJointName) {
    var x;
    for (x in robot.links) {
        robot.links[x].geom.matrix.copy(matrix4FromArray(robot.links[x].xform));
    }
    for (x in robot.joints) {
        var isActive = x === activeJointName;
        // matrixAutoUpdate is off (matrix is set directly from .xform, not
        // recomposed from position/rotation/scale), so the active-joint
        // marker's larger size has to be baked into the matrix itself via
        // an explicit scale multiply, not a separate .scale assignment.
        var jointMat = matrix4FromArray(robot.joints[x].xform);
        if (isActive) jointMat.multiply(new THREE.Matrix4().makeScale(1.8, 1.8, 1.8));
        robot.joints[x].geom.matrix.copy(jointMat);
        robot.joints[x].geom.material = isActive ? rig.activeJointMaterial : rig.jointMaterial;
    }

    var eeFrame = robot.joints[robot.endeffector.frame];
    var eeWorld = matrix_multiply(eeFrame.xform, robot.endeffector.position);
    rig.endeffectorMarker.matrix.copy(matrix4FromArray(generate_translation_matrix(eeWorld[0][0], eeWorld[1][0], eeWorld[2][0])));
}
// #endregion update-robot-meshes

// #region resize-renderer
function resizeViewer(viewer, container) {
    var w = container.clientWidth, h = container.clientHeight;
    viewer.camera.aspect = w / h;
    viewer.camera.updateProjectionMatrix();
    viewer.renderer.setSize(w, h);
}
// #endregion resize-renderer

export { createViewer, buildRobotMeshes, removeRobotMeshes, updateRobotMeshes, resizeViewer };
