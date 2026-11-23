/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Behavior Trees | three.js Scene

    Based on ../../inverse_kinematics/reference/scene.js, which is in turn
    based on ../../forward_kinematics/reference/scene.js. The approach is
    unchanged from both and the reason is worth repeating: the link meshes
    are deliberately NOT nested as three.js parent-child Object3Ds. Every
    mesh sits flat in the scene with matrixAutoUpdate disabled and is
    posed each frame straight from the already-composed
    robot.links[name].xform, so nothing here quietly redoes the
    matrix-stack composition kinematics.js is responsible for.

    THE ONLY FILE IN THIS MODULE THAT IMPORTS three.js. Everything else --
    the tick engine, the trajectory, the action, the trees, the robot
    descriptions -- is plain data and plain functions, which is what lets
    the same code run in the deck's Vue panels and in
    tests/smoke.cjs under Node with no renderer at all.

    What this module adds over the IK version:

      - OBSTACLES. The committed plan was found in a world of 24 spheres
        (worlds/world_arm_workspace.js), and a trajectory that avoids
        them is only interesting if they are on screen.
      - WAYPOINT GHOSTS. One translucent marker per trajectory waypoint,
        at the endeffector's position for that configuration, so the
        discrete plan and the continuous motion tracking it are visible at
        the same time.
      - A DESIRED-VS-ACTUAL PAIR of endeffector markers. The action's
        feedback has desired, actual and error fields; this is those three
        numbers drawn. When they separate, the servo is lagging.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see behavior_trees.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// #region make-link-geometry
// Bakes a link's fixed visual offset into the geometry once, at load
// time -- rpy in the same Rz*Ry*Rx order matrix_from_rpy uses.
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
function createViewer(container) {
    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f5f5);

    var camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.05, 200);
    camera.position.set(4.0, 3.0, 4.6);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(renderer.domElement);

    var controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 1.1, 0);
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

    return { scene: scene, camera: camera, renderer: renderer, controls: controls, linkMeshes: {}, extras: [] };
}
// #endregion create-viewer

// #region robot-meshes
function buildRobotMeshes(viewer, robot) {
    var linkMeshes = {};
    var name;
    for (name in robot.links) {
        var link = robot.links[name];
        var material = new THREE.MeshLambertMaterial({
            color: link.color !== undefined ? link.color : 0x8a8f98
        });
        var mesh = new THREE.Mesh(makeLinkGeometry(link.geometry), material);
        mesh.matrixAutoUpdate = false;
        viewer.scene.add(mesh);
        linkMeshes[name] = mesh;
    }
    viewer.linkMeshes = linkMeshes;
    return viewer;
}

// Copies each link's composed world matrix onto its mesh. THREE.Matrix4
// .set takes sixteen arguments in ROW-MAJOR order, which is the same
// convention matrix.js uses (m[row][col]), so this is a direct copy and
// not a transpose.
//
// `.xform`, NOT `.matrix`. The snapshot in kinematics.js is the VIEWER's
// copy, which stores each link's and joint's world transform as `.xform`
// to match the upstream kineval-stencil's own kineval_threejs.js. The
// forward_kinematics/ and inverse_kinematics/ modules' own copies of that
// file call the same field `.matrix`, so code moved between modules
// breaks silently on a field that is merely absent -- which is exactly
// how this file was first written, and the symptom was matrix_multiply
// throwing on undefined several frames in. The deck's TypeScript port
// keeps the `.matrix` name, because it was ported from the IK module.
function updateRobotMeshes(viewer, robot) {
    var name;
    for (name in robot.links) {
        var m = robot.links[name].xform;
        if (!m || !viewer.linkMeshes[name]) continue;
        viewer.linkMeshes[name].matrix.set(
            m[0][0], m[0][1], m[0][2], m[0][3],
            m[1][0], m[1][1], m[1][2], m[1][3],
            m[2][0], m[2][1], m[2][2], m[2][3],
            m[3][0], m[3][1], m[3][2], m[3][3]
        );
    }
}
// #endregion robot-meshes

// The planning world's obstacle spheres, drawn at their real radii.
//
// Translucent on purpose: the arm has to be visible THROUGH them, since
// the whole claim being illustrated is that the trajectory threads
// between them. Opaque spheres hide exactly the moment worth watching.
// #region obstacle-meshes
function buildObstacleMeshes(viewer, world) {
    var material = new THREE.MeshLambertMaterial({
        color: 0xc0392b, transparent: true, opacity: 0.35
    });
    var i;
    for (i = 0; i < world.obstacles.length; i++) {
        var obstacle = world.obstacles[i];
        var mesh = new THREE.Mesh(new THREE.SphereGeometry(obstacle.radius, 20, 16), material);
        mesh.position.set(obstacle.location[0][0], obstacle.location[1][0], obstacle.location[2][0]);
        viewer.scene.add(mesh);
        viewer.extras.push(mesh);
    }
}
// #endregion obstacle-meshes

// One small translucent sphere per trajectory waypoint, placed at the
// endeffector's world position for that waypoint's configuration.
//
// Computed by posing a THROWAWAY COPY of the robot at each waypoint and
// running the same forward kinematics the renderer uses, rather than by
// any separate formula -- so the ghosts cannot disagree with where the
// arm actually goes. Returns the positions as well, for the HUD.
// #region waypoint-ghosts
function buildWaypointGhosts(viewer, robotFactory, trajectory) {
    var ghost = robotFactory();
    initRobot(ghost);

    var positions = [];
    var i, j;
    for (i = 0; i < trajectory.points.length; i++) {
        for (j = 0; j < trajectory.joint_names.length; j++) {
            ghost.joints[trajectory.joint_names[j]].angle = trajectory.points[i].positions[j];
        }
        buildFKTransforms(ghost);

        var frame = ghost.joints[ghost.endeffector.frame].xform;
        var p = matrix_multiply(frame, ghost.endeffector.position);
        positions.push([p[0][0], p[1][0], p[2][0]]);

        var mesh = new THREE.Mesh(
            new THREE.SphereGeometry(0.055, 16, 12),
            new THREE.MeshLambertMaterial({
                // the first and last waypoints are the plan's start and
                // goal, and are worth telling apart from the ones in
                // between at a glance
                color: (i === 0) ? 0x3b6ea5 : (i === trajectory.points.length - 1 ? 0x27966b : 0x8a8f98),
                transparent: true, opacity: 0.75
            })
        );
        mesh.position.set(positions[i][0], positions[i][1], positions[i][2]);
        viewer.scene.add(mesh);
        viewer.extras.push(mesh);
    }
    return positions;
}
// #endregion waypoint-ghosts

// Two endeffector markers: where the trajectory says the endeffector
// should be (desired, magenta) and where it is (actual, orange).
//
// The gap between them IS the feedback message's `error` field, drawn.
// A P servo lags, so the gap is never zero while the arm is moving, and
// seeing that is the difference between understanding a trajectory
// controller and assuming it teleports.
// #region marker-pair
function createMarkerPair(viewer) {
    var desired = new THREE.Mesh(
        new THREE.SphereGeometry(0.05, 16, 16),
        new THREE.MeshLambertMaterial({ color: 0xe8349a })
    );
    var actual = new THREE.Mesh(
        new THREE.SphereGeometry(0.045, 16, 16),
        new THREE.MeshLambertMaterial({ color: 0xe8710a })
    );
    viewer.scene.add(desired);
    viewer.scene.add(actual);
    return { desired: desired, actual: actual };
}

function updateMarkerPair(pair, desiredPosition, actualPosition) {
    if (desiredPosition) pair.desired.position.set(desiredPosition[0], desiredPosition[1], desiredPosition[2]);
    if (actualPosition) pair.actual.position.set(actualPosition[0], actualPosition[1], actualPosition[2]);
}
// #endregion marker-pair

// Removes everything built per-run (obstacles, ghosts) so a reset does
// not leave the previous run's geometry in the scene. Disposes rather
// than merely detaching: a page left running for a while otherwise
// accumulates GPU buffers for every reset.
// #region clear-extras
function clearExtras(viewer) {
    var i;
    for (i = 0; i < viewer.extras.length; i++) {
        var mesh = viewer.extras[i];
        viewer.scene.remove(mesh);
        mesh.geometry.dispose();
    }
    viewer.extras = [];
}
// #endregion clear-extras

// #region resize-renderer
function resizeRenderer(viewer, container) {
    var w = container.clientWidth, h = container.clientHeight;
    if (!w || !h) return;
    viewer.camera.aspect = w / h;
    viewer.camera.updateProjectionMatrix();
    viewer.renderer.setSize(w, h);
}
// #endregion resize-renderer

export {
    createViewer, buildRobotMeshes, updateRobotMeshes, buildObstacleMeshes,
    buildWaypointGhosts, createMarkerPair, updateMarkerPair, clearExtras, resizeRenderer
};
