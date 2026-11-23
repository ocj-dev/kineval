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
// #region skeleton-geometry
// The ported fetch/baxter/sawyer descriptions carry no geometry (their real
// meshes are 20-35MB apiece and stay out of this repo), so build a stand-in
// from the kinematics itself: each link becomes a slim box reaching from its
// own origin toward its first child joint, which is exactly the information
// the URDF already gives us. The result is a recognisable stick figure whose
// proportions are the robot's real link lengths, and it is driven by the same
// forward kinematics as a fully-meshed robot.
function synthesizeSkeletonGeometry(THREE, robot, linkName) {
    var childJoints = robot.links[linkName].children || [];
    var span = null;

    for (var i = 0; i < childJoints.length; i++) {
        var o = robot.joints[childJoints[i]].origin.xyz;
        var d = Math.sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
        if (d > 1e-4 && (span === null || d > span.d)) span = { d: d, o: o };
    }

    if (span === null) {
        // leaf link, or a child sitting exactly on this link's origin
        return new THREE.BoxGeometry(0.06, 0.06, 0.06);
    }

    var thickness = Math.max(0.03, Math.min(0.09, span.d * 0.22));
    var geom = new THREE.BoxGeometry(span.d, thickness, thickness);
    // lay the box along the direction of that child joint, then shift it so
    // it starts at this link's own origin rather than straddling it
    geom.translate(span.d / 2, 0, 0);
    var from = new THREE.Vector3(1, 0, 0);
    var to = new THREE.Vector3(span.o[0], span.o[1], span.o[2]).normalize();
    geom.applyMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(
        new THREE.Quaternion().setFromUnitVectors(from, to)
    ));
    return geom;
}
// #endregion skeleton-geometry

// Attaches a three.js mesh directly onto robot.links[x].geom (link) and
// robot.joints[x].geom (a small marker sphere per joint) -- matching
// upstream's own robot.links[x].geom/robot.joints[x].geom convention in
// kineval_threejs.js/kineval.js#robotDraw, so this file and
// userinput.js/controls.js read/write the exact fields upstream does.
function buildRobotMeshes(viewer, robot, links_geom) {
    // Materials are SHARED across links/joints rather than cloned per
    // mesh. updateRobotMeshes swaps whole materials every frame to
    // highlight the active joint and the colliding link, so a per-mesh
    // clone would be allocated, immediately replaced by the shared one,
    // and then leaked -- and on a robot like the crawler (25 links, 24
    // joints) that is 49 orphaned materials per robot switch.
    var rig = {
        linkMaterial: new THREE.MeshLambertMaterial({ color: 0x8a8f98 }),
        collisionMaterial: new THREE.MeshLambertMaterial({ color: 0xdb3b3b }),
        jointMaterial: new THREE.MeshLambertMaterial({ color: 0x202b4e }),
        activeJointMaterial: new THREE.MeshLambertMaterial({ color: 0xffd34d }),
        endeffectorMaterial: new THREE.MeshLambertMaterial({ color: 0xe8710a })
    };

    var x;
    for (x in robot.links) {
        // robots ported without their meshes get a skeleton stand-in built
        // from their own joint offsets (see synthesizeSkeletonGeometry)
        var geom = links_geom[x] || synthesizeSkeletonGeometry(THREE, robot, x);
        var mesh = new THREE.Mesh(geom, rig.linkMaterial);
        mesh.matrixAutoUpdate = false;
        viewer.scene.add(mesh);
        robot.links[x].geom = mesh;
    }

    for (x in robot.joints) {
        var marker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 12, 12), rig.jointMaterial);
        marker.matrixAutoUpdate = false;
        viewer.scene.add(marker);
        robot.joints[x].geom = marker;
    }

    rig.endeffectorMarker = new THREE.Mesh(new THREE.SphereGeometry(0.08, 12, 12), rig.endeffectorMaterial);
    rig.endeffectorMarker.matrixAutoUpdate = false;
    viewer.scene.add(rig.endeffectorMarker);

    return rig;
}

function removeRobotMeshes(viewer, robot, rig) {
    var x;
    for (x in robot.links) {
        viewer.scene.remove(robot.links[x].geom);
        robot.links[x].geom.geometry.dispose();
    }
    for (x in robot.joints) {
        viewer.scene.remove(robot.joints[x].geom);
        robot.joints[x].geom.geometry.dispose();
    }
    viewer.scene.remove(rig.endeffectorMarker);
    rig.endeffectorMarker.geometry.dispose();

    // the shared materials, disposed once each rather than once per mesh
    rig.linkMaterial.dispose();
    rig.collisionMaterial.dispose();
    rig.jointMaterial.dispose();
    rig.activeJointMaterial.dispose();
    rig.endeffectorMaterial.dispose();
}
// #endregion build-robot-meshes

// #region build-world-meshes
// Turns a world's plain {boundary, obstacles} data into three.js objects.
// worlds/*.js deliberately return data and nothing else, so this file
// stays the only one in the viewer that knows three.js exists -- the same
// split the robot files already use (they return links_geom, and
// buildRobotMeshes turns it into meshes).
//
// The boundary is drawn as a line loop on the ground plane rather than as
// a solid, because it is an x-z extent with no ceiling: a wall would
// imply the robot cannot be lifted over it, and it can
// (collision.js#linkBoundaryViolation tests x and z only).
function buildWorldMeshes(viewer, world) {
    var group = new THREE.Group();

    var obstacleMaterial = new THREE.MeshLambertMaterial({ color: 0x4a7ebb });
    var i;
    for (i = 0; i < world.obstacles.length; i++) {
        var obstacle = world.obstacles[i];
        var mesh = new THREE.Mesh(
            new THREE.SphereGeometry(obstacle.radius, 16, 12),
            obstacleMaterial
        );
        mesh.position.set(
            obstacle.location[0][0], obstacle.location[1][0], obstacle.location[2][0]
        );
        group.add(mesh);
    }

    var b = world.boundary;
    var corners = [
        new THREE.Vector3(b[0][0], 0.01, b[0][2]),
        new THREE.Vector3(b[1][0], 0.01, b[0][2]),
        new THREE.Vector3(b[1][0], 0.01, b[1][2]),
        new THREE.Vector3(b[0][0], 0.01, b[1][2]),
        new THREE.Vector3(b[0][0], 0.01, b[0][2])
    ];
    group.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(corners),
        new THREE.LineBasicMaterial({ color: 0xcc3333 })
    ));

    viewer.scene.add(group);
    return { group: group, obstacleMaterial: obstacleMaterial };
}

function removeWorldMeshes(viewer, worldRig) {
    if (!worldRig) return;
    worldRig.group.traverse(function (child) {
        if (child.geometry) child.geometry.dispose();
    });
    worldRig.obstacleMaterial.dispose();
    viewer.scene.remove(worldRig.group);
}
// #endregion build-world-meshes

// #region planner-visualization
// Draws the search: one small marker at each tree vertex's BASE location,
// and the found path as a line through those locations.
//
// Base location only, which is worth being explicit about: a tree vertex
// is a point in an n-dimensional configuration space and there is no
// honest way to draw one in three dimensions. What is drawn is its
// shadow -- where the base would be -- so two vertices with identical
// base poses and completely different arm configurations land on top of
// each other. The picture is a projection, not the tree. Upstream does
// the same thing (add_config_origin_indicator_geom) without saying so.
//
// Rebuilt wholesale rather than incrementally: the planner adds vertices
// in bursts of tens per frame, and a thousand individually-added meshes
// costs far more than one rebuilt InstancedMesh.
function updatePlannerMeshes(viewer, plannerRig, planner) {
    if (plannerRig.group) {
        plannerRig.group.traverse(function (child) {
            if (child.geometry) child.geometry.dispose();
        });
        viewer.scene.remove(plannerRig.group);
        plannerRig.group = null;
    }
    if (!planner) return;

    var group = new THREE.Group();
    var trees = [planner.T_a, planner.T_b];
    var vertexGeometry = new THREE.BoxGeometry(0.06, 0.06, 0.06);

    var t, i;
    for (t = 0; t < trees.length; t++) {
        if (!trees[t]) continue;
        var count = trees[t].vertices.length;
        var instanced = new THREE.InstancedMesh(vertexGeometry, plannerRig.treeMaterial, count);
        var matrix = new THREE.Matrix4();
        for (i = 0; i < count; i++) {
            var q = trees[t].vertices[i].vertex;
            matrix.makeTranslation(q[0], q[1], q[2]);
            instanced.setMatrixAt(i, matrix);
        }
        instanced.instanceMatrix.needsUpdate = true;
        group.add(instanced);
    }

    if (planner.state === 'found' && planner.path && planner.path.length > 1) {
        var points = planner.path.map(function (q) {
            return new THREE.Vector3(q[0], q[1], q[2]);
        });
        group.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(points),
            plannerRig.pathMaterial
        ));
    }

    viewer.scene.add(group);
    plannerRig.group = group;
}

function createPlannerRig() {
    return {
        group: null,
        treeMaterial: new THREE.MeshLambertMaterial({
            color: 0xffd34d, transparent: true, opacity: 0.7
        }),
        // red, matching upstream's own found-path highlight
        // (kineval_rrt_connect.js:536, `color = {r:1,g:0,b:0}`)
        pathMaterial: new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2 })
    };
}
// #endregion planner-visualization

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
    // robot.collision is poseIsCollision's verdict for the current pose,
    // refreshed once per frame by robotIsCollision -- so colouring the
    // offending link red is free, and it is why poseIsCollision returns
    // WHICH link rather than a bare boolean. This is upstream's own
    // "colliding link turns red" behaviour (collision lecture, slide 16).
    var collidingLink = (robot.collision && robot.collision.link) || null;
    for (x in robot.links) {
        robot.links[x].geom.matrix.copy(matrix4FromArray(robot.links[x].xform));
        robot.links[x].geom.material = (x === collidingLink)
            ? rig.collisionMaterial : rig.linkMaterial;
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

// #region frame-camera
// These robots differ enormously in scale -- urdf_example spans about a
// metre, baxter and fetch a couple -- so fit the camera to whatever is
// loaded rather than assuming one framing suits all of them.
function frameCameraToRobot(viewer, robot) {
    var box = new THREE.Box3();
    var name, any = false;
    for (name in robot.links) {
        var mesh = robot.links[name].geom;
        if (!mesh) continue;
        mesh.updateMatrixWorld(true);
        box.expandByObject(mesh);
        any = true;
    }
    if (!any || box.isEmpty()) return;

    var centre = box.getCenter(new THREE.Vector3());
    var radius = box.getBoundingSphere(new THREE.Sphere()).radius || 1;
    var dist = radius / Math.sin((viewer.camera.fov * Math.PI) / 180 / 2);

    viewer.camera.position.set(centre.x + dist * 0.62, centre.y + dist * 0.45, centre.z + dist * 0.70);
    viewer.camera.near = Math.max(0.01, dist / 200);
    viewer.camera.far = dist * 200;
    viewer.camera.updateProjectionMatrix();
    viewer.controls.target.copy(centre);
    viewer.controls.update();
}
// #endregion frame-camera

// #region frame-camera-to-world
// Frames the whole planning world rather than the robot.
//
// frameCameraToRobot fits the camera to the robot at load time, which is
// right for inspecting kinematics and wrong the moment the robot drives
// somewhere -- step through a plan and the robot walks straight out of
// shot. Worlds here span anything from 10m to 52m across, so there is no
// one framing that suits both.
function frameCameraToWorld(viewer, world) {
    var b = world.boundary;
    var cx = (b[0][0] + b[1][0]) / 2;
    var cz = (b[0][2] + b[1][2]) / 2;
    var radius = 0.5 * Math.sqrt(
        Math.pow(b[1][0] - b[0][0], 2) + Math.pow(b[1][2] - b[0][2], 2)
    ) || 1;
    var dist = radius / Math.sin((viewer.camera.fov * Math.PI) / 180 / 2);

    viewer.camera.position.set(cx, dist * 0.85, cz + dist * 0.85);
    viewer.camera.near = Math.max(0.01, dist / 200);
    viewer.camera.far = dist * 200;
    viewer.camera.updateProjectionMatrix();
    viewer.controls.target.set(cx, 0, cz);
    viewer.controls.update();
}
// #endregion frame-camera-to-world

// #region resize-renderer
function resizeViewer(viewer, container) {
    var w = container.clientWidth, h = container.clientHeight;
    viewer.camera.aspect = w / h;
    viewer.camera.updateProjectionMatrix();
    viewer.renderer.setSize(w, h);
}
// #endregion resize-renderer

export {
    createViewer, buildRobotMeshes, removeRobotMeshes, updateRobotMeshes,
    frameCameraToRobot, frameCameraToWorld, resizeViewer,
    buildWorldMeshes, removeWorldMeshes, createPlannerRig, updatePlannerMeshes
};
