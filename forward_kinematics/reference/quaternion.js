/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Quaternion Transform Routines

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) kineval/kineval_quaternion.js module:
    quaternion_from_axisangle, quaternion_normalize, quaternion_multiply,
    quaternion_to_rotation_matrix.

    A quaternion is represented as a plain object q = {a,b,c,d}, with
    q.a the real/scalar part and q.b/q.c/q.d the i/j/k imaginary parts
    (q = a + bi + cj + dk), matching the AutoRob quaternions lecture's
    notation.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// #region quaternion-from-axisangle
function quaternion_from_axisangle(axis, angle) {
    // returns quaternion q = {a,b,c,d} for a rotation of "angle" radians
    // about the (not necessarily unit-length) 3-vector "axis":
    //   q = cos(angle/2) + sin(angle/2) * (axis_x i + axis_y j + axis_z k)
    var unitAxis = vector_normalize(axis);
    var halfAngle = angle / 2;
    var s = Math.sin(halfAngle);

    return {
        a: Math.cos(halfAngle),
        b: unitAxis[0] * s,
        c: unitAxis[1] * s,
        d: unitAxis[2] * s
    };
}
// #endregion quaternion-from-axisangle

// #region quaternion-normalize
function quaternion_normalize(q1) {
    // returns q1 rescaled to unit norm: q / sqrt(a^2+b^2+c^2+d^2)
    var norm = Math.sqrt(q1.a * q1.a + q1.b * q1.b + q1.c * q1.c + q1.d * q1.d);
    if (norm === 0) {
        return { a: 1, b: 0, c: 0, d: 0 };
    }
    return { a: q1.a / norm, b: q1.b / norm, c: q1.c / norm, d: q1.d / norm };
}
// #endregion quaternion-normalize

// #region quaternion-multiply
function quaternion_multiply(q1, q2) {
    // returns the Hamilton product q1*q2 (NOT commutative -- q1*q2 != q2*q1
    // in general, exactly like rotation composition it represents):
    //   (a1+b1 i+c1 j+d1 k)(a2+b2 i+c2 j+d2 k), using i^2=j^2=k^2=ijk=-1
    return {
        a: q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d,
        b: q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c,
        c: q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b,
        d: q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a
    };
}
// #endregion quaternion-multiply

// #region quaternion-to-rotation-matrix
function quaternion_to_rotation_matrix(q) {
    // returns the 4x4 homogeneous rotation matrix equivalent to unit
    // quaternion q (caller is responsible for normalizing first, same as
    // every other generate_rotation_matrix_* in matrix.js)
    var a = q.a, b = q.b, c = q.c, d = q.d;

    return [
        [1 - 2 * (c * c + d * d), 2 * (b * c - a * d), 2 * (b * d + a * c), 0],
        [2 * (b * c + a * d), 1 - 2 * (b * b + d * d), 2 * (c * d - a * b), 0],
        [2 * (b * d - a * c), 2 * (c * d + a * b), 1 - 2 * (b * b + c * c), 0],
        [0, 0, 0, 1]
    ];
}
// #endregion quaternion-to-rotation-matrix
