/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Forward Kinematics and Quaternions | Matrix Algebra and Geometric Transforms

    COMPLETED REFERENCE IMPLEMENTATION for the AutoRob (autorob.org) lab-session
    slides. Completes every "STENCIL" section of the upstream kineval-stencil
    (github.com/autorob/kineval-stencil) kineval/kineval_matrix.js module:
    matrix_multiply, matrix_transpose, matrix_pseudoinverse,
    matrix_invert_affine, vector_normalize, vector_cross, generate_identity,
    generate_translation_matrix, generate_rotation_matrix_{X,Y,Z}.

    Every matrix here is a plain 2D array (array of row arrays), and every
    transform is a 4x4 homogeneous matrix operating on 4x1 homogeneous column
    vectors [x,y,z,1]^T -- composed by LEFT-multiplication as the matrix stack
    is walked outward-in from the robot base (parent transform times child
    transform), matching the matrix-stack lecture's convention and this
    module's forward_kinematics.js traversal.

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

// #region matrix-copy
function matrix_copy(m1) {
    // returns 2D array that is a copy of m1
    var mat = [];
    var i, j;
    for (i = 0; i < m1.length; i++) {
        mat[i] = [];
        for (j = 0; j < m1[0].length; j++) {
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}
// #endregion matrix-copy

// #region matrix-multiply
function matrix_multiply(m1, m2) {
    // returns 2D array that is the result of m1*m2
    var result = [];
    var i, j, k, sum;
    for (i = 0; i < m1.length; i++) {
        result[i] = [];
        for (j = 0; j < m2[0].length; j++) {
            sum = 0;
            for (k = 0; k < m2.length; k++) {
                sum += m1[i][k] * m2[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}
// #endregion matrix-multiply

// #region matrix-transpose
function matrix_transpose(m) {
    // returns 2D array that is the transpose of m
    var result = [];
    var i, j;
    for (j = 0; j < m[0].length; j++) {
        result[j] = [];
        for (i = 0; i < m.length; i++) {
            result[j][i] = m[i][j];
        }
    }
    return result;
}
// #endregion matrix-transpose

// #region matrix-square-invert
function matrix_square_invert(m) {
    // returns inverse of a square (NxN) matrix via Gauss-Jordan elimination
    // with partial pivoting. Not itself one of the stencil's listed
    // functions, but the smallest self-contained building block
    // matrix_pseudoinverse needs -- kept here rather than reaching for an
    // external linear-algebra dependency (the upstream inverse_kinematics
    // stencil instead vendors the numeric.js library for this; that module
    // is out of scope here, so this stays dependency-free).
    var n = m.length;
    var i, j, k, pivotRow, maxAbs, tmp, factor;

    // augmented [m | I]
    var aug = [];
    for (i = 0; i < n; i++) {
        aug[i] = m[i].slice();
        for (j = 0; j < n; j++) {
            aug[i].push(i === j ? 1 : 0);
        }
    }

    for (k = 0; k < n; k++) {
        pivotRow = k;
        maxAbs = Math.abs(aug[k][k]);
        for (i = k + 1; i < n; i++) {
            if (Math.abs(aug[i][k]) > maxAbs) {
                maxAbs = Math.abs(aug[i][k]);
                pivotRow = i;
            }
        }
        if (pivotRow !== k) {
            tmp = aug[k];
            aug[k] = aug[pivotRow];
            aug[pivotRow] = tmp;
        }

        factor = aug[k][k];
        for (j = 0; j < 2 * n; j++) {
            aug[k][j] /= factor;
        }

        for (i = 0; i < n; i++) {
            if (i === k) continue;
            factor = aug[i][k];
            for (j = 0; j < 2 * n; j++) {
                aug[i][j] -= factor * aug[k][j];
            }
        }
    }

    var inv = [];
    for (i = 0; i < n; i++) {
        inv[i] = aug[i].slice(n, 2 * n);
    }
    return inv;
}
// #endregion matrix-square-invert

// #region matrix-pseudoinverse
function matrix_pseudoinverse(m) {
    // returns the Moore-Penrose pseudoinverse of (possibly non-square) m,
    // via the normal equations: for an overdetermined/tall m (more rows
    // than columns, the common case for a robot Jacobian stacked by DOF),
    //     pinv(m) = inv(m^T m) m^T
    // for a wide m (more columns than rows, e.g. an end-effector Jacobian
    // with more joints than task DOFs -- the case inverse_kinematics/ will
    // actually call this for), the left form is singular, so instead
    //     pinv(m) = m^T inv(m m^T)
    // Not used by forward kinematics itself -- included here only because
    // it's one of the matrix.js STENCIL functions, for the inverse_kinematics
    // module to reuse directly.
    var rows = m.length, cols = m[0].length;
    var mT = matrix_transpose(m);

    if (rows >= cols) {
        return matrix_multiply(matrix_square_invert(matrix_multiply(mT, m)), mT);
    } else {
        return matrix_multiply(mT, matrix_square_invert(matrix_multiply(m, mT)));
    }
}
// #endregion matrix-pseudoinverse

// #region matrix-invert-affine
function matrix_invert_affine(m) {
    // returns the inverse of a 4x4 affine (rotation + translation)
    // homogeneous transform m, exploiting the structure of such a matrix
    // rather than a generic (and more expensive) full inverse:
    //   m = [ R  t ]       m^-1 = [ R^T  -R^T t ]
    //       [ 0  1 ]              [ 0     1    ]
    // because the rotation block R of any rigid transform is orthonormal
    // (R^-1 = R^T).
    var i, j;
    var rotT = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            rotT[i][j] = m[j][i];
        }
    }

    var t = [m[0][3], m[1][3], m[2][3]];
    var tInv = [0, 0, 0];
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            tInv[i] -= rotT[i][j] * t[j];
        }
    }

    return [
        [rotT[0][0], rotT[0][1], rotT[0][2], tInv[0]],
        [rotT[1][0], rotT[1][1], rotT[1][2], tInv[1]],
        [rotT[2][0], rotT[2][1], rotT[2][2], tInv[2]],
        [0, 0, 0, 1]
    ];
}
// #endregion matrix-invert-affine

// #region vector-normalize
function vector_normalize(v) {
    // returns normalized vector for v (array of any length)
    var i, sumsq = 0;
    for (i = 0; i < v.length; i++) {
        sumsq += v[i] * v[i];
    }
    var len = Math.sqrt(sumsq);
    if (len === 0) return v.slice();
    var result = [];
    for (i = 0; i < v.length; i++) {
        result[i] = v[i] / len;
    }
    return result;
}
// #endregion vector-normalize

// #region vector-cross
function vector_cross(a, b) {
    // returns the cross product of 3-dimensional vectors a and b
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}
// #endregion vector-cross

// #region generate-identity
function generate_identity() {
    // returns a 4x4 identity matrix as a 2D array
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}
// #endregion generate-identity

// #region generate-translation-matrix
function generate_translation_matrix(tx, ty, tz) {
    // returns a 4x4 homogeneous translation matrix as a 2D array
    return [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ];
}
// #endregion generate-translation-matrix

// #region generate-rotation-matrix-x
function generate_rotation_matrix_X(angle) {
    // returns a 4x4 homogeneous rotation matrix about the X axis, angle in radians
    var c = Math.cos(angle), s = Math.sin(angle);
    return [
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ];
}
// #endregion generate-rotation-matrix-x

// #region generate-rotation-matrix-y
function generate_rotation_matrix_Y(angle) {
    // returns a 4x4 homogeneous rotation matrix about the Y axis, angle in radians
    var c = Math.cos(angle), s = Math.sin(angle);
    return [
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ];
}
// #endregion generate-rotation-matrix-y

// #region generate-rotation-matrix-z
function generate_rotation_matrix_Z(angle) {
    // returns a 4x4 homogeneous rotation matrix about the Z axis, angle in radians
    var c = Math.cos(angle), s = Math.sin(angle);
    return [
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}
// #endregion generate-rotation-matrix-z
