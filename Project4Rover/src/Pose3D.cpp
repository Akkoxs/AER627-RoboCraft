#include "Pose3D.h"
//#include <algorithm>

// Default Constructor
// Pose3D::Pose3D() {
//     setIdentity();
// }

// Pose3D::Pose3D(const double R[3][3], const double t[3]){
//     setRotation(R);
//     setTranslation(t);
// }

// Set translation using individual coordinates
void Pose3D::setTranslation(double x, double y, double z) {
    t[0] = x; t[1] = y; t[2] = z;
}

// Set translation using an array
void Pose3D::setTranslation(const double new_t[3]) {
    //std::copy(new_t, new_t + 3, t);
    for (int i = 0; i < 3; ++i) {
        t[i] = new_t[i];
    }
}

// Set rotation using a 3x3 array
void Pose3D::setRotation(const double new_R[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = new_R[i][j];
        }
    }
}

// Resets to Identity
void Pose3D::setIdentity() {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = (i == j) ? 1.0 : 0.0;
        }
        t[i] = 0.0;
    }
}

// Composition: T_out = T1 * T2
Pose3D Pose3D::operator*(const Pose3D& other) const {
    Pose3D result;
    for (int i = 0; i < 3; ++i) {
        // New translation: t_new = R1 * t2 + t1
        result.t[i] = t[i];
        for (int j = 0; j < 3; ++j) {
            result.t[i] += R[i][j] * other.t[j];
        }

        // New rotation: R_new = R1 * R2
        for (int j = 0; j < 3; ++j) {
            result.R[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                result.R[i][j] += R[i][k] * other.R[k][j];
            }
        }
    }
    return result;
}

// Efficient Inversion
Pose3D Pose3D::inverse() const {
    Pose3D inv;
    // R_inv = R^Transpose
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            inv.R[i][j] = R[j][i];
        }
    }
    // t_inv = -R_inv * t
    for (int i = 0; i < 3; ++i) {
        inv.t[i] = 0;
        for (int j = 0; j < 3; ++j) {
            inv.t[i] -= inv.R[i][j] * t[j];
        }
    }
    return inv;
}

void Pose3D::transformPoint(const double point_in[3], double point_out[3]) const {
    for (int i = 0; i < 3; ++i) {
        // Initialize with the translation component (t)
        point_out[i] = t[i];
        
        // Add the rotated components: (R * v)
        for (int j = 0; j < 3; ++j) {
            point_out[i] += R[i][j] * point_in[j];
        }
    }
}