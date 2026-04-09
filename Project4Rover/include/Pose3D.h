#ifndef POSE3D_H
#define POSE3D_H

class Pose3D {
public:
    double R[3][3];
    double t[3];

    // Constructors
    // Removing these constructors allows us to treat this as an aggregate clase
    // which can be initialized with designated initializers.
    // Pose3D();
    // Pose3D(const double R[3][3], const double t[3]);

    // Setters
    void setTranslation(double x, double y, double z);
    void setTranslation(const double new_t[3]);
    void setRotation(const double new_R[3][3]);
    void setIdentity();

    // Core Operations
    Pose3D operator*(const Pose3D& other) const;
    Pose3D inverse() const;
    void transformPoint(const double point_in[3], double point_out[3]) const;
};

#endif // POSE3D_H