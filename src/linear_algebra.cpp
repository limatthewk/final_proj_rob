#include "ml_strategy/linear_algebra.h"

// Line3d class ----------------------------------------------------

// Constructor: returns line that goes through two points p1, p2
Line3d::Line3d(const Eigen::Vector3d &p1,
       const Eigen::Vector3d &p2) {
    p0_ = p1;
    vec_ = p1 - p2;
}

// Methods
// Calculate the distance between one point and the line
void Line3d::DistancePoint2Line(const Eigen::Vector3d &point,
                                double *dist) {
    // Two points on the line
    const Eigen::Vector3d x1 = p0_;
    const Eigen::Vector3d x2 = p0_ + vec_;

    // Optimal t is the t at which the point is closest to the line
    // t_opt = (x1-x2)'*(x1-p)/norm(x1-x2)^2;
    static double t_opt;
    if (x1 == x2) {
        t_opt = 0;
    } else {
        const double gain = 1.0/(pow((x1-x2).norm(), 2.0));
        t_opt = gain*(x1-x2).transpose()*(x1-point);
    }

    // p_opt is the closest point between the point and the line
    const Eigen::Vector3d p_opt = x1 + t_opt*vec_;

    *dist = (point - p_opt).norm();
}



// Plane3d class ----------------------------------------------------
// Constructor: plane given by an origin and a normal
Plane3d::Plane3d(const Eigen::Vector3d &origin,
        const Eigen::Vector3d &normal) {
    origin_ = origin;
    if (normal.norm() == 0) {
        std::cout << "Warning: plane ill-defined: "
                  << "zero-norm normal vector!"
                  << std::endl;
        normal_ << 0.0, 0.0, 0.0;
    } else {
        normal_ = normal.normalized();
    }
}

// Plane given by three points.
// Normal is in the direction of (p2-p1)x(p3-p1)
Plane3d::Plane3d(const Eigen::Vector3d &p1,
        const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3) {
    const Eigen::Vector3d v1 = p2 - p1;
    const Eigen::Vector3d v2 = p3 - p1;
    // const Eigen::Vector3d v3 = (p1 + p2 + p3)/3.0;
    if ((v1.norm() == 0) || (v2.norm() == 0) || ((p2-p3).norm() == 0)) {
        std::cout << "Warning: plane ill-defined: "
                  << "three distinct points are needed!"
                  << std::endl;
        origin_ = p1;
        normal_ << 0.0, 0.0, 0.0;
    } else {
        origin_ = p1;
        normal_ = v1.cross(v2).normalized();
    }
}

// Perform a homogeneous transformation on a plane
void Plane3d::TransformPlane(const Eigen::Affine3d &transform,
                             Plane3d *transformedPlane) {
    transformedPlane->origin_ = transform*origin_;
    transformedPlane->normal_ = (transform.linear()*normal_).normalized();
}

// This function returns a positive value if point is in the
// direction of the normal, and a negative value if it is
// in the opposite direction of the normal
void Plane3d::DistancePoint2Plane(const Eigen::Vector3d &point,
                         double *dist) {
    // Get distance between point and origin
    const Eigen::Vector3d dist_point_origin = point - origin_;

    // Project the calculated distance into the normal vector
    *dist =  dist_point_origin.dot(normal_);
}

// This function returns a positive distance if point is in the
// direction of the normal, and a negative value if it is
// in the opposite direction of the normal
void Plane3d::ProjectPointOntoPlane(const Eigen::Vector3d &point,
                           Eigen::Vector3d *nearest_point,
                           double *dist) {
    this->DistancePoint2Plane(point, dist);
    *nearest_point = point - (*dist)*normal_;
}

// Projects vector onto plane (vector starts at origin)
void Plane3d::ProjectVectorOntoPlane(const Eigen::Vector3d &vector,
                           Eigen::Vector3d *projection) {
    const double dist =  vector.dot(normal_);
    *projection = vector - dist*normal_;
}