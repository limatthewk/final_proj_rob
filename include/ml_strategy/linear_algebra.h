/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <vector>
#include <string>
#include "ml_strategy/helper.h"

// Line parameterized as l = p0 + t.vec, t belongs to (-inf,inf)
// This class is used in point compression algorithms
class Line3d{
 public:
    Eigen::Vector3d p0_;
    Eigen::Vector3d vec_;

    // Constructor: returns line that goes through two points p1, p2
    Line3d(const Eigen::Vector3d &p1,
           const Eigen::Vector3d &p2);

    // Methods
    // Calculate the distance between one point and the line
    void DistancePoint2Line(const Eigen::Vector3d &point,
                            double *dist);
};

// Plane parameterized as an origin and a normal vector
class Plane3d{
 public:
    Eigen::Vector3d origin_;
    Eigen::Vector3d normal_;

    // Empty constructor
    Plane3d() {};
    // Constructor: plane given by an origin and a normal
    Plane3d(const Eigen::Vector3d &origin,
            const Eigen::Vector3d &normal);
    // Plane given by three points.
    // Normal is in the direction of (p2-p1)x(p3-p1)
    Plane3d(const Eigen::Vector3d &p1,
            const Eigen::Vector3d &p2,
            const Eigen::Vector3d &p3);

    // Perform a homogeneous transformation on a plane
    void TransformPlane(const Eigen::Affine3d &transform,
                        Plane3d *transformedPlane);

    // This function returns a positive value if point is in the
    // direction of the normal, and a negative value if it is
    // in the opposite direction of the normal
    void DistancePoint2Plane(const Eigen::Vector3d &point,
                             double *dist);

    // This function returns a positive distance if point is in the
    // direction of the normal, and a negative value if it is
    // in the opposite direction of the normal
    void ProjectPointOntoPlane(const Eigen::Vector3d &point,
                               Eigen::Vector3d *nearest_point,
                               double *dist);

    // Projects vector onto plane (vector starts at origin)
    void ProjectVectorOntoPlane(const Eigen::Vector3d &vector,
                               Eigen::Vector3d *projection);
};

#endif  // LINEAR_ALGEBRA_H_
