/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.lib.math.geometry;

/**
 * Add your docs here.
 */
public class Plane {
    private Vector3d orthogonal_vector_;
    private Vector3d point_;

    public Plane(Vector3d orthogonal_vector, Vector3d point) {
        orthogonal_vector_ = orthogonal_vector;
        point_ = point;
    }

    public Plane(Vector3d p0, Vector3d p1, Vector3d p2) {
        orthogonal_vector_ = p0.subtract(p1).crossProduct(p2.subtract(p1));
        point_ = p1;
    }

    public Vector3d orthogonalVector() {
        return orthogonal_vector_;
    }
}
