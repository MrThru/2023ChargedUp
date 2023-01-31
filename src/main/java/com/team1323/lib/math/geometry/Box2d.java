// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.math.geometry;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Translation2d;

/** Add your docs here. */
public class Box2d {

    private Translation2d position1; //Top Right Point
    private Translation2d position2; //Top Left Point
    private Translation2d position3; //Bottom Left Point
    private Translation2d position4; //Bottom Right Point

    List<Translation2d> positions;
    public Box2d() {
        this(Translation2d.identity(), Translation2d.identity(), Translation2d.identity(), Translation2d.identity());
    }
    public Box2d(Translation2d position1, Translation2d position2, 
                    Translation2d position3, Translation2d position4) {
        this.position1 = position1;
        this.position2 = position2;
        this.position3 = position4;
        this.position4 = position4;

        positions = Arrays.asList(this.position1, this.position2, this.position3, this.position4);
    }


    public Box2d moveOrigin(Translation2d shift) {
        return new Box2d(this.position1.translateBy(shift), this.position2.translateBy(shift),
                    this.position3.translateBy(shift), this.position4.translateBy(shift));
    }


    public boolean pointWithinBox(Translation2d point) {
        if(!(this.position2.x() < point.x() && point.x() < this.position1.x() &&
                this.position3.x() < point.x() && point.x() < this.position4.x())) //Within the X perimeter
            return false;
        if(!(this.position3.y() < point.y() && point.y() < this.position2.y() &&
                this.position4.y() < point.y() && point.y() < this.position1.y()))
            return false;
        return true;
    }

    public static Box2d fromSizing(double width, double height) {
        return new Box2d(new Translation2d(width / 2, height / 2), new Translation2d(-width / 2, height / 2), 
                new Translation2d(-width / 2, -height / 2), new Translation2d(width / 2, -height / 2));
    }



}
