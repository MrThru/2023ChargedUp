package com.team1323.frc2023.field;

import java.util.Map;

import com.team1323.frc2023.field.NodeLocation.Column;
import com.team1323.frc2023.field.NodeLocation.Grid;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class FieldOffsets {
    protected Map<Grid, Map<Column, Translation2d>> getRedOffsets() {
        return Map.of(
            Grid.LEFT, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            ),
            Grid.CENTER, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            ),
            Grid.RIGHT, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            )
        );
    } 

    protected Map<Grid, Map<Column, Translation2d>> getBlueOffsets() {
        return Map.of(
            Grid.LEFT, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            ),
            Grid.CENTER, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            ),
            Grid.RIGHT, Map.of(
                Column.LEFT, Translation2d.identity(),
                Column.CENTER, Translation2d.identity(),
                Column.RIGHT, Translation2d.identity()
            )
        );
    } 

    public Pose2d applyOffsets(Pose2d scoringPose, NodeLocation nodeLocation) {
        Map<Grid, Map<Column, Translation2d>> offsetMap = AllianceChooser.getAlliance() == Alliance.Blue ?
                getBlueOffsets() : getRedOffsets();
        Translation2d offset = offsetMap.get(nodeLocation.grid).get(nodeLocation.column);

        return scoringPose.transformBy(Pose2d.fromTranslation(offset));
    }
}