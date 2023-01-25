package com.team1323.frc2023.field;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NodeLocation {
    public final Grid grid;
    public final Row row;
    public final Column column;

    public NodeLocation(Grid grid, Row row, Column column) {
        this.grid = grid;
        this.row = row;
        this.column = column;
    }

    private static final Grid[] gridArray = new Grid[]{Grid.LEFT, Grid.CENTER, Grid.RIGHT};
    private static final Row[] rowArray = new Row[]{Row.BOTTOM, Row.MIDDLE, Row.TOP};
    private static final Column[] columnArray = new Column[]{Column.LEFT, Column.CENTER, Column.RIGHT};
    private static final double[] defaultLocationArray = new double[]{0, 0, 0};
    private static final NodeLocation defaultNodeLocation = new NodeLocation(Grid.LEFT, Row.BOTTOM, Column.LEFT);

    public static NodeLocation getDashboardLocation() {
        final int locationArrayLength = 3;
        double[] locationDoubleArray = SmartDashboard.getNumberArray("Selected Node Location", defaultLocationArray);

        if (locationDoubleArray.length != locationArrayLength) {
            System.out.println(String.format("Invalid node location array! Expected length: %d. Actual length: %d.", locationArrayLength, locationDoubleArray.length));
            return defaultNodeLocation;
        }

        int[] locationIntArray = new int[locationArrayLength];
        for (int i = 0; i < locationArrayLength; i++) {
            locationIntArray[i] = (int) locationDoubleArray[i];
        }

        for (int index : locationIntArray) {
            if (index < 0 || index > 2) {
                System.out.println(String.format("Invalid node location indices: %s", locationIntArray.toString()));
                return defaultNodeLocation;
            }
        }

        return new NodeLocation(gridArray[locationIntArray[0]], rowArray[locationIntArray[1]], columnArray[locationIntArray[2]]);
    }

    /**
     * Positions are from the robot's perspective when scoring.
     */
    public enum Grid {
        LEFT, CENTER, RIGHT;
    }

    public enum Row {
        BOTTOM, MIDDLE, TOP;
    }

    public enum Column {
        LEFT, CENTER, RIGHT;
    }
}
