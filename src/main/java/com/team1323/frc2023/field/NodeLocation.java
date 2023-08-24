package com.team1323.frc2023.field;

public class NodeLocation {
    public final Grid grid;
    public final Row row;
    public final Column column;

    public NodeLocation(Grid grid, Row row, Column column) {
        this.grid = grid;
        this.row = row;
        this.column = column;
    }

    public boolean isEdgeColumn() {
        return (grid == Grid.RIGHT && column == Column.RIGHT) ||
                (grid == Grid.LEFT && column == Column.LEFT);
    }

    public NodeLocation mirrorAboutX() {
        return new NodeLocation(grid.mirrorAboutX(), row, column.mirrorAboutX());
    }

    public NodeLocation mirrorAboutY() {
        return new NodeLocation(grid.mirrorAboutY(), row, column.mirrorAboutY());
    }

    /**
     * Positions are from the robot's perspective when scoring.
     */
    public enum Grid {
        LEFT, CENTER, RIGHT;

        public Grid mirrorAboutX() {
            if (this == Grid.LEFT) {
                return Grid.RIGHT;
            } else if (this == Grid.RIGHT) {
                return Grid.LEFT;
            }

            return this;
        }

        public Grid mirrorAboutY() {
            return mirrorAboutX();
        }
    }

    public enum Row {
        BOTTOM, MIDDLE, TOP;
    }

    public enum Column {
        LEFT, CENTER, RIGHT;

        public Column mirrorAboutX() {
            if (this == Column.LEFT) {
                return Column.RIGHT;
            } else if (this == Column.RIGHT) {
                return Column.LEFT;
            }

            return this;
        }

        public Column mirrorAboutY() {
            return mirrorAboutX();
        }
    }
}
