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

    /**
     * Positions are from the robot's perspective when scoring.
     */
    public enum Grid {
        LEFT(0), CENTER(1), RIGHT(2);

        public final int index;

        private Grid(int index) {
            this.index = index;
        }
    }

    public enum Row {
        BOTTOM(0), MIDDLE(1), TOP(2);

        public final int index;

        private Row(int index) {
            this.index = index;
        }
    }

    public enum Column {
        LEFT(0), CENTER(1), RIGHT(2);

        public final int index;

        private Column(int index) {
            this.index = index;
        }
    }
}
