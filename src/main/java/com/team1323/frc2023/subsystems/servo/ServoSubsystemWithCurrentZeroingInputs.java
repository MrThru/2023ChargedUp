package com.team1323.frc2023.subsystems.servo;

import org.littletonrobotics.junction.LogTable;

public class ServoSubsystemWithCurrentZeroingInputs extends ServoSubsystemInputsAutoLogged {
    public double supplyCurrent;

    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("SupplyCurrent", supplyCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        super.fromLog(table);
        supplyCurrent = table.getDouble("SupplyCurrent", supplyCurrent);
    }

    @Override
    public ServoSubsystemWithCurrentZeroingInputs clone() {
        ServoSubsystemWithCurrentZeroingInputs copy = new ServoSubsystemWithCurrentZeroingInputs();
        copy.position = this.position;
        copy.velocity = this.velocity;
        copy.supplyCurrent = this.supplyCurrent;
        return copy;
    }
}
