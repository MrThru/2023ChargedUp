package com.team1323.frc2023.subsystems.servo;

import org.littletonrobotics.junction.LogTable;

public class ServoSubsystemWithAbsoluteEncoderInputs extends ServoSubsystemWithCurrentZeroingInputs {
    public double absoluteEncoderDegrees;

    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("AbsoluteEncoderDegrees", absoluteEncoderDegrees);
    }

    @Override
    public void fromLog(LogTable table) {
        super.fromLog(table);
        absoluteEncoderDegrees = table.getDouble("AbsoluteEncoderDegrees", absoluteEncoderDegrees);
    }

    @Override
    public ServoSubsystemWithAbsoluteEncoderInputs clone() {
        ServoSubsystemWithAbsoluteEncoderInputs copy = new ServoSubsystemWithAbsoluteEncoderInputs();
        copy.position = this.position;
        copy.velocity = this.velocity;
        copy.supplyCurrent = this.supplyCurrent;
        copy.absoluteEncoderDegrees = this.absoluteEncoderDegrees;
        return copy;
    }
}
