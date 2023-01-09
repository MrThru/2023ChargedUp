package com.team1323.frc2023.subsystems;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Request;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends ServoSubsystemWithAbsoluteEncoder {
    private static Wrist instance = null;
    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    public Wrist() {
        super(Ports.WRIST, null, Constants.Wrist.kEncoderUnitsPerDegree, 
                Constants.Wrist.kMinControlAngle, Constants.Wrist.kMaxControlAngle, 
                0.25, 1.0, Constants.Wrist.kAbsoluteEncoderInfo);

        setPIDF(0, Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD, Constants.Wrist.kF);
        setSupplyCurrentLimit(30.0);
        zeroPosition();
        stop();
    }

    public Request angleRequest(double degrees) {
        return new Request() {
            @Override
            public void act() {
                setPosition(degrees);
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Wrist Angle", getPosition());
        SmartDashboard.putNumber("Wrist Absolute Encoder", getAbsoluteEncoderDegrees());
    }
}
