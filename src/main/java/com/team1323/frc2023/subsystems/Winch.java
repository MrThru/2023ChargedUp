package com.team1323.frc2023.subsystems;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Winch extends ServoSubsystem {
    private static Winch instance = null;
    public static Winch getInstance() {
        if (instance == null) {
            instance = new Winch();
        }
        return instance;
    }
    
    public Winch() {
        super(Constants.Winch.kConfig);
        leader.setPIDF(Constants.Winch.kPIDF);
        setSupplyCurrentLimit(Constants.Winch.kSupplyCurrentLimit);
        zeroPosition();
        stop();
    }

    public Request angleRequest(double degrees) {
        return new Request() {
            @Override
            public void act() {
                setPosition(degrees);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Winch Angle", getPosition());
        SmartDashboard.putNumber("Winch Encoder Position", periodicIO.position);
    }
}
