package com.team1323.frc2023.subsystems;

import java.util.Arrays;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Request;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerticalElevator extends ServoSubsystem {
    private static VerticalElevator instance = null;
    public static VerticalElevator getInstance() {
        if(instance == null)
            instance = new VerticalElevator();
        return instance;
    }

    public VerticalElevator() {
        super(Ports.VERTICAL_ELEVATOR_LEADER, Arrays.asList(Ports.VERTICAL_ELEVATOR_FOLLOWER), 
                null, Constants.VerticalElevator.kTicksPerInch, Constants.VerticalElevator.kMinControlHeight, 
                Constants.VerticalElevator.kMaxControlHeight, Constants.VerticalElevator.kHeightTolerance, 
                Constants.VerticalElevator.kVelocityScalar, Constants.VerticalElevator.kAccelerationScalar);

        setPIDF(0, Constants.VerticalElevator.kP, Constants.VerticalElevator.kI, Constants.VerticalElevator.kD, Constants.VerticalElevator.kF);
        setSupplyCurrentLimit(Constants.VerticalElevator.kSupplyCurrentLimit);
        periodicIO.arbitraryFeedForward = Constants.VerticalElevator.kArbitraryFeedForward;
        zeroPosition();
        stop();
        leader.enableLogging(true);
    }

    public Request heightRequest(double inches) {
        return new Request() {
            @Override
            public void act() {
                setPosition(inches);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vertical Elevator Height", getPosition());
        SmartDashboard.putNumber("Vertical Elevator Encoder Position", periodicIO.position);
    }
}
