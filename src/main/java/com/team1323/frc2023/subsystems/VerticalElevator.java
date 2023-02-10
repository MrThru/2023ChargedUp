package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystem;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerticalElevator extends ServoSubsystemWithCurrentZeroing {
    private static VerticalElevator instance = null;
    public static VerticalElevator getInstance() {
        if(instance == null)
            instance = new VerticalElevator();
        return instance;
    }

    public VerticalElevator() {
        super(Ports.VERTICAL_ELEVATOR_LEADER, Ports.CANBUS,
                Constants.VerticalElevator.kTicksPerInch, Constants.VerticalElevator.kMinControlHeight, 
                Constants.VerticalElevator.kMaxControlHeight, Constants.VerticalElevator.kHeightTolerance, 
                Constants.VerticalElevator.kVelocityScalar, Constants.VerticalElevator.kAccelerationScalar,
                Constants.VerticalElevator.kCurrentZeroingConfig);

        leader.setInverted(TalonFXInvertType.Clockwise);
        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(0.5));
        leader.setPIDF(Constants.VerticalElevator.kPIDF);
        setSupplyCurrentLimit(Constants.VerticalElevator.kSupplyCurrentLimit);
        periodicIO.arbitraryFeedForward = Constants.VerticalElevator.kArbitraryFeedForward;
        zeroPosition();
        stop();
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

    public Prerequisite heightPrerequisite(double inches) {
        return () -> isAtPosition(inches);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vertical Elevator Height", getPosition());
        SmartDashboard.putNumber("Vertical Elevator Encoder Position", periodicIO.position);
    }
}
