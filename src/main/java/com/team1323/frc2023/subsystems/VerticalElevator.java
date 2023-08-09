package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.requests.Prerequisite;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroing;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithCurrentZeroingInputs;
import com.team1323.lib.drivers.Phoenix5FXMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerticalElevator extends ServoSubsystemWithCurrentZeroing<ServoSubsystemWithCurrentZeroingInputs> {
    private static VerticalElevator instance = null;
    public static VerticalElevator getInstance() {
        if (instance == null) {
            instance = new VerticalElevator();
        }
        return instance;
    }

    private VerticalElevator() {
        super(Phoenix5FXMotorController.createRealOrSimulatedController(Constants.VerticalElevator.kConfig.leaderPortNumber, Constants.VerticalElevator.kConfig.canBus), 
                new ArrayList<>(), Constants.VerticalElevator.kConfig, Constants.VerticalElevator.kCurrentZeroingConfig, new ServoSubsystemWithCurrentZeroingInputs());
        leader.useIntegratedSensor();
        leader.setInverted(TalonFXInvertType.Clockwise);
        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(0.5));
        leader.setPIDF(Constants.VerticalElevator.kPIDF);
        setSupplyCurrentLimit(Constants.VerticalElevator.kSupplyCurrentLimit);
        leader.setSelectedSensorPosition(0);
        isZeroed = true;
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

            @Override
            public String toString() {
                return String.format("VerticalElevatorRequest(target = %.2f)", inches);
            }
        };
    }

    public Prerequisite heightPrerequisite(double inches) {
        return () -> isAtPosition(inches);
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            outputs.arbitraryFeedForward = 0;
        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {
            outputs.arbitraryFeedForward = 0;
        }
        
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
        super.registerEnabledLoops(enabledLooper);
    }

    @Override
    public void setPosition(double outputUnits) {
        outputs.arbitraryFeedForward = Constants.VerticalElevator.kArbitraryFeedForward;
        super.setPosition(outputUnits);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vertical Elevator Height", getPosition());
        SmartDashboard.putNumber("Vertical Elevator Target Height", encoderUnitsToOutputUnits(outputs.demand));
        SmartDashboard.putBoolean("Vertical Elevator Is On Target", isOnTarget());
    }
}
