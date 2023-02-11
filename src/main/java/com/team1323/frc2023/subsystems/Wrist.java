package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.encoders.CanEncoder;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;
import com.team254.lib.geometry.Rotation2d;

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
        super(Ports.WRIST, Ports.CANBUS, Constants.Wrist.kEncoderUnitsPerDegree, 
                Constants.Wrist.kMinControlAngle, Constants.Wrist.kMaxControlAngle, 
                Constants.Wrist.kAngleTolerance, Constants.Wrist.kVelocityScalar, 
                Constants.Wrist.kAccelerationScalar, new CanEncoder(Ports.WRIST_ENCODER, false), Constants.Wrist.kAbsoluteEncoderInfo);

        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(4.0));
        leader.setPIDF(Constants.Wrist.kPIDF);
        leader.setInverted(TalonFXInvertType.CounterClockwise);
        setSupplyCurrentLimit(Constants.Wrist.kSupplyCurrentLimit);
        zeroPosition();
        stop();
    }

    /**
     * Assumes that the wrist is at 0 degrees when completely horizontal (i.e., when gravity exerts the most
     * torque on the motor), and that rotating the wrist upward results in a positive angle.
     */
    private void updateArbitraryFeedForward() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(getPosition());
        periodicIO.arbitraryFeedForward = Constants.Wrist.kArbitraryFeedForward * currentAngle.cos();
    }

    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onLoop(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onStop(double timestamp) {}
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
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
        SmartDashboard.putNumber("Wrist Angle", getPosition());
        SmartDashboard.putNumber("Wrist Encoder Position", periodicIO.position);
        SmartDashboard.putNumber("Wrist Absolute Encoder", absoluteEncoder.getDegrees());
        SmartDashboard.putBoolean("Wrist On Target", isOnTarget());
        SmartDashboard.putNumber("Wrist Target Angle", encoderUnitsToOutputUnits(periodicIO.demand));
    }
}
