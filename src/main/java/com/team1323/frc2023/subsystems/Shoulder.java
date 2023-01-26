package com.team1323.frc2023.subsystems;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoulder extends ServoSubsystemWithAbsoluteEncoder {
    private static Shoulder instance = null;
    public static Shoulder getInstance() {
        if (instance == null) {
            instance = new Shoulder();
        }
        return instance;
    }
    
    public Shoulder() {
        super(Ports.SHOULDER, null, Constants.Shoulder.kEncoderUnitsPerDegree, 
                Constants.Shoulder.kMinControlAngle, Constants.Shoulder.kMaxControlAngle, 
                Constants.Shoulder.kAngleTolerance, Constants.Shoulder.kVelocityScalar, 
                Constants.Shoulder.kAccelerationScalar, Constants.Shoulder.kAbsoluteEncoderInfo);

        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(2.0));
        setPIDF(Constants.Shoulder.kPIDF);
        setSupplyCurrentLimit(Constants.Shoulder.kSupplyCurrentLimit);
        zeroPosition();
        stop();
    }

    /**
     * Assumes that the shoulder is at 0 degrees when completely horizontal (i.e., when gravity exerts the most
     * torque on the motor), and that rotating the shoulder upward results in a positive angle.
     */
    private void updateArbitraryFeedForward() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(getPosition());
        periodicIO.arbitraryFeedForward = Constants.Shoulder.kArbitraryFeedForward * currentAngle.cos();
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
        SmartDashboard.putNumber("Shoulder Angle", getPosition());
        SmartDashboard.putNumber("Shoulder Encoder Position", periodicIO.position);
        SmartDashboard.putNumber("Shoulder Absolute Encoder", getAbsoluteEncoderDegrees());
    }
}
