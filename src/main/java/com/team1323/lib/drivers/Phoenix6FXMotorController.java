package com.team1323.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Settings;

import edu.wpi.first.wpilibj.RobotBase;

public class Phoenix6FXMotorController extends TalonFX implements MotorController {
    private static final double kTalonFXEncoderResolution = 2048.0;
    private static final double kCANCoderResolution = 4096.0;

    private final TalonFXConfiguration configuration = new TalonFXConfiguration();

    private final VoltageOut voltageOutRequest = new VoltageOut(0.0, true, false);
    // TODO: Experiment with the new acceleration field in the VelocityVoltage object.
    private final VelocityTorqueCurrentFOC velocityCurrentRequest = new VelocityTorqueCurrentFOC(0.0, Constants.kMaxFalconRotationsPerSecond * 5.0, 0.0, 0, false);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0, Constants.kMaxFalconRotationsPerSecond * 5.0, true, 0.0, 0, false);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);
    private final Follower followerRequest = new Follower(0, false);

    private ControlRequest currentControlRequest = voltageOutRequest;

    public static MotorController createRealOrSimulatedController(int deviceId, boolean enableFOC) {
        return RobotBase.isReal() ? new Phoenix6FXMotorController(deviceId, enableFOC) : new SimulatedMotorController();
    }

    public static MotorController createRealOrSimulatedController(int deviceId, String canBus, boolean enableFOC) {
        return RobotBase.isReal() ? new Phoenix6FXMotorController(deviceId, canBus, enableFOC) : new SimulatedMotorController();
    }

    public static MotorController createRealOrSimulatedController(int deviceId, String canBus, int cancoderId, boolean enableFOC) {
        return RobotBase.isReal() ? new Phoenix6FXMotorController(deviceId, canBus, cancoderId, enableFOC) : new SimulatedMotorController();
    }

    private Phoenix6FXMotorController(int deviceId, boolean enableFOC) {
        this(deviceId, "rio", enableFOC);
    }

    private Phoenix6FXMotorController(int deviceId, String canBus, boolean enableFOC) {
        super(deviceId, canBus);
        enableFOC(enableFOC);
    }

    private Phoenix6FXMotorController(int deviceId, String canBus, int cancoderId, boolean enableFOC) {
        super(deviceId, canBus);
        useCANCoder(cancoderId);
        enableFOC(enableFOC);
    }

    private void enableFOC(boolean enable) {
        voltageOutRequest.EnableFOC = enable;
        velocityVoltageRequest.EnableFOC = enable;
        motionMagicRequest.EnableFOC = enable;
    }

    private void configureDefaultSettings() {
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        configuration.MotorOutput.PeakForwardDutyCycle = 1.0;
        configuration.MotorOutput.PeakReverseDutyCycle = -1.0;

        configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

        configuration.Voltage.PeakForwardVoltage = 16.0;
        configuration.Voltage.PeakReverseVoltage = -16.0;

        applyConfig();
    }

    @Override
    public void configureAsRoller() {
        configureDefaultSettings();
        useIntegratedSensor();
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        applyConfig();
    }

    @Override
    public void configureAsServo() {
        configureDefaultSettings();
    }

    @Override
    public void configureAsCoaxialSwerveRotation() {
        // TODO: Implement this
    }

    @Override
    public void configureAsCoaxialSwerveDrive() {
        useIntegratedSensor();

        configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.kIsUsingCompBot ? 0.0 : 0.2; // 0.2
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.CurrentLimits.StatorCurrentLimit = 100.0;
        configuration.CurrentLimits.StatorCurrentLimitEnable = Settings.kIsUsingCompBot;
        configuration.CurrentLimits.SupplyCurrentLimit = 60.0;
        configuration.CurrentLimits.SupplyCurrentThreshold = 120.0;
        configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = false;

        configuration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0; //75.0
        configuration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0; //75.0
        applyConfig();

        // Slot 0 is reserved for MotionMagic
        setPIDF(new MotorPIDF(0, 0.18, 0.0, 3.6, 1.0 / Constants.kMaxFalconRotationsPerSecond));
        // Slot 1 corresponds to velocity mode
        final double falconFeedForward = 12.0 / (Constants.kMaxFalconRotationsPerSecond * 0.95);
        //final double krakenFeedForward = 12.0 / (Constants.kMaxKrakenRotationsPerSecond * 0.92);
        final double krakenFeedForward = 0.0;
        if (Settings.kIsUsingCompBot) {
            setPIDF(new MotorPIDF(1, 12.5, 0.0, 0.0, krakenFeedForward));
        } else {
            setPIDF(new MotorPIDF(1, 0.11, 0.0, 0.0, falconFeedForward));
        }
        

        this.setPosition(0.0);
    }

    @Override
    public void configureAsDifferentialSwerveMotor() {
        configureDefaultSettings();

        useIntegratedSensor();

        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        applyConfig();

        // Slot 1 corresponds to velocity mode
        setPIDF(new MotorPIDF(1, 0.0, 0.0, 0.0, 0.0));
        selectProfileSlot(1);
    }

    private double getEncoderResolution() {
        return configuration.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder ?
                kCANCoderResolution : kTalonFXEncoderResolution;
    }

    private double encoderUnitsToEncoderRotations(double encoderUnits) {
        return encoderUnits / getEncoderResolution();
    }

    private double encoderRotationsToEncoderUnits(double rotations) {
        return rotations * getEncoderResolution();
    }

    private double encoderVelocityToRotationsPerSecond(double encUnitsPer100Ms) {
        return encUnitsPer100Ms * 10.0 / getEncoderResolution();
    }

    private double rotationsPerSecondToEncoderVelocity(double rps) {
        return rps * getEncoderResolution() / 10.0;
    }

    private ErrorCode convertToErrorCode(StatusCode statusCode) {
        if (statusCode != StatusCode.OK) {
            return ErrorCode.GENERAL_ERROR;
        }

        return ErrorCode.OK;
    }

    private ErrorCode applyConfig() {
        StatusCode statusCode = this.getConfigurator().apply(configuration, Constants.kCANTimeoutMs);
        return convertToErrorCode(statusCode);
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(double encoderUnits) {
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = encoderUnitsToEncoderRotations(encoderUnits);
        return applyConfig();
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double encoderUnits) {
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = encoderUnitsToEncoderRotations(encoderUnits);
        return applyConfig();
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable) {
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
        return applyConfig();
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable) {
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
        return applyConfig();
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(double encoderUnitsPer100Ms) {
        configuration.MotionMagic.MotionMagicCruiseVelocity = encoderVelocityToRotationsPerSecond(encoderUnitsPer100Ms);
        return applyConfig();
    }

    @Override
    public ErrorCode configMotionAcceleration(double encoderUnitsPer100MsPerSecond) {
        configuration.MotionMagic.MotionMagicAcceleration = encoderVelocityToRotationsPerSecond(encoderUnitsPer100MsPerSecond);
        return applyConfig();
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(double amps) {
        configuration.CurrentLimits.SupplyCurrentLimit = amps;
        configuration.CurrentLimits.SupplyCurrentThreshold = amps;
        configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

        return applyConfig();
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currentLimitConfig) {
        configuration.CurrentLimits.SupplyCurrentLimit = currentLimitConfig.currentLimit;
        configuration.CurrentLimits.SupplyCurrentThreshold = currentLimitConfig.triggerThresholdCurrent;
        configuration.CurrentLimits.SupplyTimeThreshold = currentLimitConfig.triggerThresholdTime;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = currentLimitConfig.enable;

        return applyConfig();
    }

    @Override
    public ErrorCode configStatorCurrentLimit(double amps) {
        configuration.CurrentLimits.StatorCurrentLimit = amps;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        return applyConfig();
    }

    @Override
    public ErrorCode disableStatorCurrentLimit() {
        configuration.CurrentLimits.StatorCurrentLimitEnable = false;
        return applyConfig();
    }

    @Override
    public double getSupplyAmps() {
        return Math.abs(this.getSupplyCurrent().getValue());
    }

    @Override
    public double getStatorAmps() {
        return Math.abs(this.getStatorCurrent().getValue());
    }

    @Override
    public double getAppliedVoltage() {
        return Math.abs(this.getSupplyVoltage().getValue());
    }

    @Override
    public double getMotorTemperature() {
        return this.getDeviceTemp().getValue();
    }

    @Override
    public double getVelocityEncoderUnitsPer100Ms() {
        return rotationsPerSecondToEncoderVelocity(this.getVelocity().getValue());
    }

    @Override
    public double getSelectedSensorPosition() {
        return encoderRotationsToEncoderUnits(this.getPosition().getValue());
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double encoderUnits) {
        return convertToErrorCode(this.setPosition(encoderUnitsToEncoderRotations(encoderUnits)));
    }

    @Override
    public void useIntegratedSensor() {
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        applyConfig();
    }

    @Override
    public void useCANCoder(int cancoderId) {
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        configuration.Feedback.FeedbackRemoteSensorID = cancoderId;
        applyConfig();
    }

    @Override
    public void setInverted(TalonFXInvertType invertType) {
        configuration.MotorOutput.Inverted = invertType == TalonFXInvertType.Clockwise ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        applyConfig();
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        configuration.MotorOutput.NeutralMode = neutralMode == NeutralMode.Coast ?
                NeutralModeValue.Coast : NeutralModeValue.Brake;
        applyConfig();
    }

    @Override
    public void setPIDF(MotorPIDF pidf) {
        switch (pidf.slotIndex) {
            case 0:
                configuration.Slot0.kP = pidf.kP;
                configuration.Slot0.kI = pidf.kI;
                configuration.Slot0.kD = pidf.kD;
                configuration.Slot0.kV = pidf.kF;
                break;
            case 1:
                configuration.Slot1.kP = pidf.kP;
                configuration.Slot1.kI = pidf.kI;
                configuration.Slot1.kD = pidf.kD;
                configuration.Slot1.kV = pidf.kF;
                break;
            case 2:
                configuration.Slot2.kP = pidf.kP;
                configuration.Slot2.kI = pidf.kI;
                configuration.Slot2.kD = pidf.kD;
                configuration.Slot2.kV = pidf.kF;
                break;
        }

        applyConfig();
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIndex, double integralZoneEncoderUnits) {
        // Integral zone is no longer configurable with Phoenix Pro
        return ErrorCode.OK;
    }

    @Override
    public void selectProfileSlot(int slotIndex) {
        velocityVoltageRequest.Slot = slotIndex;
        velocityCurrentRequest.Slot = slotIndex;
        motionMagicRequest.Slot = slotIndex;
    }

    @Override
    public void set(ControlMode mode, double demand) {
        set(mode, demand, 0.0);
    }

    @Override
    public void set(ControlMode mode, double demand, double arbitraryFeedForward) {
        switch (mode) {
            case PercentOutput:
                voltageOutRequest.Output = demand * 12.0;
                currentControlRequest = voltageOutRequest;
                break;
            case Velocity:
                if (Settings.kIsUsingCompBot) {
                    velocityCurrentRequest.Velocity = encoderVelocityToRotationsPerSecond(demand);
                    // velocityRequest.FeedForward = arbitraryFeedForward * 12.0;
                    velocityCurrentRequest.FeedForward = 0.0;
                    currentControlRequest = velocityCurrentRequest;
                } else {
                    velocityVoltageRequest.Velocity = encoderVelocityToRotationsPerSecond(demand);
                    velocityVoltageRequest.FeedForward = arbitraryFeedForward * 12.0;
                    currentControlRequest = velocityVoltageRequest;
                }
                break;
            case MotionMagic:
                motionMagicRequest.Position = encoderUnitsToEncoderRotations(demand);
                motionMagicRequest.FeedForward = arbitraryFeedForward * 12.0;
                currentControlRequest = motionMagicRequest;
                break;
            case Follower:
                followerRequest.MasterID = (int) demand;
                currentControlRequest = followerRequest;
                break;
            default:
                System.out.println(String.format("Unexpected control mode for Phoenix Pro motor! %s", mode.toString()));
                break;
        }

        this.setControl(currentControlRequest);
    }

    @Override
    public boolean isConnected() {
        return this.getRotorPosition().getStatus() == StatusCode.OK;
    }
}
