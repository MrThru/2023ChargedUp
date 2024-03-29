/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.SignalLogger;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.Claw.HoldingObject;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.LEDs;
import com.team1323.frc2023.subsystems.LEDs.LEDColors;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.requests.EmptyRequest;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.GridTracker;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.io.Xbox;
import com.team1323.lib.util.Netlink;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to assign controller inputs to robot actions
 */
public class DriverControls implements Loop {

    private static DriverControls instance = null;

    public static DriverControls getInstance() {
        if (instance == null)
            instance = new DriverControls();
        return instance;
    }

	Xbox driver, coDriver, singleController, testController;
    //PS4 driver;

    private Swerve swerve;
    private VerticalElevator verticalElevator;
    private HorizontalElevator horizontalElevator;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;
    private CubeIntake cubeIntake;
    private Tunnel tunnel;

    private LEDs leds;

    private SubsystemManager subsystems;
    public SubsystemManager getSubsystems(){ return subsystems; }

    private Superstructure s;

    private final boolean oneControllerMode = false;
    private boolean inAuto = true;
    public void setAutoMode(boolean auto) {
        inAuto = auto;
    }

    public boolean getInAuto() {
        return inAuto;
    }

    private NodeLocation.Row targetScoringRow = NodeLocation.Row.MIDDLE;
    private Tunnel.State lastTunnelState = Tunnel.State.OFF;
    private boolean balanceRumbleActivated = false;

    public DriverControls() {
        driver = new Xbox(0);
		coDriver = new Xbox(1);
        testController = new Xbox(4);
        singleController = new Xbox(5);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.6); //0.6

        swerve = Swerve.getInstance();
        verticalElevator = VerticalElevator.getInstance();
        horizontalElevator = HorizontalElevator.getInstance();
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();
        cubeIntake = CubeIntake.getInstance();
        tunnel = Tunnel.getInstance();

        leds = LEDs.getInstance();

        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(Arrays.asList(swerve, cubeIntake, tunnel, verticalElevator, horizontalElevator, wrist, shoulder, claw, leds, s));
    }

    @Override
    public void onStart(double timestamp) {
        if (inAuto) {
            swerve.requireModuleConfiguration();
        } else {
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        }
        SmartDashboard.putBoolean("Subsystems Coast Mode", false);
        SmartDashboard.putBoolean("Swerve Coast Mode", false);

        swerve.setDriveNeutralMode(NeutralMode.Brake);
        cubeIntake.lockPosition();
        leds.configLEDs(LEDs.LEDColors.TWINKLE);
    }

    @Override
    public void onLoop(double timestamp) {
        if(inAuto) {
            // Any auto-specific LED controls can go here
        } else {
            driver.update();
			coDriver.update();
            //singleController.update();
            testController.update();
            if(oneControllerMode)
                singleController.update();
            if(!oneControllerMode) 
                twoControllerMode();;;;;;;
            SmartDashboard.putNumber("timestamp", timestamp);
        }
    }

    @Override
    public void onStop(double timestamp) {
        if(!inAuto) {
            SmartDashboard.putBoolean("Subsystems Coast Mode", true);
        }
        subsystems.stop();
    }

    private void twoControllerMode() {
        double swerveYInput = -driver.getLeftX();
        double swerveXInput = -driver.getLeftY();
        double swerveRotationInput = -driver.getRightX(); //+ (driver.leftBumper.isBeingPressed() ? 0.3 : 0.0));
        
        Translation2d currentTranslationVector = Translation2d.identity();
        if(Netlink.getBooleanValue("Driving Rotation Offset Enabled")) {
            currentTranslationVector = new Translation2d(swerveXInput, swerveYInput).rotateBy(Rotation2d.fromDegrees(Netlink.getNumberValue("Driving Rotation Offset")));
            swerveXInput = currentTranslationVector.x();
            swerveYInput = currentTranslationVector.y();
        }
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, (Netlink.getBooleanValue("Slow Driving Enabled")/* || driver.leftTrigger.isBeingPressed()*/));
        
        SmartDashboard.putNumber("Translation Scalar", new Translation2d(swerveXInput, swerveYInput).norm());

        if(driver.bButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(-90));
            //swerve.rotate(swerve.getHeading().rotateBy(Rotation2d.fromDegrees(90)).getDegrees());
        else if(driver.aButton.wasActivated()) 
            swerve.rotate(Rotation2d.fromDegrees(180));
        else if (driver.xButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(90));
        else if (driver.yButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(0));
        
        
        /*if (driver.rightTrigger.wasActivated()) {
            NodeLocation dashboardNodeLocation = NodeLocation.getDashboardLocation();
            Pose2d scoringPose = ScoringPoses.getScoringPose(dashboardNodeLocation);
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            s.scoringSequence(dashboardNodeLocation);
        }*/

        if(driver.rightTrigger.wasActivated()) {
            tunnel.setState(Tunnel.State.EJECT_ONE);
            //swerve.startBalancePID();
        }

        if(testController.rightBumper.wasActivated()) {
            SignalLogger.startLogger();
            System.out.println("Starting Logger");
        } else if(testController.rightBumper.wasReleased()) {
            SignalLogger.stopLogger();
            System.out.println("Stopping Logger");
        }
        /*if (driver.rightTrigger.wasActivated()) {
            Translation2d conePosition = LimelightProcessor.getInstance().getConePosition();
            if (!conePosition.equals(Translation2d.identity())) {
                s.coneIntakeSequence();
                Pose2d intakingPose = new Pose2d(conePosition, swerve.getPose().getRotation())
                        .transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
                swerve.startVisionPID(intakingPose, intakingPose.getRotation(), false);
            }
        }*/

        /*if(driver.rightTrigger.wasActivated()) {
            //s.shuttleIntakeSequence();
            /*Pose2d kShuttleIntakePosition = (AllianceChooser.getAlliance() == Alliance.Blue) ? new Pose2d(new Translation2d(554.34, 287.25), Rotation2d.fromDegrees(90)) :
                            new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0));
            swerve.startVisionPID(kShuttleIntakePosition, kShuttleIntakePosition.getRotation(), false);*/
            //swerve.startBalancePID();
            //s.tripleCubeScoringSequence();
            //cubeIntake.conformToState(CubeIntake.State.FLOOR);
        /*} else if(driver.rightTrigger.wasReleased()) {
            cubeIntake.conformToState(CubeIntake.State.STOWED);
        }*/
            
        if (driver.backButton.wasActivated()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensorsBasedOnAlliance();
            swerve.resetAveragedDirection();
        }

        if (driver.startButton.wasActivated()) {
            swerve.stop();
            swerve.resetVisionPID();
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        }

        if (driver.leftTrigger.wasActivated()) {
            System.out.println("Cone Intaking Sequence" + s.coneIntakingSequence);
            if(!s.coneIntakingSequence) {
                System.out.println("Detected Object When Tracking: " + claw.getCurrentHoldingObject().toString());
                boolean limelightConnected = LimelightProcessor.getInstance().isConnected();
                if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                    Pose2d leftScoringPose = ScoringPoses.getLeftScoringPose(swerve.getPose());
                    Pose2d rightScoringPose = ScoringPoses.getRightScoringPose(swerve.getPose());
                    Pose2d closestScoringPose = rightScoringPose;
                    if(leftScoringPose.getTranslation().distance(swerve.getPose().getTranslation()) < rightScoringPose.getTranslation().distance(swerve.getPose().getTranslation())) {
                        closestScoringPose = leftScoringPose;
                        System.out.println("Left Scoring Pose");
                    } else {
                        System.out.println("Right Scoring Pose");
                    }
                    System.out.println("Target scoring row: " + targetScoringRow.toString());
                    if(targetScoringRow == NodeLocation.Row.TOP) {
                        if(limelightConnected) {
                            s.coneHighScoringSequence(closestScoringPose);
                        } else {
                            s.request(SuperstructureCoordinator.getInstance().getConeHighScoringChoreography());
                        }
                    } else if(targetScoringRow == NodeLocation.Row.MIDDLE) {
                        if(limelightConnected) {
                            s.coneMidScoringSequence(closestScoringPose);
                        } else {
                            s.request(SuperstructureCoordinator.getInstance().getConeMidScoringChoreography());
                        }
                    }
                } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
                    if(targetScoringRow == NodeLocation.Row.TOP) {
                        if(limelightConnected) {
                            s.cubeHighScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()), new VisionPIDBuilder().build(), true);
                        } else {
                            s.request(SuperstructureCoordinator.getInstance().getCubeHighScoringChoreography());
                        }
                    } else if(targetScoringRow == NodeLocation.Row.MIDDLE) {
                        if(limelightConnected) {
                            s.cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()), true);
                        } else {
                            s.request(SuperstructureCoordinator.getInstance().getCubeMidScoringChoreography());
                        }
                    }
                }
                if(targetScoringRow == NodeLocation.Row.BOTTOM && !tunnel.allowSingleIntakeMode()) {
                    s.cubeLowScoringSequence(ScoringPoses.getClosestScoringPosition(swerve.getPose()));
                }
                GridTracker.getInstance().addRobotScorePosition(null, targetScoringRow, null);
            }
        } else if(driver.leftTrigger.wasReleased()) {
            swerve.stop(); //POOFS!!!
            swerve.resetVisionPID();
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        }

        if(driver.rightBumper.wasActivated()) {
            leds.configLEDs(LEDColors.CUBE);
            //swerve.setVelocity(Rotation2d.fromDegrees(0),72.0);
        }
        if(driver.leftBumper.wasActivated()) {
            leds.configLEDs(LEDColors.CONE);
        }

        if(coDriver.startButton.wasActivated()) {
            if(claw.getCurrentHoldingObject() != Claw.HoldingObject.Cube)
                s.manualShelfSequence();
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        } else if(coDriver.startButton.wasReleased()) {
            s.request(SuperstructureCoordinator.getInstance().getConeStowChoreography());
        }

        if(coDriver.rightTrigger.wasActivated()) {
            if(claw.getCurrentHoldingObject() != Claw.HoldingObject.Cube)
                s.shuttleIntakeSequence(false);
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
        } else if(coDriver.rightTrigger.wasReleased()) {
            s.request(SuperstructureCoordinator.getInstance().getConeStowChoreography());
        }
        /*if(driver.POV0.wasActivated()) {
            swerve.setCenterOfRotation(new Translation2d(0, Math.hypot(Constants.kWheelbaseLength, Constants.kWheelbaseWidth) + 8).rotateBy(Rotation2d.fromDegrees(-90).rotateBy(swerve.getHeading().inverse())));
        } else if(driver.POV90.wasActivated()) {
            swerve.setCenterOfRotation(new Translation2d(0, Math.hypot(Constants.kWheelbaseLength, Constants.kWheelbaseWidth) + 8).rotateBy(Rotation2d.fromDegrees(0).rotateBy(swerve.getHeading().inverse())));
        } else if(driver.POV180.wasActivated()) {
            swerve.setCenterOfRotation(new Translation2d(0, Math.hypot(Constants.kWheelbaseLength, Constants.kWheelbaseWidth) + 8).rotateBy(Rotation2d.fromDegrees(90).rotateBy(swerve.getHeading().inverse())));
        } else if(driver.POV270.wasActivated()) {
            swerve.setCenterOfRotation(new Translation2d(0, Math.hypot(Constants.kWheelbaseLength, Constants.kWheelbaseWidth) + 8).rotateBy(Rotation2d.fromDegrees(180).rotateBy(swerve.getHeading().inverse())));
        }
        if((!driver.POV0.isBeingPressed() && !driver.POV90.isBeingPressed() && !driver.POV180.isBeingPressed() && !driver.POV270.isBeingPressed())) {
            swerve.setCenterOfRotation(new Translation2d());
        }*/
        if(driver.POV270.wasActivated()) {
            double sign = AllianceChooser.getAlliance() == Alliance.Blue ? 1.0 : -1.0;
            double offset = sign * 3.0;
            if (swerve.isVisionPIDDone()) {
                Pose2d robotPose = swerve.getPose();
                swerve.startVisionPID(new Pose2d(robotPose.getTranslation().translateBy(new Translation2d(0.0, offset)), robotPose.getRotation()), robotPose.getRotation(), false);
            } else {
                swerve.setVisionPIDTarget(swerve.getVisionPIDTarget().translateBy(new Translation2d(0.0, offset)));
            }
        }
        if(driver.POV90.wasActivated()) {
            double sign = AllianceChooser.getAlliance() == Alliance.Blue ? -1.0 : 1.0;
            double offset = sign * 3.0;
            if (swerve.isVisionPIDDone()) {
                Pose2d robotPose = swerve.getPose();
                swerve.startVisionPID(new Pose2d(robotPose.getTranslation().translateBy(new Translation2d(0.0, offset)), robotPose.getRotation()), robotPose.getRotation(), false);
            } else {
                swerve.setVisionPIDTarget(swerve.getVisionPIDTarget().translateBy(new Translation2d(0.0, offset)));
            }
        }
        if(driver.rightCenterClick.wasActivated()) {
            swerve.zukLockDrivePosition();
        } else if(driver.rightCenterClick.wasReleased()) {
            swerve.stop();
        }

        if(coDriver.aButton.wasActivated()) {
            if(tunnel.getFrontBanner()) {
                s.communityIntakeState();
            } else {
                s.intakeState(Tunnel.State.SINGLE_INTAKE);
            }
            LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
            /*if(tunnel.allowSingleIntakeMode() && !tunnel.cubeOnBumper()) {
            } else if(!tunnel.allowSingleIntakeMode() && !tunnel.cubeOnBumper()) {
                tunnel.stateRequest(Tunnel.State.SINGLE_INTAKE);
            }*/
        } else if(coDriver.aButton.isBeingPressed()) {
            if(tunnel.getRearBanner() && tunnel.getState() == Tunnel.State.SINGLE_INTAKE) {
                s.postIntakeState(0.25); //0.25
            }
        } else if(coDriver.aButton.wasReleased()) {
            if(tunnel.getState() == Tunnel.State.COMMUNITY || tunnel.getState() == Tunnel.State.TRIPLE_CUBE_HOLD) {
                cubeIntake.conformToState(CubeIntake.State.FLOOR);
            } else {
                s.postIntakeState(1.0);
                tunnel.queueShutdown(true);
            }
        }
        SmartDashboard.putNumber("RB Pressed", (coDriver.rightBumper.isBeingPressed() ? 1 : 0));

        if(coDriver.rightCenterClick.wasActivated()) {
            if(claw.getCurrentHoldingObject() != Claw.HoldingObject.Cube) {
                claw.conformToState(Claw.ControlState.CONE_OUTAKE);
            } else {
                claw.conformToState(Claw.ControlState.CUBE_OUTAKE);
            }
            System.out.println(claw.getCurrentHoldingObject().toString() + " object manually ejected");
        } else if(coDriver.rightCenterClick.wasReleased()) {
            //claw.conformToState(Claw.ControlState.OFF);
            claw.setCurrentHoldingObject(Claw.HoldingObject.None);
        }

        if(coDriver.leftBumper.wasActivated()) {
            s.reverseSubsystemsState();
        } else if(coDriver.leftBumper.wasReleased()) {
            tunnel.setState(Tunnel.State.OFF);
            cubeIntake.setIntakeSpeed(0);
        }
        /*if(coDriver.leftBumper.wasActivated()) {
            s.handOffCubeState();
        }*/

        if(coDriver.bButton.shortReleased()) {
            if(claw.getState() != Claw.ControlState.CONE_INTAKE || claw.getRPM() > 2000 || claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                s.objectAwareStowSequence();
            }
        } else if(coDriver.bButton.longReleased()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None && (claw.getRPM() > 2000 || claw.getState() == Claw.ControlState.OFF)) {
                s.request(new ParallelRequest(
                    SuperstructureCoordinator.getInstance().getFullStowChoreography(true),
                    claw.stateRequest(Claw.ControlState.OFF)
                ));
                System.out.println("Button Pressed For Full Stow Choreography");
            } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube && claw.getState() == Claw.ControlState.CUBE_INTAKE) {
                s.request(new ParallelRequest(
                    s.objectAwareStow()
                ));
            }
        } else if(coDriver.bButton.longPressed()) {
            if(claw.getCurrentHoldingObject() != Claw.HoldingObject.Cube) {
                claw.setCurrentHoldingObject(HoldingObject.None);
                s.coneIntakeSequence();
                LimelightProcessor.getInstance().setPipeline(Pipeline.FIDUCIAL);
                System.out.println("Intaking Cone Request");
            }
        }

        if(coDriver.xButton.shortReleased()) {
            targetScoringRow = NodeLocation.Row.MIDDLE;
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None && !tunnel.allowSingleIntakeMode()) {
                s.handOffCubeState(SuperstructureCoordinator.getInstance()::getHalfCubeStowChoreography);
            }
            leds.configLEDs(LEDs.LEDColors.ORANGE);
        } else if(coDriver.xButton.longPressed()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                s.request(SuperstructureCoordinator.getInstance().getConeMidScoringChoreography());
            } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
                s.request(SuperstructureCoordinator.getInstance().getCubeMidScoringChoreography());
            }
        }
        if(coDriver.yButton.shortReleased()) {
            targetScoringRow = NodeLocation.Row.TOP;
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None && !tunnel.allowSingleIntakeMode()) {
                s.handOffCubeState(SuperstructureCoordinator.getInstance()::getHalfCubeStowChoreography);
            }
            leds.configLEDs(LEDs.LEDColors.RED);

        } else if(coDriver.yButton.longPressed()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                s.request(SuperstructureCoordinator.getInstance().getConeHighScoringChoreography());
            } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
                s.request(SuperstructureCoordinator.getInstance().getCubeHighScoringChoreography());
            }
        }



        if(coDriver.leftTrigger.wasActivated()/*  && AllianceChooser.getCommunityBoundingBox().pointWithinBox(swerve.getPose().getTranslation())*/) {
            s.communityIntakeState();
        } else if(coDriver.leftTrigger.wasReleased()) {
            //verticalElevator.setPosition(0.25);
            cubeIntake.conformToState(CubeIntake.State.FLOOR);
        }


        if(coDriver.rightBumper.wasActivated()) {
            //lastTunnelState = tunnel.getState();
            tunnel.setState(Tunnel.State.SPIT);
            cubeIntake.setIntakeSpeed(0.5);
            System.out.println("Right Bumper Activated at " + DriverStation.getMatchTime());
        } else if(coDriver.rightBumper.wasReleased()) {
            //lastTunnelState = (lastTunnelState == Tunnel.State.SPIT) ? Tunnel.State.OFF : lastTunnelState;
            tunnel.setState(Tunnel.State.OFF);
            cubeIntake.setIntakeSpeed(0.0);
        }

        if(coDriver.leftCenterClick.wasActivated() && coDriver.bButton.isBeingPressed()) {
            //wrist.setPosition(12); // 12
            s.request(SuperstructureCoordinator.getInstance().getConeFlipChoreography());
            claw.conformToState(Claw.ControlState.OFF);
        } else if(coDriver.leftCenterClick.wasReleased() && coDriver.bButton.isBeingPressed()) {
            //s.coneIntakeSequence();
            if(shoulder.getPosition() < 0) {
                s.request(new SequentialRequest(
                    new ParallelRequest(
                        shoulder.angleRequest(-55.76),
                        horizontalElevator.extensionRequest(0.25)
                    ),
                    s.getConeIntakeSequence()
                ));
            }
        }

        if(coDriver.POV180.wasActivated() || coDriver.POV135.wasActivated() || coDriver.POV225.wasActivated()) {
            cubeIntake.conformToState(CubeIntake.State.FLOOR);
        } else if((coDriver.POV180.wasReleased() || coDriver.POV135.wasReleased() || coDriver.POV225.wasReleased()) && 
                    !(coDriver.POV180.isBeingPressed() || coDriver.POV135.isBeingPressed() || coDriver.POV225.isBeingPressed())) {
            cubeIntake.conformToState(CubeIntake.State.STOWED);
        }

        if(coDriver.POV0.wasActivated()) {
            claw.setCurrentConeOffset(Claw.ConeOffset.Center);
            ScoringPoses.updateConeLateralOffset();
        }

        if(coDriver.POV90.wasActivated()) {
            claw.setCurrentConeOffset(Claw.ConeOffset.Left);
            ScoringPoses.updateConeLateralOffset();
        }
        if(coDriver.POV270.wasActivated()) {
            claw.setCurrentConeOffset(Claw.ConeOffset.Right);
            ScoringPoses.updateConeLateralOffset();
        }

        if(cubeIntake.getState() == CubeIntake.State.INTAKE) {
            cubeIntake.acceptManualInput(-coDriver.getLeftY() * 0.1);
        }
        
        double rightStickY = -coDriver.getRightY() * 0.15;
        verticalElevator.acceptManualInput(rightStickY);
        if (Math.abs(rightStickY) != 0) {
            s.request(new EmptyRequest());
        }

        if (coDriver.backButton.shortReleased()) {
            verticalElevator.startCurrentZeroing();
            horizontalElevator.startCurrentZeroing();
            //shoulder.startCurrentZeroing();
            //wrist.startCurrentZeroing();
            //cubeIntake.startCurrentZeroing();
        } else if (coDriver.backButton.longPressed()) {
            shoulder.stop();
            wrist.stop();
            cubeIntake.stop();
            swerve.stop();

            verticalElevator.startCurrentZeroing();
            horizontalElevator.startCurrentZeroing();
            shoulder.setPositionToAbsolute();
            wrist.setPositionToAbsolute();
            cubeIntake.setPositionToAbsolute();
            swerve.forceZeroModuleAngles();

            shoulder.stop();
            wrist.stop();
            cubeIntake.stop();
            swerve.stop();
        }

        /*if(s.coneIntakingSequence) {
            wrist.acceptManualInput(-coDriver.getLeftY() * 0.25);
        } else {
            wrist.lockPosition();
        }*/
        /*if(claw.getRPM() <= 100 && coDriver.rightTrigger.isBeingPressed() && claw.rumbleStopwatch.getTime() > 1.0) {
            driver.rumble(1.0, 2.0);
        }*/
        if(tunnel.getCubeEnteredNotifier()) {
            driver.rumble(1.0, 1.0);
        }
        if(claw.needsToNotifyDrivers()) {
            System.out.println("Rumbling controller for cone");
            driver.rumble(1.0, 1.0);
        }
        if(Util.isInRange(DriverStation.getMatchTime(), 0.0, 15.0) && !balanceRumbleActivated) {
            balanceRumbleActivated = true;
            coDriver.rumble(1.0, 2.0);
            driver.rumble(1.0, 2.0);
        }
        
    }

    private void manualMode() {
        double swerveYInput = -driver.getLeftX();
        double swerveXInput = -driver.getLeftY();
        double swerveRotationInput = -(driver.getRightX() + (driver.leftBumper.isBeingPressed() ? 0.3 : 0.0));
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, (Netlink.getBooleanValue("Slow Driving Enabled") || driver.leftTrigger.isBeingPressed()));
        
        SmartDashboard.putNumber("Translation Scalar", new Translation2d(swerveXInput, swerveYInput).norm());

        if (driver.startButton.wasActivated()) {
            //swerve.setVelocity(Rotation2d.fromDegrees(180), 36.0);
            //swerve.setState(Swerve.ControlState.NEUTRAL);
            NodeLocation dashboardNodeLocation = NodeLocation.getDashboardLocation();
            Pose2d scoringPose = ScoringPoses.getScoringPose(dashboardNodeLocation);
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            s.scoringSequence(dashboardNodeLocation);
        }

        if (driver.backButton.wasActivated()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
        }

        double verticalElevatorYInput = -coDriver.getLeftY() * 0.2;
        double horizontalElevatorYInput = -coDriver.getRightY() * 0.2;
        double shoulderYInput = -testController.getLeftY() * 0.25;
        double wristAngleYInput = -testController.getRightY() * 0.25;

        verticalElevator.acceptManualInput(verticalElevatorYInput);
        horizontalElevator.acceptManualInput(horizontalElevatorYInput);
        shoulder.acceptManualInput(shoulderYInput);
        wrist.acceptManualInput(wristAngleYInput);
        //cubeIntake.acceptManualInput(shoulderYInput);

        SmartDashboard.putNumber("Vertical Elevator Manual Input", verticalElevatorYInput);
        SmartDashboard.putNumber("Shoulder Manual Input", shoulderYInput);
        
        if (coDriver.aButton.shortReleased()) {
            s.request(new SequentialRequest(
                SuperstructureCoordinator.getInstance().getConeIntakeChoreography(),
                claw.stateRequest(Claw.ControlState.CONE_INTAKE)
            ));
            //SuperstructureCoordinator.getInstance().getConeIntakeChoreography();
        } else if(coDriver.aButton.longPressed()) {
            s.request(new SequentialRequest(
                SuperstructureCoordinator.getInstance().getCubeIntakeChoreography(),
                new ParallelRequest(
                    tunnel.stateRequest(Tunnel.State.EJECT_ONE),
                    claw.stateRequest(Claw.ControlState.CUBE_INTAKE)
                )
            ));        
        }
        if (coDriver.bButton.wasActivated()) {
            //LimelightHelper.setPipelineIndex("limelight", 3);
            if(claw.getState() == Claw.ControlState.CONE_INTAKE) {
                s.coneStowSequence();
            } else if(claw.getState() == Claw.ControlState.CUBE_INTAKE) {
                s.request(SuperstructureCoordinator.getInstance().getCubeStowChoreography());
            } else {
                s.request(SuperstructureCoordinator.getInstance().getFullStowChoreography(true));
            }
        }
        if(coDriver.yButton.wasActivated()) {
            s.request(SuperstructureCoordinator.getInstance().getCubeHighScoringChoreography());
            
        }
        if (coDriver.xButton.wasActivated()) {
            s.request(new SequentialRequest(
                SuperstructureCoordinator.getInstance().getCubeIntakeChoreography(),
                new ParallelRequest(
                    tunnel.stateRequest(Tunnel.State.EJECT_ONE),
                    claw.stateRequest(Claw.ControlState.CUBE_INTAKE)
                )
            ));
            //SuperstructureCoordinator.getInstance().getCubeIntakeChoreography();
        } 
        /*if(testController.yButton.wasActivated()) {
            shoulder.setPosition(0);
        }*/

        
        /*if(coDriver.aButton.wasActivated()) {
            s.intakeState(Tunnel.State.DETECT);
        } else if(coDriver.aButton.wasReleased()) {
            s.postIntakeState();
        }

        if(coDriver.bButton.wasActivated()) {
            s.intakeState(Tunnel.State.COMMUNITY);
        } else if(coDriver.bButton.wasReleased()) {
            s.postIntakeState();
        } */

        /*if(testController.aButton.wasActivated()) {
            claw.conformToState(Claw.ControlState.CONE_INTAKE);
            //claw.setPercentSpeed(0.5);
        }
        if(testController.bButton.wasActivated()) {
            claw.reset();
            claw.conformToState(Claw.ControlState.CONE_OUTAKE);
        } else if(testController.bButton.wasReleased()) {
            claw.conformToState(Claw.ControlState.OFF);
        }*/
        
        /*if(testController.aButton.wasActivated()) {
            tunnel.setState(Tunnel.State.MANUAL);
            tunnel.setConveyorSpeed(Netlink.getNumberValue("Tunnel Manual Conveyor Speed"));
        } else if(testController.aButton.wasReleased()) {
            tunnel.setConveyorSpeed(0);
        }

        if(testController.bButton.wasActivated()) {
            tunnel.setState(Tunnel.State.MANUAL);
            tunnel.setRollerSpeed(Netlink.getNumberValue("Tunnel Manual Top Roller Speed"));
        } else if(testController.bButton.wasReleased()) {
            tunnel.setRollerSpeed(0);
        }*/

        /*if(testController.xButton.wasActivated()) {
            s.intakeState(Tunnel.State.SPIT);
        } else if(testController.xButton.wasReleased()) {
            cubeIntake.setIntakeSpeed(0.0);
            tunnel.setState(Tunnel.State.OFF);
        }*/

        /*if(testController.xButton.wasActivated()) {
            s.intakeState(Tunnel.State.DETECT);
        } else if(testController.xButton.wasReleased()) {
            //cubeIntake.setIntakeSpeed(0.0);
            s.postIntakeState();
        }*/
        
        /*
        if(coDriver.aButton.wasActivated()) {
            s.intakeState(Tunnel.State.SPIT);
        } else if(coDriver.aButton.wasReleased()) {
            s.postIntakeState();
        }

        if(coDriver.bButton.wasActivated()) {
        } else if(coDriver.bButton.wasReleased()) {
            s.postIntakeState();
        }
        if(coDriver.yButton.wasActivated()) {
            cubeIntake.setIntakeCurrent(10.0);
        }
        if(coDriver.xButton.wasActivated()) {
            cubeIntake.setIntakeCurrent(10);
            cubeIntake.setIntakeSpeed(0.1);
            cubeIntake.setPosition(Constants.CubeIntake.kIntakeAngle);
            verticalElevator.setPosition(4.0);
        } else if(coDriver.xButton.wasReleased()) {
            //cubeIntake.setIntakeCurrent(120);
            cubeIntake.setIntakeSpeed(0);
        }*/
        /*s
        if(coDriver.aButton.wasActivated()) {
            tunnel.setFrontSpeed(0.25);
            cubeIntake.setIntakeSpeed(0.25);
        } else if(coDriver.aButton.wasReleased()) {
            tunnel.setFrontSpeed(0.0);
            cubeIntake.setIntakeSpeed(0);
        }*/


        if(testController.aButton.wasActivated()) {
            //horizontalElevator.setPosition(20.0);
            //verticalElevator.setPosition(10.0);
            //wrist.setPosition(-90);
            shoulder.setPosition(-45);
            //cubeIntake.setPosition(Constants.CubeIntake.kIntakeAngle);
            // cubeIntake.setIntakeSpeed(0.25);
            // tunnel.setState(Tunnel.State.MANUAL);
            // tunnel.setAllSpeeds(0.25);
            // claw.conformToState(Claw.ControlState.CUBE_OUTAKE);
        }
        if(testController.bButton.wasActivated()) {
            //horizontalElevator.setPosition(0.5);
            //verticalElevator.setPosition(0.5);
            //wrist.setPosition(0);
            shoulder.setPosition(90);
            //cubeIntake.setPosition(100);
            // cubeIntake.setIntakeSpeed(0.0);
            // tunnel.setState(Tunnel.State.OFF);
            // claw.conformToState(Claw.ControlState.OFF);
        }
        if(testController.yButton.wasActivated()) {
            //horizontalElevator.setPosition(30.0);
            //verticalElevator.setPosition(19.0);
            //wrist.setPosition(90);
            shoulder.setPosition(169.0);
            //cubeIntake.setPosition(0);

        }

        if (testController.POV0.wasActivated()) {
            s.request(SuperstructureCoordinator.getInstance().getConeHighScoringChoreography());
        } else if (testController.POV90.wasActivated()) {
            s.request(SuperstructureCoordinator.getInstance().getCubeIntakeChoreography());
        } else if (testController.POV180.wasActivated()) {
            s.objectAwareStowSequence();
        } else if (testController.POV270.wasActivated()) {
            s.request(SuperstructureCoordinator.getInstance().getCubeHighScoringChoreography());
        }

        // D-pad controls for vision PID
        if (coDriver.POV0.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getCenterScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            //s.cubeHighScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation(), false);
        } else if (coDriver.POV270.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getRightScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            //s.coneMidScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation(), false);
        } else if (coDriver.POV90.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getLeftScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0});
            //s.coneHighScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation(), false);
        }

        if(coDriver.backButton.wasActivated()) {
            s.neutralState();
        }

    }
    
    private void oneControllerMode() {}
}
