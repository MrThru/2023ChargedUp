/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.field.NodeLocation;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.Claw;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.LEDs;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.LEDs.LEDColors;
import com.team1323.frc2023.subsystems.requests.ParallelRequest;
import com.team1323.frc2023.subsystems.requests.SequentialRequest;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.superstructure.SuperstructureCoordinator;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.ObjectDetector.Cone;
import com.team1323.io.Xbox;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

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

    private boolean highScoreSet = false;

    public DriverControls() {
        driver = new Xbox(0);
		coDriver = new Xbox(1);
        testController = new Xbox(4);
        singleController = new Xbox(5);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.15);

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

        subsystems = new SubsystemManager(Arrays.asList(swerve, cubeIntake, tunnel, verticalElevator, leds, horizontalElevator, wrist, shoulder, claw, s));
    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.requireModuleConfiguration();
        }
        swerve.setDriveNeutralMode(NeutralMode.Brake);
        cubeIntake.lockPosition();
        leds.configLEDs(LEDColors.TWINKLE);
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
            if(oneControllerMode) oneControllerMode();
            else manualMode();
            SmartDashboard.putNumber("timestamp", timestamp);
        }
    }

    @Override
    public void onStop(double timestamp) {
        subsystems.stop();
    }

    private void twoControllerMode() {
        double swerveYInput = -driver.getLeftX();
        double swerveXInput = -driver.getLeftY();
        double swerveRotationInput = -(driver.getRightX() + (driver.leftBumper.isBeingPressed() ? 0.3 : 0.0));
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, (Netlink.getBooleanValue("Slow Driving Enabled") || driver.leftTrigger.isBeingPressed()));
        
        SmartDashboard.putNumber("Translation Scalar", new Translation2d(swerveXInput, swerveYInput).norm());

        if (driver.bButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(-90));
            //swerve.rotate(swerve.getHeading().rotateBy(Rotation2d.fromDegrees(90)).getDegrees());
        else if (driver.aButton.wasActivated()) 
            swerve.rotate(Rotation2d.fromDegrees(180));
        else if (driver.xButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(90));
        else if (driver.yButton.wasActivated())
            swerve.rotate(Rotation2d.fromDegrees(0));
        

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

        if (driver.leftTrigger.wasActivated()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                Pose2d leftScoringPose = ScoringPoses.getLeftScoringPose(swerve.getPose());
                Pose2d rightScoringPose = ScoringPoses.getRightScoringPose(swerve.getPose());
                Pose2d closestScoringPose = rightScoringPose;
                if(leftScoringPose.distance(swerve.getPose()) < rightScoringPose.distance(swerve.getPose())) {
                    closestScoringPose = leftScoringPose;
                }
                if(highScoreSet) {
                    s.coneHighScoringSequence(closestScoringPose);
                } else {
                    s.coneMidScoringSequence(closestScoringPose);
                }
            } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube || !tunnel.allowSingleIntakeMode()) {
                if(!tunnel.allowSingleIntakeMode() && claw.getCurrentHoldingObject() == Claw.HoldingObject.None) {
                    s.intakeCubeAndScore(null, null, null);
                    return;
                }
                if(highScoreSet) {
                    s.cubeHighScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()));
                } else {
                    s.cubeMidScoringSequence(ScoringPoses.getCenterScoringPose(swerve.getPose()));
                }
            }
        }


        if(driver.POV0.wasActivated()) {
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
        }



        if(coDriver.aButton.wasActivated()) {
            if(tunnel.allowSingleIntakeMode()) {
                s.intakeState(Tunnel.State.SINGLE_INTAKE);
            }
        } else if(coDriver.aButton.isBeingPressed()) {
            if(tunnel.getFrontBanner()) {
                s.postIntakeState();
            } 
        } else if(coDriver.aButton.wasReleased()) {
            s.postIntakeState();
        }

        
        if (coDriver.bButton.shortReleased()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cone) {
                s.coneStowSequence();
            } else if(claw.getCurrentHoldingObject() == Claw.HoldingObject.Cube) {
                s.request(SuperstructureCoordinator.getInstance().getCubeStowChoreography());
            } else {
                s.request(SuperstructureCoordinator.getInstance().getFullStowChoreography());
            }
        } else if(coDriver.bButton.longPressed()) {
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None) {
                s.coneIntakeSequence();
            }
        }

        if(coDriver.xButton.wasActivated()) {
            highScoreSet = false;
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None && !tunnel.allowSingleIntakeMode()) {
                s.handOffCubeState();
            }
        }
        if(coDriver.yButton.wasActivated()) {
            highScoreSet = true;
            if(claw.getCurrentHoldingObject() == Claw.HoldingObject.None && !tunnel.allowSingleIntakeMode()) {
                s.handOffCubeState();
            }
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

        SmartDashboard.putNumber("Vertical Elevator Manual Input", verticalElevatorYInput);
        
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
                s.request(SuperstructureCoordinator.getInstance().getFullStowChoreography());
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
        // if(testController.yButton.wasActivated()) {
        //     tunnel.setState(Tunnel.State.SPIT);
        // }
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

        if(testController.aButton.wasActivated()) {
            claw.conformToState(Claw.ControlState.CONE_INTAKE);
            //claw.setPercentSpeed(0.5);
        }
        if(testController.bButton.wasActivated()) {
            claw.reset();
            claw.conformToState(Claw.ControlState.CONE_OUTAKE);
        } else if(testController.bButton.wasReleased()) {
            claw.conformToState(Claw.ControlState.OFF);
        }

        /*if(testController.xButton.wasActivated()) {
            s.intakeState(Tunnel.State.SPIT);
        } else if(testController.xButton.wasReleased()) {
            cubeIntake.setIntakeSpeed(0.0);
            tunnel.setState(Tunnel.State.OFF);
        }*/

        if(testController.xButton.wasActivated()) {
            s.intakeState(Tunnel.State.DETECT);
        } else if(testController.xButton.wasReleased()) {
            //cubeIntake.setIntakeSpeed(0.0);
            s.postIntakeState();
        }
        
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

        // D-pad controls for vision PID
        if (coDriver.POV0.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getCenterScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            //s.cubeHighScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation());
        } else if (coDriver.POV270.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getRightScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0}); 
            //s.coneMidScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation());
        } else if (coDriver.POV90.wasActivated()) {
            Pose2d scoringPose = ScoringPoses.getLeftScoringPose(swerve.getPose());
            SmartDashboard.putNumberArray("Path Pose", new double[]{scoringPose.getTranslation().x(), scoringPose.getTranslation().y(), scoringPose.getRotation().getDegrees(), 0.0});
            //s.coneHighScoringSequence(scoringPose);
            swerve.startVisionPID(scoringPose, scoringPose.getRotation());
        }

        if(coDriver.backButton.wasActivated()) {
            s.neutralState();
        }

    }
    
    private void oneControllerMode() {}
}
