/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.Constants.ScoringPositions;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Intake;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Superstructure;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.swerve.Swerve;
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
    private Wrist wrist;
    private Intake intake;


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

    public DriverControls() {
        driver = new Xbox(0);
		coDriver = new Xbox(1);
        testController = new Xbox(4);
        singleController = new Xbox(5);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.25);

        swerve = Swerve.getInstance();
        verticalElevator = VerticalElevator.getInstance();
        horizontalElevator = HorizontalElevator.getInstance();
        wrist = Wrist.getInstance();
        intake = Intake.getInstance();

        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(Arrays.asList(swerve, verticalElevator, horizontalElevator, wrist, intake, s));
    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.requireModuleConfiguration();
        }
        swerve.setDriveNeutralMode(NeutralMode.Brake);
        s.enableCompressor(false);
    }

    @Override
    public void onLoop(double timestamp) {
        //RobotState.getInstance().outputToSmartDashboard();
        if(inAuto) {
            // Any auto-specific LED controls can go here
        } else {
            driver.update();
			coDriver.update();
            //singleController.update();
            //testController.update();
            if(oneControllerMode)
                singleController.update();
            if(oneControllerMode) oneControllerMode();
            else twoControllerMode();
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
            swerve.rotate(-90);
            //swerve.rotate(swerve.getHeading().rotateBy(Rotation2d.fromDegrees(90)).getDegrees());
        else if (driver.aButton.wasActivated()) 
            swerve.rotate(180);
        else if (driver.xButton.wasActivated())
            swerve.rotate(90);
        else if (driver.yButton.wasActivated())
            swerve.rotate(0);
        

        if (driver.startButton.isBeingPressed()) 
            swerve.setState(Swerve.ControlState.NEUTRAL);

        if (driver.backButton.wasActivated()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
        }

        // if (driver.leftTrigger.wasActivated()) {
        //     swerve.toggleEvade(true);
        // } else if(driver.leftTrigger.wasReleased()) {
        //     swerve.toggleEvade(false);
        // }


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


        double verticalElevatorYInput = -coDriver.getLeftY() * 0.25;
        double wristAngleYInput = -coDriver.getRightY() * 0.25;

        verticalElevator.acceptManualInput(verticalElevatorYInput);
        wrist.acceptManualInput(wristAngleYInput);

        if(coDriver.POV0.wasActivated()) {
            horizontalElevator.acceptManualInput(0.10);
        } else if(coDriver.POV180.wasActivated()) {
            horizontalElevator.acceptManualInput(-0.10);
        } else if(coDriver.POV0.wasReleased() || coDriver.POV180.wasReleased()) {
            horizontalElevator.acceptManualInput(0);
        }

        if(coDriver.POV90.wasActivated()) {
            intake.setSpeed(-0.5);
        } else if(coDriver.POV90.wasReleased()) {
            intake.setSpeed(0);
        }

        if(coDriver.POV270.wasActivated()) {
            intake.setSpeed(0.75);
        } else if(coDriver.POV270.wasReleased()) {
            intake.setSpeed(0.0);
        }

        if (coDriver.aButton.shortReleased()) {
            s.setScoringPositionState(ScoringPositions.LOW_CUBE);
        } else if (coDriver.aButton.longPressed()) {
            s.setScoringPositionState(ScoringPositions.LOW_CONE);
        }

        if (coDriver.xButton.shortReleased()) {
            s.setScoringPositionState(ScoringPositions.MID_CUBE);
        } else if (coDriver.xButton.longPressed()) {
            s.setScoringPositionState(ScoringPositions.MID_CONE);
        } 

        if (coDriver.yButton.shortReleased()) {
            s.setScoringPositionState(ScoringPositions.HIGH_CUBE);
        } else if (coDriver.yButton.longPressed()) {
            s.setScoringPositionState(ScoringPositions.HIGH_CONE);
        }
        if (coDriver.bButton.wasActivated()) {
            s.setScoringPositionState(ScoringPositions.STOW);
        }

        if (coDriver.leftBumper.wasActivated()) {
            s.intakeConeState();
        } else if (coDriver.leftBumper.wasReleased()) {
            intake.conformToState(Intake.ControlState.HOLD_CONE);
        }
    
        if (coDriver.rightBumper.wasActivated()) {
            s.intakeCubeState();
        } else if (coDriver.rightBumper.wasReleased()) {
            intake.conformToState(Intake.ControlState.HOLD_CUBE);
        }
        

        if (coDriver.rightTrigger.wasActivated()) {
            intake.conformToState(Intake.ControlState.EJECT_CUBE);
        } else if (coDriver.rightTrigger.wasReleased()) {
            intake.conformToState(Intake.ControlState.INTAKE_CONE);
            //intake.conformToState(Intake.ControlState.OFF);
        }
        if(coDriver.leftTrigger.wasActivated()) {
            intake.conformToState(Intake.ControlState.EJECT_CONE);
        } else if(coDriver.leftTrigger.wasReleased()) {
            intake.conformToState(Intake.ControlState.OFF);
        }

        if(coDriver.backButton.wasActivated()) {
            s.neutralState();
        }
    }
    
    private void oneControllerMode() {}
}
