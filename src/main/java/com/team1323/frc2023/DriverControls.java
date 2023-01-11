/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.HorizontalElevator;
import com.team1323.frc2023.subsystems.Intake;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Swerve;
import com.team1323.frc2023.subsystems.VerticalElevator;
import com.team1323.frc2023.subsystems.Wrist;
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
		coDriver.rightBumper.setLongPressDuration(1.0);

        swerve = Swerve.getInstance();
        verticalElevator = VerticalElevator.getInstance();
        horizontalElevator = HorizontalElevator.getInstance();
        wrist = Wrist.getInstance();
        intake = Intake.getInstance();

        subsystems = new SubsystemManager(Arrays.asList(swerve, verticalElevator, horizontalElevator, wrist, intake));

    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.requireModuleConfiguration();
        }
        swerve.setDriveNeutralMode(NeutralMode.Brake);
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

        }
    }

    @Override
    public void onStop(double timestamp) {
        subsystems.stop();
    }

    private void twoControllerMode() {
        double swerveYInput = -driver.getLeftX();
        double swerveXInput = -driver.getLeftY();
        double swerveRotationInput = driver.getRightX() + (driver.leftBumper.isBeingPressed() ? 0.3 : 0.0);
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, Netlink.getBooleanValue("Slow Driving Enabled"));
        
        SmartDashboard.putNumber("Translation Scalar", new Translation2d(swerveXInput, swerveYInput).norm());

        if (driver.bButton.wasActivated())
            swerve.rotate(swerve.getHeading().rotateBy(Rotation2d.fromDegrees(90)).getDegrees());
        else if (driver.aButton.wasActivated()) 
            swerve.rotate(180);
        else if (driver.xButton.wasActivated())
            swerve.rotate(270);
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
        if(driver.leftTrigger.wasActivated()) {
            intake.setSpeed(0.7);
        } else if(driver.leftTrigger.wasReleased()) {
            intake.setSpeed(0);
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

        verticalElevator.acceptManualInput(-coDriver.getLeftY() * 0.25);
        wrist.acceptManualInput(-coDriver.getRightY());

        if (coDriver.xButton.wasActivated()) {
            verticalElevator.setPosition(25);
        }

        if (coDriver.aButton.wasActivated()) {
            verticalElevator.setPosition(2);
        }
    }

    private void oneControllerMode() {}
}
