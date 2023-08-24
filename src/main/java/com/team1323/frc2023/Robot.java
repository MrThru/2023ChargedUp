// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023;

import java.util.Set;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.team1323.frc2023.auto.SmartDashboardInteractions;
import com.team1323.frc2023.auto.routines.AutoRoutine;
import com.team1323.frc2023.auto.routines.HighLinkRoutine;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.loops.AutoLoop;
import com.team1323.frc2023.loops.QuinticPathTransmitter;
import com.team1323.frc2023.loops.SynchronousLooper;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.LEDs;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.lib.util.Netlink;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

	private SubsystemManager subsystems;

	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private AutoLoop autoLoop;
	private DriverControls driverControls;	

	private SynchronousLooper looper;

	public Robot() {
		super(Constants.kMainThreadDt);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoLoop = new AutoLoop(smartDashboardInteractions);
		driverControls = DriverControls.getInstance();
		subsystems = driverControls.getSubsystems();
		looper = new SynchronousLooper(subsystems);

		Swerve.getInstance().zeroSensors();

		looper.registerAutoLoop(autoLoop);
		looper.registerTeleopLoop(driverControls);
		looper.register(QuinticPathTransmitter.getInstance());

		smartDashboardInteractions.initWithDefaults();

		generator.generateTrajectories();

		AutoRoutine auto = new HighLinkRoutine(Quadrant.BOTTOM_LEFT, true);
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));
	}

	@Override
	public void robotPeriodic() {
		Netlink.getInstance().update();

		// If any of these values are used to alter robot behavior in the future, treat them as
		// inputs so that they can be replayed accurately from logs.
		Logger.getInstance().recordOutput("Enabled", DriverStation.isEnabled());
		Logger.getInstance().recordOutput("Match time", DriverStation.getMatchTime());
		Logger.getInstance().recordOutput("Battery Voltage", RobotController.getBatteryVoltage());
	}

	@Override
	public void autonomousInit() {
		Netlink.setBooleanValue("Subsystems Coast Mode", false);
		AllianceChooser.update();
		looper.startAuto(Timer.getFPGATimestamp());
		Swerve.getInstance().requireModuleConfiguration();
	}

	@Override
	public void teleopInit() {
		// TODO: Update AllianceChooser to support log replay
		AllianceChooser.update();
		looper.startTeleop(Timer.getFPGATimestamp());
	}

	@Override
	public void autonomousPeriodic() {
		looper.onAutoLoop(Timer.getFPGATimestamp());
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		looper.onTeleopLoop(Timer.getFPGATimestamp());
	}

	@Override
	public void disabledInit() {
		AllianceChooser.update();
		LEDs.getInstance().configLEDs(LEDs.getInstance().disabledLEDColorsMode);;;;;;;;;;;;;;
		looper.startDisabled(Timer.getFPGATimestamp());
	}

	@Override
	public void disabledPeriodic() {
		smartDashboardInteractions.output();
		// TODO: Move this to a Loop
		Swerve.getInstance().zeroModuleAngles();
		Shoulder.getInstance().setAbsolutePositionWithCounter();
		Wrist.getInstance().setAbsolutePositionWithCounter();
		CubeIntake.getInstance().setAbsolutePositionWithCounter();

		looper.onDisabledLoop(Timer.getFPGATimestamp());
	}

	public void printStackTrace() {
		Set<Thread> threads = Thread.getAllStackTraces().keySet();
	
		for (Thread thread : threads) {
		  System.out.println("Thread name: " + thread.getName());
		  System.out.println("Thread state: " + thread.getState());
	
		  StackTraceElement[] stackTraceElements = thread.getStackTrace();
		  for (StackTraceElement stackTraceElement : stackTraceElements) {
			System.out.println("\t" + stackTraceElement);
		  }
		  System.out.println("\n");
		}
	  }
}
