// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023;

import java.util.Set;

import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeExecuter;
import com.team1323.frc2023.auto.SmartDashboardInteractions;
import com.team1323.frc2023.auto.modes.TestMode;
import com.team1323.frc2023.loops.LimelightProcessor;
import com.team1323.frc2023.loops.Looper;
import com.team1323.frc2023.loops.QuinticPathTransmitter;
import com.team1323.frc2023.loops.RobotStateEstimator;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Swerve;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private SubsystemManager subsystems;

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();

	private DriverControls driverControls;	

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driverControls = DriverControls.getInstance();
		
		subsystems = driverControls.getSubsystems();
		Swerve.getInstance().zeroSensors();

		Logger.clearLog();

		enabledLooper.register(driverControls);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		smartDashboardInteractions.initWithDefaults();

		Settings.initializeToggles();

		generator.generateTrajectories();

		AutoModeBase auto = new TestMode();
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));

		VoltageOut cr1 = new VoltageOut(12.0, true, false);
		ControlRequest cr2 = new VoltageOut(12.0, true, false);
		cr1.Output = 1.0;
		System.out.println("Are cr1 and cr2 equal? " + (cr1.equals(cr2) ? "true" : "false"));
	}

	@Override
	public void robotPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", DriverStation.isEnabled());
		SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			driverControls.setAutoMode(true);
			disabledLooper.stop();
			enabledLooper.start();

			SmartDashboard.putBoolean("Auto", true);

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			driverControls.setAutoMode(false);
			disabledLooper.stop();
			enabledLooper.start();
			SmartDashboard.putBoolean("Auto", false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			enabledLooper.outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			disabledLooper.outputToSmartDashboard();
			smartDashboardInteractions.output();
			Settings.update();
			Swerve.getInstance().zeroModuleAngles();
			//Wrist.getInstance().zeroPositionWithCounter();
			if (subsystems.haveEmergency()) {
				//leds.configLEDs(LEDColors.RED);
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

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
