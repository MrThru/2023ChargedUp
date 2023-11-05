// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023;

import java.util.Set;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.team1323.frc2023.auto.SmartDashboardInteractions;
import com.team1323.frc2023.auto.routines.AutoRoutine;
import com.team1323.frc2023.auto.routines.HighLinkRoutine;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.ScoringPoses;
import com.team1323.frc2023.loops.AutoLoop;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.loops.QuinticPathTransmitter;
import com.team1323.frc2023.loops.SynchronousLooper;
import com.team1323.frc2023.requests.EmptyRequest;
import com.team1323.frc2023.subsystems.CubeIntake;
import com.team1323.frc2023.subsystems.LEDs;
import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.frc2023.subsystems.SubsystemManager;
import com.team1323.frc2023.subsystems.Wrist;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team1323.frc2023.vision.VisionPIDController.VisionPIDBuilder;
import com.team1323.lib.math.TwoPointRamp;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
	private final Loop zeroingLoop = new Loop() {
		@Override
		public void onStart(double timestamp){
		}

		@Override
		public void onLoop(double timestamp){
			Swerve.getInstance().zeroModuleAngles();
			Shoulder.getInstance().setAbsolutePositionWithCounter();
			Wrist.getInstance().setAbsolutePositionWithCounter();
			CubeIntake.getInstance().setAbsolutePositionWithCounter();
		}

		@Override
		public void onStop(double timestamp){
		}
	};

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
		RobotController.setBrownoutVoltage(6.1);

		Logger logger = Logger.getInstance();

		// Record metadata
		logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Set up data receivers & replay source
		if (RobotBase.isReal()) {
			logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
			logger.addDataReceiver(new NT4Publisher());
		} else if (Settings.kIsReplayingLog) {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog();
			logger.setReplaySource(new WPILOGReader(logPath));
			logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		} else {
			logger.addDataReceiver(new WPILOGWriter(""));
			logger.addDataReceiver(new NT4Publisher());
		}

		// Start AdvantageKit logger
		logger.start();

		// Disable Phoenix 6 signal logger
		SignalLogger.enableAutoLogging(false);

		autoLoop = new AutoLoop(smartDashboardInteractions);
		driverControls = DriverControls.getInstance();
		subsystems = driverControls.getSubsystems();
		looper = new SynchronousLooper(subsystems);

		looper.registerAutoLoop(autoLoop);
		looper.registerTeleopLoop(driverControls);
		looper.registerDisabledLoop(zeroingLoop);
		looper.register(QuinticPathTransmitter.getInstance());

		smartDashboardInteractions.initWithDefaults();

		generator.generateTrajectories();

		AutoRoutine auto = new HighLinkRoutine(Quadrant.BOTTOM_LEFT, true);
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));

		long startTime = Logger.getInstance().getRealTimestamp();
		smartDashboardInteractions.preGenerateAutoRoutines();
		long endTime = Logger.getInstance().getRealTimestamp();
		System.out.println(String.format("Autos took %d microseconds to generate", endTime - startTime));

		startTime = Logger.getInstance().getRealTimestamp();
		Superstructure.getInstance().cubeHighScoringSequence(
			ScoringPoses.getCenterScoringPose(Swerve.getInstance().getPose()), 
			new VisionPIDBuilder()
					.withDecelerationRamp(new TwoPointRamp(
						new Translation2d(1.0, 0.1),
						new Translation2d(60.0, 0.5),
						1.0,
						true
					))
					.build(),
			false
		);
		Superstructure.getInstance().request(new EmptyRequest());
		endTime = Logger.getInstance().getRealTimestamp();
		System.out.println(String.format("Cube scoring sequence took %d microseconds to generate", endTime - startTime));
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
