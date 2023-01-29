package com.team1323.frc2023.subsystems.gyros;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team1323.frc2023.Ports;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.RobotBase;

public class Pigeon extends Gyro{
	private static Pigeon instance = null;
	public static Pigeon getInstance(){
		if(instance == null){
			instance = new Pigeon();
		}
		return instance;
	}
	
	private PigeonIMU pigeon;
    
	private Pigeon(){
		try{
			pigeon = new PigeonIMU(Ports.PIGEON);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	
	public boolean isGood(PigeonIMU imu){
		return imu.getState() == PigeonState.Ready;
	}
	
	public Rotation2d getYaw(){
		if(RobotBase.isReal()){
			return Rotation2d.fromDegrees(pigeon.getFusedHeading());
		}
		return new Rotation2d();
	}

	public double getRawPitch(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}
	public Rotation2d getPitch() {
		return Rotation2d.fromDegrees(getRawPitch());
	}
	public double getRawRoll(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}
	public Rotation2d getRoll() {
		return Rotation2d.fromDegrees(getRawRoll());
	}
	public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}
	
	public void setAngle(double angle){
		pigeon.setFusedHeading(angle * 64.0, 10);
		pigeon.setYaw(-angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}
	
	public void outputToSmartDashboard(){
		/*SmartDashboard.putBoolean("Pigeon 1 Good", isGood(pigeon));
		SmartDashboard.putBoolean("Pigeon 2 Good", isGood(secondPigeon));
		SmartDashboard.putNumber("Pigeon 1 Yaw", pigeon.getFusedHeading());
		SmartDashboard.putNumber("Pigeon 2 Yaw", secondPigeon.getFusedHeading());*/
	}

	@Override
	public Rotation2d getAngle() {
		return getYaw();
	}
}
