package com.team1323.frc2023.subsystems.gyros;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.RobotBase;

public class Pigeon implements Gyro {
	private final PigeonIMU pigeon;
	private double rollOffset = 0.0;

	public static Gyro createRealOrSimulatedGyro(int deviceId) {
		return RobotBase.isReal() ? new Pigeon(deviceId) : new SimulatedGyro();
	}
    
	private Pigeon(int deviceId){
		pigeon = new PigeonIMU(deviceId);
	}
	
	public boolean isGood(PigeonIMU imu){
		return imu.getState() == PigeonState.Ready;
	}
	
	@Override
	public double getYaw(){
		return pigeon.getFusedHeading();
	}

	public double getRawPitch(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	@Override
	public double getPitch() {
		return getRawPitch();
	}

	public double getRawRoll(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	@Override
	public double getRoll() {
		return getRawRoll() - rollOffset;
	}

	@Override
	public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

	@Override
	public void resetRoll() {
		rollOffset = getRawRoll();
	}
	
	@Override
	public void setYaw(double angle){
		pigeon.setFusedHeading(angle * 64.0, 10);
		pigeon.setYaw(-angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}
}
