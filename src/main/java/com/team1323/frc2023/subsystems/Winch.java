package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystem;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemInputsAutoLogged;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.drivers.SimulatedMotorController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Winch extends ServoSubsystem<ServoSubsystemInputsAutoLogged> {
    private static Winch instance = null;
    public static Winch getInstance() {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = new Winch(new Phoenix5FXMotorController(Constants.Winch.kConfig.leaderPortNumber, Constants.Winch.kConfig.canBus));
            } else {
                instance = new Winch(new SimulatedMotorController());
            }
        }
        return instance;
    }
    
    private Winch(MotorController leader) {
        super(leader, new ArrayList<>(), Constants.Winch.kConfig, new ServoSubsystemInputsAutoLogged());
        leader.setPIDF(Constants.Winch.kPIDF);
        setSupplyCurrentLimit(Constants.Winch.kSupplyCurrentLimit);
        zeroPosition();
        stop();
    }

    public Request angleRequest(double degrees) {
        return new Request() {
            @Override
            public void act() {
                setPosition(degrees);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Winch Angle", getPosition());
        SmartDashboard.putNumber("Winch Encoder Position", inputs.position);
    }
}
