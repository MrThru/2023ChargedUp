package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystem;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemInputsAutoLogged;
import com.team1323.lib.drivers.Phoenix5FXMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Winch extends ServoSubsystem<ServoSubsystemInputsAutoLogged> {
    private static Winch instance = null;
    public static Winch getInstance() {
        if (instance == null) {
            instance = new Winch();
        }
        return instance;
    }
    
    private Winch() {
        super(Phoenix5FXMotorController.createRealOrSimulatedController(Constants.Winch.kConfig.leaderPortNumber, Constants.Winch.kConfig.canBus), 
                new ArrayList<>(), Constants.Winch.kConfig, new ServoSubsystemInputsAutoLogged());
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
