package com.team1323.frc2023.auto;

import com.team1323.frc2023.auto.modes.StandStillMode;
import com.team1323.frc2023.auto.modes.TwoConesAndRampMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String SELECTED_SIDE = "selected_side";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.STAND_STILL;
    private static final StartingSide DEFAULT_SIDE = StartingSide.LEFT;

    private SendableChooser<AutoOption> modeChooser;
    private SendableChooser<StartingSide> sideChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.TWO_CONES_AND_RAMP.name, AutoOption.TWO_CONES_AND_RAMP);

        sideChooser = new SendableChooser<StartingSide>();
        sideChooser.setDefaultOption(DEFAULT_SIDE.toString(), DEFAULT_SIDE);
        sideChooser.addOption(StartingSide.RIGHT.toString(), StartingSide.RIGHT);

        SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        SmartDashboard.putData("Side Chooser", sideChooser);
        SmartDashboard.putString(SELECTED_SIDE, DEFAULT_SIDE.toString());
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
        StartingSide selectedSide = getSelectedStartingSide();
        return createAutoMode(selectedOption, selectedSide);
    }

    private StartingSide getSelectedStartingSide() {
        StartingSide selectedSide = (StartingSide) sideChooser.getSelected();
        return selectedSide;
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    enum AutoOption{
        STAND_STILL("Stand Still"), TWO_CONES_AND_RAMP("Two Cones and Ramp");

    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    public enum StartingSide {
        LEFT, RIGHT;
    }

    private AutoModeBase createAutoMode(AutoOption option, StartingSide startingSide){
    	switch(option){
            case STAND_STILL:
                return new StandStillMode();
            case TWO_CONES_AND_RAMP:
                return new TwoConesAndRampMode(startingSide);
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
        SmartDashboard.putString(SELECTED_SIDE, getSelectedStartingSide().toString());
    }
}
