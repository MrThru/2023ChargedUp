package com.team1323.frc2023.auto;

import com.team1323.frc2023.auto.modes.StandStillMode;
import com.team1323.frc2023.auto.modes.TwoHighPieceAndRampMode;
import com.team1323.frc2023.auto.modes.HighLinkMode;
import com.team1323.frc2023.auto.modes.MidLinkMode;
import com.team1323.frc2023.auto.modes.TwoMidPieceAndRampMode;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.AutoZones.StartingSide;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String SELECTED_SIDE = "selected_side";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.TWO_MID_AND_RAMP;
    private static final StartingSide DEFAULT_SIDE = StartingSide.LEFT;

    private SendableChooser<AutoOption> modeChooser;
    private SendableChooser<StartingSide> sideChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.TWO_HIGH_AND_RAMP.name, AutoOption.TWO_HIGH_AND_RAMP);
        modeChooser.addOption(AutoOption.HIGH_MID_AND_RAMP.name, AutoOption.HIGH_MID_AND_RAMP);
        modeChooser.addOption(AutoOption.THREE_MID.name, AutoOption.THREE_MID);
        modeChooser.addOption(AutoOption.TWO_HIGH_ONE_MID.name, AutoOption.TWO_HIGH_ONE_MID);
        modeChooser.addOption(AutoOption.ONE_HIGH_TWO_MID.name, AutoOption.ONE_HIGH_TWO_MID);
        modeChooser.addOption(AutoOption.STAND_STILL.name, AutoOption.STAND_STILL);

        sideChooser = new SendableChooser<StartingSide>();
        sideChooser.setDefaultOption(DEFAULT_SIDE.toString(), DEFAULT_SIDE);
        sideChooser.addOption(StartingSide.RIGHT.toString(), StartingSide.RIGHT);

        SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        SmartDashboard.putData("Side Chooser", sideChooser);
        SmartDashboard.putString(SELECTED_SIDE, DEFAULT_SIDE.toString());
    }
    
    public AutoModeBase getSelectedAutoMode() {
        AutoOption selectedOption = getSelectedAutoModeEnum();
        StartingSide selectedSide = getSelectedStartingSide();
        Quadrant quadrant = getQuadrant(AllianceChooser.getAlliance(), selectedSide);
        return createAutoMode(selectedOption, quadrant);
    }

    private StartingSide getSelectedStartingSide() {
        StartingSide selectedSide = (StartingSide) sideChooser.getSelected();

        if (selectedSide == null) {
            return DEFAULT_SIDE;
        }

        return selectedSide;
    }

    private Quadrant getQuadrant(Alliance alliance, StartingSide startingSide) {
        if (alliance == Alliance.Blue) {
            if (startingSide == StartingSide.RIGHT) {
                return Quadrant.BOTTOM_LEFT;
            } else {
                return Quadrant.TOP_LEFT;
            }
        } else {
            if (startingSide == StartingSide.RIGHT) {
                return Quadrant.TOP_RIGHT;
            } else {
                return Quadrant.BOTTOM_RIGHT;
            }
        }
    }
    
    public AutoOption getSelectedAutoModeEnum() {
    	AutoOption option = (AutoOption) modeChooser.getSelected();

        if (option == null) {
            return AutoOption.STAND_STILL;
        }

    	return option;
    }

    enum AutoOption{
        STAND_STILL("Stand Still"), THREE_MID("3 Mid"), TWO_MID_AND_RAMP("2 Mid + Ramp"),
        TWO_HIGH_ONE_MID("2 High + 1 Mid"), TWO_HIGH_AND_RAMP("2 High + Ramp"),
        ONE_HIGH_TWO_MID("1 High + 2 Mid"), HIGH_MID_AND_RAMP("1 High + 1 Mid + Ramp");

    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option, Quadrant quadrant){
    	switch(option){
            case STAND_STILL:
                return new StandStillMode();
            case THREE_MID:
                return new MidLinkMode(quadrant);
            case TWO_MID_AND_RAMP:
                return new TwoMidPieceAndRampMode(quadrant);
            case TWO_HIGH_ONE_MID:
                return new HighLinkMode(quadrant, true);
            case TWO_HIGH_AND_RAMP:
                return new TwoHighPieceAndRampMode(quadrant, true);
            case ONE_HIGH_TWO_MID:
                return new HighLinkMode(quadrant, false);
            case HIGH_MID_AND_RAMP:
                return new TwoHighPieceAndRampMode(quadrant, false);
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedAutoModeEnum().name);
        SmartDashboard.putString(SELECTED_SIDE, getSelectedStartingSide().toString());
    }
}
