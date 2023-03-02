package com.team1323.frc2023.auto;

import com.team1323.frc2023.auto.modes.StandStillMode;
import com.team1323.frc2023.auto.modes.ThreeConesMode;
import com.team1323.frc2023.auto.modes.ThreeMidConesMode;
import com.team1323.frc2023.auto.modes.TwoConesAndRampMode;
import com.team1323.frc2023.auto.modes.TwoConesOneCubeMidMode;
import com.team1323.frc2023.auto.modes.TwoPieceAndRampMode;
import com.team1323.frc2023.field.AllianceChooser;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.field.AutoZones.StartingSide;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        modeChooser.addOption(AutoOption.TWO_CONES_ONE_CUBE.name, AutoOption.TWO_CONES_ONE_CUBE);
        modeChooser.addOption(AutoOption.TWO_PIECE_RAMP.name, AutoOption.TWO_PIECE_RAMP);

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
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    enum AutoOption{
        STAND_STILL("Stand Still"), TWO_CONES_ONE_CUBE("Two Cones One Cube"), TWO_PIECE_RAMP("Two Piece Ramp");

    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option, Quadrant quadrant){
    	switch(option){
            case STAND_STILL:
                return new StandStillMode();
            case TWO_CONES_ONE_CUBE:
                return new TwoConesOneCubeMidMode(quadrant);
            case TWO_PIECE_RAMP:
                return new TwoPieceAndRampMode(quadrant);
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
