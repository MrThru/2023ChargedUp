// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.io;



import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;

/** Because why not */
public class PS4 extends PS4Controller{
    private static final double PRESS_THRESHOLD = 0.2;
    private double DEAD_BAND = 0.25;
    private boolean rumbling = false;
    public ButtonCheck squareButton, crossButton, circleButton, triangleButton, leftBumper, rightBumper,
        leftTrigger, rightTrigger, shareButton, optionButton, leftCenterClick, rightCenterClick, psButton, touchpad,
        POV0, POV90, POV180, POV270; 
	List<ButtonCheck> buttons = Arrays.asList(squareButton, crossButton, circleButton, triangleButton, leftBumper, rightBumper,
	leftTrigger, rightTrigger, shareButton, optionButton, leftCenterClick, rightCenterClick, psButton, touchpad,
	POV0, POV90, POV180, POV270);

    public static final int SQUARE_BUTTON = 1;
    public static final int CROSS_BUTTON = 2;
    public static final int CIRCLE_BUTTON = 3;
    public static final int TRIANGLE_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER = -7;
    public static final int RIGHT_TRIGGER = -8;
    public static final int SHARE_BUTTON = 9;
    public static final int OPTION_BUTTON = 10;
    public static final int LEFT_CENTER_CLICK = 11;
    public static final int RIGHT_CENTER_CLICK = 12;
    public static final int PS_BUTTON = 13;
    public static final int TOUCHPAD = 14;
    public static final int POV_0 = -2;
    public static final int POV_90 = -3;
    public static final int POV_180 = -4;
    public static final int POV_270 = -5;



    public void setDeadband(double deadband) {
        DEAD_BAND = deadband;
    }

    public PS4(int usb) {
        super(usb);
        squareButton = new ButtonCheck(SQUARE_BUTTON);
        crossButton = new ButtonCheck(CROSS_BUTTON);
        circleButton = new ButtonCheck(CIRCLE_BUTTON);
        triangleButton = new ButtonCheck(TRIANGLE_BUTTON);
        leftBumper = new ButtonCheck(LEFT_BUMPER);
        rightBumper = new ButtonCheck(RIGHT_BUMPER);
        leftTrigger = new ButtonCheck(LEFT_TRIGGER);
        rightTrigger = new ButtonCheck(RIGHT_TRIGGER);
        shareButton = new ButtonCheck(SHARE_BUTTON);
        optionButton = new ButtonCheck(OPTION_BUTTON);
        leftCenterClick = new ButtonCheck(LEFT_CENTER_CLICK);
        rightCenterClick = new ButtonCheck(RIGHT_CENTER_CLICK);
        psButton = new ButtonCheck(PS_BUTTON);
        touchpad = new ButtonCheck(TOUCHPAD);
        POV0 = new ButtonCheck(POV_0);
        POV90 = new ButtonCheck(POV_90);
        POV180 = new ButtonCheck(POV_180);
        POV270 = new ButtonCheck(POV_270);
    }
    @Override
    public double getRightX() {
        return Util.deadBand(getRawAxis(2), DEAD_BAND);
    }
	@Override
	public double getLeftX() {
		return Util.deadBand(getRawAxis(0), DEAD_BAND);
	}
    @Override
    public double getRightY() {
    	return Util.deadBand(getRawAxis(5), DEAD_BAND);
    }
	@Override
	public double getLeftY() {
		return Util.deadBand(getRawAxis(1), DEAD_BAND);
	}
    
    public double getRightTriggerAxis() {
        return Util.deadBand(getRawAxis(4), PRESS_THRESHOLD);
    }
    
	public double getLeftTriggerAxis() {
		return Util.deadBand(getRawAxis(3), PRESS_THRESHOLD);
	}

    public void rumble(double rumblesPerSecond, double numberOfSeconds){
    	if(!rumbling){
    		RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
    		r.start();
    	}
    }
    public boolean isRumbling(){
    	return rumbling;
    }
    public class RumbleThread extends Thread{
    	public double rumblesPerSec = 1;
    	public long interval = 500;
    	public double seconds = 1;
    	public double startTime = 0;
    	public RumbleThread(double rumblesPerSecond, double numberOfSeconds){
    		rumblesPerSec = rumblesPerSecond;
    		seconds = numberOfSeconds;
    		interval =(long) (1/(rumblesPerSec*2)*1000);
    	}
    	public void run(){
    		rumbling = true;
    		startTime = Timer.getFPGATimestamp();
    		try{
    			while((Timer.getFPGATimestamp() - startTime) < seconds){
		    		setRumble(RumbleType.kLeftRumble, 1);
		    		setRumble(RumbleType.kRightRumble, 1);
		    		sleep(interval);
		    		setRumble(RumbleType.kLeftRumble, 0);
		    		setRumble(RumbleType.kRightRumble, 0);
		    		sleep(interval);
    			}
    		}catch (InterruptedException e) {
				rumbling = false;
				e.printStackTrace();
			}
    		rumbling = false;
    	}
    }


    public class ButtonCheck{
    	boolean buttonCheck = false;
		boolean buttonActive = false;
		boolean activationReported = false;
    	boolean longPressed = false;
    	boolean longPressActivated = false;
    	boolean hasBeenPressed = false;
    	boolean longReleased = false;
		private double buttonStartTime = 0;
		private double longPressDuration = 0.25;
		public void setLongPressDuration(double seconds){
			longPressDuration = seconds;
		}
    	private int buttonNumber;
    	
    	public ButtonCheck(int id){
    		buttonNumber = id;
    	}
    	public void update(){
    		if(buttonNumber > 0){
    			buttonCheck = getRawButton(buttonNumber);
    		}else{
    			switch(buttonNumber){
    				case LEFT_TRIGGER:
    					buttonCheck = getLeftTriggerAxis() > 0;
    					break;
    				case RIGHT_TRIGGER:
    					buttonCheck = getRightTriggerAxis() > 0;
    					break;
    				case POV_0:
    					buttonCheck = (getPOV() == 0);
    					break;
    				case POV_90:
    					buttonCheck = (getPOV() == 90);
    					break;
    				case POV_180:
    					buttonCheck = (getPOV() == 180);
    					break;
    				case POV_270:
    					buttonCheck = (getPOV() == 270);
    					break;
    				default:
    					buttonCheck = false;
    					break;
    			}
    		}
    		if(buttonCheck){
	    		if(buttonActive){
	    			if(((Timer.getFPGATimestamp() - buttonStartTime) > longPressDuration) && !longPressActivated){
	    				longPressActivated = true;
						longPressed = true;
						longReleased = false;
	    			}
	    		}else{
					buttonActive = true;
					activationReported = false;
	    			buttonStartTime = Timer.getFPGATimestamp();
	    		}
    		}else{
    			if(buttonActive){
					buttonActive = false;
					activationReported = true;
    				if(longPressActivated){
    					hasBeenPressed = false;
    					longPressActivated = false;
    					longPressed = false;
    					longReleased = true;
    				}else{
						hasBeenPressed = true;
    				}
    			}
    		}
		}

		/** Returns true once the button is pressed, regardless of
		 *  the activation duration. Only returns true one time per
		 *  button press, and is reset upon release.
		 */
		public boolean wasActivated(){
			if(buttonActive && !activationReported){
				activationReported = true;
				return true;
			}
			return false;
		}
		
		/** Returns true once the button is released after being
		 *  held for 0.25 seconds or less. Only returns true one time
		 *  per button press.
		 */
    	public boolean shortReleased(){
    		if(hasBeenPressed){
    			hasBeenPressed = false;
    			return true;
    		}
    		return false;
		}
		
		/** Returns true once if the button is pressed for more than 0.25 seconds.
		 *  Only true while the button is still depressed; it becomes false once the 
		 *  button is released.
		 */
    	public boolean longPressed(){
    		if(longPressed){
    			longPressed = false;
    			return true;
    		}
    		return false;
		}
		
		/** Returns true one time once the button is released after being held for
		 *  more than 0.25 seconds.
		 */
    	public boolean longReleased(){
    		if(longReleased){
    			longReleased = false;
    			return true;
    		}
    		return false;
		}

		/** Returns true once the button is released, regardless of activation duration. */
		public boolean wasReleased(){
			return shortReleased() || longReleased();
		}
		
		/** Returns true if the button is currently being pressed. */
    	public boolean isBeingPressed(){
    		return buttonActive;
    	}
    }
	public void update() {
		squareButton.update();
		circleButton.update();
		triangleButton.update();
		crossButton.update();
		shareButton.update();
		optionButton.update();
		leftBumper.update();
		rightBumper.update();
		leftCenterClick.update();
		rightCenterClick.update();
		leftTrigger.update();
		rightTrigger.update();
		POV0.update();
		POV90.update();
		POV180.update();
		POV270.update();
	}
}
