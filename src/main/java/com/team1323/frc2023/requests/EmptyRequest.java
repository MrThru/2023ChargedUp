package com.team1323.frc2023.requests;

public class EmptyRequest extends Request{
	
	@Override
	public void act(){
		// empty, as the name suggests
	}
	
	@Override
	public boolean isFinished(){
		return true;
	}

	@Override
	public String toString() {
		return "EmptyRequest()";
	}
	
}
