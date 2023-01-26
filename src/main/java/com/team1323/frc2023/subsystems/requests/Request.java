package com.team1323.frc2023.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

public abstract class Request {

	public abstract void act();
	
	public boolean isFinished() {return true;}

	private final List<Prerequisite> prerequisites = new ArrayList<>();

	public Request withPrerequisites(Prerequisite... prereqs) {
		for(Prerequisite prereq : prereqs){
			prerequisites.add(prereq);
		}
		return this;
	}

	public Request withPrerequisite(Prerequisite prereq) {
		prerequisites.add(prereq);
		return this;
	}

	public boolean allowed() {
		return prerequisites.stream().allMatch(p -> p.met());
	}

}
