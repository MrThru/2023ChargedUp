package com.team1323.frc2023.auto.routines;

import com.team1323.frc2023.requests.LambdaRequest;
import com.team1323.frc2023.requests.Request;

public class StandStillRoutine extends AutoRoutine {
    @Override
    public Request getRoutine() {
        return new LambdaRequest(() -> System.out.println("Starting Stand Still Routine... Done!"));
    }
}
