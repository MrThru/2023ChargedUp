package com.team1323.frc2023.loops;

import org.littletonrobotics.junction.Logger;

import com.team1323.frc2023.auto.SmartDashboardInteractions;
import com.team1323.frc2023.auto.routines.AutoRoutine;
import com.team1323.frc2023.requests.EmptyRequest;
import com.team1323.frc2023.requests.RequestExecuter;

public class AutoLoop implements Loop {
    private final SmartDashboardInteractions smartDashboardInteractions;
    private final RequestExecuter requestExecuter = new RequestExecuter();

    public AutoLoop(SmartDashboardInteractions smartDashboardInteractions) {
        this.smartDashboardInteractions = smartDashboardInteractions;
    }

    @Override
    public void onStart(double timestamp) {
        long startTime = Logger.getInstance().getRealTimestamp();
        AutoRoutine selectedRoutine = smartDashboardInteractions.getSelectedAutoRoutine();
        requestExecuter.request(selectedRoutine.getRoutine());
        long endTime = Logger.getInstance().getRealTimestamp();
        System.out.println(String.format("Time to construct auto request: %d microseconds", endTime - startTime));
    }

    @Override
    public void onLoop(double timestamp) {
        requestExecuter.update();
    }

    @Override
    public void onStop(double timestamp) {
        if (!requestExecuter.isFinished()) {
            System.out.println("Auto mode ended early.");
        }
        requestExecuter.request(new EmptyRequest());
    }
}
