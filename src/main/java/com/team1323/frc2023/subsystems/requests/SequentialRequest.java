/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * A Request which itself takes a list of Requests and executes them in series.
 */
public class SequentialRequest extends Request {
    private final List<Request> requests;
    private Request currentRequest = null;
    private boolean startedCurrentRequest = false;

    public SequentialRequest(Request... requests) {
        this.requests = new ArrayList<>();
        for (Request request : requests) {
            this.requests.add(request);
        }
    }

    private void startRequestIfAllowed() {
        if (currentRequest.allowed()) {
            currentRequest.act();
            startedCurrentRequest = true;
        }
    }

    @Override
    public void act() {
        currentRequest = requests.remove(0);
        startRequestIfAllowed();
    }

    @Override
    public boolean isFinished(){
        if (!startedCurrentRequest && currentRequest != null) {
            startRequestIfAllowed();
        }

        if (startedCurrentRequest && currentRequest.isFinished()) {
            if (requests.isEmpty()) {
                currentRequest = null;
                return true;
            } 

            currentRequest = requests.remove(0);
            startedCurrentRequest = false;
            startRequestIfAllowed();
        }

        return false;
    }
}
