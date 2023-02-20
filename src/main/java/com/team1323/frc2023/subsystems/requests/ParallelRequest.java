/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023.subsystems.requests;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

/**
 * A Request which takes a list of Requests and executes them in parallel.
 */
public class ParallelRequest extends Request {
    private final Set<Request> idleRequests;
    private final Set<Request> inProgressRequests;

    public ParallelRequest(Request... requests) {
        idleRequests = new HashSet<>(Arrays.asList(requests));
        inProgressRequests = new HashSet<>();
    }

    @Override
    public void cleanup() {
        inProgressRequests.forEach(r -> r.cleanup());
        idleRequests.forEach(r -> r.cleanup());
        super.cleanup();
    }

    private void startRequestsIfAllowed() {
        for (Iterator<Request> iter = idleRequests.iterator(); iter.hasNext();) {
            Request request = iter.next();
            if (request.allowed()) {
                request.act();
                inProgressRequests.add(request);
                iter.remove();
            }
        }
    }

    @Override
    public void act() {
        startRequestsIfAllowed();
    }

    @Override
    public boolean isFinished() {
        startRequestsIfAllowed();
        inProgressRequests.removeIf(r -> r.isFinished());

        return idleRequests.isEmpty() && inProgressRequests.isEmpty();
    }
}
