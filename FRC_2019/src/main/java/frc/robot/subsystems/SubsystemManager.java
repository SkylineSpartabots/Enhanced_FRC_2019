/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

/**
 * Add your docs here.
 */
public class SubsystemManager implements ILooper{

    private final List<Subsystem> allSubsystems;
    public List<Subsystem> getSubsystems() {
        return allSubsystems;
    }

    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        this.allSubsystems = allSubsystems;
    }

    public void outputToSmartDashboard() {
        allSubsystems.forEach((subsystem) -> subsystem.outputTelemetry());
    }

    public void writeToLog() {
        allSubsystems.forEach((subsystem) -> subsystem.writeToLog());
    }

    public void stop() {
        allSubsystems.forEach((subsystem) -> subsystem.stop());
    }

    public boolean hasEmergency() {
        boolean emergency = false;
        for(Subsystem subsystem : allSubsystems) {
            emergency |= subsystem.hasEmergency;
        }
        return emergency;
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for (Loop loop : mLoops) {
                loop.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for(Subsystem subsystem : allSubsystems) {
                subsystem.readPeriodicInputs();
            }

            for(Loop loop : mLoops) {
                loop.onLoop(timestamp);
            }

            for(Subsystem subsystem : allSubsystems) {
                subsystem.writePeriodicOutputs();
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            for(Loop loop : mLoops) {
                loop.onStop(timestamp);
            }
        }

    }


    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for(Subsystem subsystem : allSubsystems) {
                subsystem.readPeriodicInputs();
            }

            //for(Subsystem subsystem : allSubsystems) {
            //    subsystem.writePeriodicOutputs();
            //}
        }

        @Override
        public void onStop(double timestamp) {

        }

    }

    public void registerEnabledLoops(Looper enabledLooper) {
        allSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }


    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
	}
    
}
