/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utils.CrashTrackingRunnable;

/**
 * Add your docs here.
 */
public class Looper implements ILooper {
    public final double kPeriod = Constants.kLooperDt;

    private boolean running;
    
    private final Notifier notifier;
    private final List<Loop> loops;
    private final Object taskRunningLock = new Object();
    private double timestamp = 0;
    private double dt = 0;

    public double getDt() {
        return dt;
    }

    private final CrashTrackingRunnable runnable = new CrashTrackingRunnable(){
    
        @Override
        public void runCrashTracked() {
            synchronized (taskRunningLock) {
                if(running) {
                    double now = Timer.getFPGATimestamp();

                    for(Loop loop : loops) {
                        loop.onLoop(now);
                    }

                    dt = now - timestamp;
                    timestamp = now;
                }
            }
        }
    };

    public Looper() {
        notifier = new Notifier(runnable);
        running = false;
        loops = new ArrayList<>();
    }


    @Override
    public synchronized void register(Loop loop) {
       synchronized (taskRunningLock) {
           loops.add(loop);
       }
    }

    public synchronized void start() {
        if(!running) {
            System.out.println("Starting loops");
            synchronized (taskRunningLock) {
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : loops) {
                    loop.onStart(timestamp);
                }
                running = true;
            }
            notifier.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if(running) {
            System.out.println("Stopping loops");
            notifier.stop();
            synchronized (taskRunningLock) {
                running = false;
                timestamp = Timer.getFPGATimestamp();
                for(Loop loop : loops) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(timestamp);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Looper Duty Time", dt);
    }
}
