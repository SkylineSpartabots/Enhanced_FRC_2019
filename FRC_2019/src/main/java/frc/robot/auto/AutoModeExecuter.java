/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.utils.CrashTrackingRunnable;

/**
 * Add your docs here.
 */
public class AutoModeExecuter {
    private AutoModeBase autoMode;
    private Thread thread = null;

    public void setAutoMode(AutoModeBase autoMode) {
        this.autoMode = autoMode;
    }

    public void start() {
        if(thread == null) {
            thread = new Thread(new CrashTrackingRunnable(){
            
                @Override
                public void runCrashTracked() {
                    if(autoMode != null) {
                        autoMode.run();
                    }
                }
            });

            thread.start();
        }
    }

    public void stop() {
        if(autoMode != null) {
            autoMode.stop();
        }
        thread = null;
    }

}
