/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Add your docs here.
 */
public class CrashTracker {

    private static final UUID INSTANCE_KEY = UUID.randomUUID();

    public static void logRobotStartup() {
        log("robot startup", null);
    }

    public static void logRobotConstruction() {
        log("robot startup", null);
    }

    public static void logRobotInit() {
        log("robot init", null);
    }

    public static void logTeleopInit() {
        log("teleop init", null);
    }

    public static void logAutoInit() {
        log("auto init", null);
    }

    public static void logDisabledInit() {
        log("disabled init", null);
    }

    public static void logThrowableCrash(Throwable throwable) {
        log("Exception", throwable);
    }


    private static void log(String mark, Throwable nullableException) {
        
        try(PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracker.txt", true))) {
            writer.print(INSTANCE_KEY.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if(nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }
            } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
