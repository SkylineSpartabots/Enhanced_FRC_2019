/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Add your docs here.
 */
public class TelemetryUtil {

    private TelemetryUtil() {

    }

    public enum PrintStyle {
        INFO,
        WARNING,
        ERROR,
        NONE,
        SENSOR_VALUE
    };

    public static void print(Object messageObject, TelemetryUtil.PrintStyle style){
        String message = messageObject.toString();
        switch(style) {
            case INFO:
                System.out.println("######## " + message + " ########");
                break;
            case WARNING:
                System.out.println("<<<<<<<< Warning: " + message + " >>>>>>>>");
                DriverStation.reportWarning(message, false);
                break;
            case ERROR:
                System.out.println("!!!!!!!! Error: " + message + " !!!!!!!!");
                DriverStation.reportError(message, false);
                break;
            case SENSOR_VALUE:
                if(message.indexOf(".") >= 0){
                    String[] parts = message.split("(?<=.)");
                    if(parts[1].length() > 5) { message = parts[0]+ parts[1].substring(0, 4); }
                }
                System.out.println("||| Sensor Reading: " + message + " |||");
                break;
            default:
                System.out.println(message);
        }
    }
}
