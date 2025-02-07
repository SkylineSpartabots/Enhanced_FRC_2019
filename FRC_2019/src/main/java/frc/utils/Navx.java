/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class Navx {

    private static Navx instance = null;
    public static Navx getInstance() {
        if(instance == null)
            instance = new Navx();
        return instance;
    }

    private AHRS navx;

    private Navx() {
        navx = new AHRS(Port.kMXP);
    }

    public double getHeading() {
        return navx.getAngle();
    }

}
