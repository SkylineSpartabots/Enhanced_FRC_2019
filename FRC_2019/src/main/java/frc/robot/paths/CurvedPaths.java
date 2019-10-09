/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.paths;

/**
 * Add your docs here.
 */
public class CurvedPaths {

    private boolean isLeft;
    
    public CurvedPaths(boolean isLeft) {
        this.isLeft = isLeft;
    }

    public static CurvedProfile[] habSideToNearRocket = {
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft)
    };

    public CurvedProfile[] nearRocketToHatchDepot = {
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft)
    };

    public CurvedProfile[] hatchDepotToFarRocket = {
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft)
    };

    public CurvedProfile[] farRocketToSideCargoShip = {
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft),
        //new CurvedProfile(leftDistance, rightDistance, isLeft)
    };
}
