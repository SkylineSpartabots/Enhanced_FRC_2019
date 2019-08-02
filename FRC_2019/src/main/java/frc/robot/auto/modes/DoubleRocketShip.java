/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.SmartDashboardInteractions.DriveStation;
import frc.robot.subsystems.Superstructure;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.trajectory.Trajectory;
import frc.team254.lib.trajectory.timing.TimedState;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class DoubleRocketShip extends AutoModeBase {

    private Superstructure s;
    final boolean isLeft;
    final double directionFactor;


    public DoubleRocketShip(DriveStation driveStation) {
        if(driveStation == DriveStation.CENTER)
            TelemetryUtil.print("a center auto does not exist for this mode", PrintStyle.WARNING);

        isLeft = driveStation == DriveStation.LEFT;

        directionFactor = isLeft ? -1.0 : 1.0;
    }

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.startToNearRocket_Side.get(isLeft), trajectories.nearRocketToHatchDepot_Side.get(isLeft),
            trajectories.hatchDepotToFarRocket_Side.get(isLeft), trajectories.farRocketToFarSideCargoship_Side.get(isLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        //TODO: Reset pose?
        
	}
}
