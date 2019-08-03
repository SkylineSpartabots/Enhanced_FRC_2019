/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.trajectory.TimedView;
import frc.team254.lib.trajectory.Trajectory;
import frc.team254.lib.trajectory.TrajectoryIterator;
import frc.team254.lib.trajectory.timing.TimedState;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class DriveTrajectoryAction implements Action {
    private static RobotState robotState;
    private static Drivetrain drive;

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory;
    private final boolean resetPose;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        drive = Drivetrain.getInstance();
        robotState = RobotState.getInstance();
        this.trajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        this.resetPose = resetPose;
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }

    @Override
    public boolean isFinished() {
        if(drive.isDoneWithTrajectory()) {
            TelemetryUtil.print("Has finished trajectory", PrintStyle.NONE);
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        TelemetryUtil.print("Starting trajectory! (length=" + trajectory.getRemainingProgress() + ")", PrintStyle.NONE);
        if(resetPose) {
            robotState.reset(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
        }
        drive.setTrajectory(trajectory);
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

}
