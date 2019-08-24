/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.trajectory.Trajectory;
import frc.team254.lib.trajectory.TrajectoryUtil;
import frc.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.team254.lib.trajectory.timing.TimedState;
import frc.team254.lib.trajectory.timing.TimingConstraint;
import frc.team254.planners.DriveMotionPlanner;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class TrajectoryGenerator {

    private static TrajectoryGenerator instance = null;
    public static TrajectoryGenerator getInstance() {
        if(instance == null) {
            instance = new TrajectoryGenerator();
        }
        return instance;
    }

    private static final double maxVelocity = 100;
    private static final double maxAcceleration = 100;
    private static final double maxCentripetalAcceleration = 80;
    private static final double maxVoltage = 9.0;
    private static final double visionTransitionVelocity = 50;

    private final DriveMotionPlanner planner;
    private TrajectorySet trajectorySet = null;

    private TrajectoryGenerator() {
        planner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(trajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            TelemetryUtil.print("generating trajectories", PrintStyle.INFO);
            trajectorySet = new TrajectorySet();
            TelemetryUtil.print("finished generating trajectories in " + (Timer.getFPGATimestamp() - startTime) 
                + " seconds", PrintStyle.INFO);
        }
    }

    public TrajectorySet getTrajectorySet() {
        return trajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory (
        boolean reversed,
        final List<Pose2d> waypoints,
        final List<TimingConstraint<Pose2dWithCurvature>> constraints,
        double max_vel,
        double max_accel,
        double max_voltage) {
            return planner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory (
        boolean reversed,
        final List<Pose2d> waypoints,
        final List<TimingConstraint<Pose2dWithCurvature>> constraints,
        double start_vel,
        double end_vel,
        double max_vel,
        double max_accel,
        double max_voltage) {
            return planner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }



    public static final Pose2d startPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    

    /**
     * Robot poses relative to start position of drive base corner meeting hab corner
     * Point used on vannaka graphic interface: (84, 112, 0)
     * 
     * All poses are written for right side auto, values are mirrored across the x axis for 
     * left side auto using TrajectoryUtil
     */

    //Close side rocket
    public static final Pose2d closeRocketPose_Side = new Pose2d(new Translation2d(121.0, -89.0), Rotation2d.fromDegrees(-30));
    //Hatch Depot
    public static final Pose2d hatchDepotPose_Side = new Pose2d(new Translation2d(-72.0, -87.0), Rotation2d.fromDegrees(-180));
    //Far side Rocket
    public static final Pose2d farRocketPose_Side = new Pose2d(new Translation2d(168.0, -89.0), Rotation2d.fromDegrees(-150));
    //Near cargoship nose cone
    public static final Pose2d closeNoseConePose_Side = new Pose2d(new Translation2d(128, 38.0), Rotation2d.fromDegrees(0));
    //Near cargoship side
    public static final Pose2d nearSideCargoShipPose_Side = new Pose2d(new Translation2d(176.0, 16.0), Rotation2d.fromDegrees(90));
    //Far cargoship side
    public static final Pose2d farSideCargoShipPose_Side = new Pose2d(new Translation2d(198.0, 16.0), Rotation2d.fromDegrees(90));

    /**
     * Robot poses relative to start position of the center of robot aligned with center of hab with drive wheels
     * on very edge of hab
     * 
     * All poses written for any possible center auto, reflection should be false for all center auto trajectories
     */

    

     

    public class TrajectorySet {
        public class MirroredTrajectory {
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;

            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

        }

        public final MirroredTrajectory startToNearRocket_Side;
        public final MirroredTrajectory nearRocketToHatchDepot_Side;
        public final MirroredTrajectory hatchDepotToFarRocket_Side;
        public final MirroredTrajectory farRocketToFarSideCargoship_Side;
        public final MirroredTrajectory startToNoseCargoShip_Side;
        public final MirroredTrajectory noseCargoShipToHatchDepot_Side;
        public final MirroredTrajectory hatchDepotToNearSideCargoShip_Side;
        public final MirroredTrajectory hatchDepotToNearRocket_Side;

        private TrajectorySet() {
            startToNearRocket_Side = new MirroredTrajectory(getStartToNearRocket_Side());
            nearRocketToHatchDepot_Side = new MirroredTrajectory(getNearRocketToHatchDepot_Side());
            hatchDepotToFarRocket_Side = new MirroredTrajectory(getHatchDepotToFarRocket_Side());
            farRocketToFarSideCargoship_Side = new MirroredTrajectory(getFarRocketToFarSideCargoShip_Side());
            startToNoseCargoShip_Side = new MirroredTrajectory(getStartToNoseCargo_Side());
            noseCargoShipToHatchDepot_Side = new MirroredTrajectory(getNoseCargoShipToHatchDepot_Side());
            hatchDepotToNearSideCargoShip_Side = new MirroredTrajectory(getHatchDepotToNearSideCargoShip_Side());
            hatchDepotToNearRocket_Side = new MirroredTrajectory(getHatchDepotToNearRocket_Side());
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToNearRocket_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startPose);
            waypoints.add(closeRocketPose_Side);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearRocketToHatchDepot_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHatchDepotToFarRocket_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarRocketToFarSideCargoShip_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             maxVelocity, maxAcceleration, maxVoltage);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToNoseCargo_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNoseCargoShipToHatchDepot_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHatchDepotToNearSideCargoShip_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHatchDepotToNearRocket_Side() {
            List<Pose2d> waypoints = new ArrayList<>();
            //TODO
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(maxCentripetalAcceleration)),
             0, visionTransitionVelocity, maxVelocity, maxAcceleration, maxVoltage);
        }


    


        

        
        


    }
    


}
