/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    public final static double kLooperDt = 0.02;

	public final static int BEAM_BREAK_THRESHOLD = 50;
	public final static int HATCH_DISTANCE_THRESHOLD = 40;

    public final static boolean showDebugOutput = false;

    //Elevator constants
    public final static int elevatorEncoderStartingPosition = 0;
    public final static int maxElevatorHeight = 80;
    public final static int minElevatorHeight = 0;
    public final static int elevatorCurrentLimit = 40;
    public final static double elevatorHeightTolerance = 4;
    public final static double manualElevatorDriveProportion = 0.5;
    public final static double elevatorTicksPerInch = 58.75;

	public static final double maxElevatorInitialHeight = 0;

	public static final double minElevatorInitialHeight = 0;

	public static final double driveWheelDiameterInches = 0;

	public static final double kDriveKv = 0;

	public static final double kDriveWheelRadiusInches = 0;

	public static final double kRobotLinearInertia = 0;

	public static final double kDriveKa = 0;

	public static final double kDriveVIntercept = 0;

	public static final double kRobotAngularInertia = 0;

	public static final double kRobotAngularDrag = 0;

	public static final double kDriveWheelDiameterInches = 0;

	public static final double kDriveWheelTrackWidthInches = 0;

	public static final double kTrackScrubFactor = 0;

	public static final double kPathMinLookaheadDistance = 0;

	public static final double kPathLookaheadTime = 0;

	public static final double kPathKX = 0;

	public static final double kDriveVelocityKd = 0;

	public static final int CLIMB_ENCODER_STARTING_HEIGHT = 0;

	public static final double MAX_CLIMB_HEIGHT = 0;

	public static final double CLIMB_TICKS_PER_INCH = 0;

	public static final double MIN_CLIMB_HEIGHT = 0;

	public static final double MAX_DRIVE_VELOCITY = 0;
}
