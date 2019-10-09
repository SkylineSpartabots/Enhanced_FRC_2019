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

	public static final double MAX_CLIMB_HEIGHT = 0;

	public static final double CLIMB_TICKS_PER_INCH = 0;

	public static final double MIN_CLIMB_HEIGHT = 0;

	public static final double MAX_DRIVE_VELOCITY = 0;

	public static final double TRACK_WIDTH = 26.7;

	public static final double driveWheelDiameterInches = 0;
}
