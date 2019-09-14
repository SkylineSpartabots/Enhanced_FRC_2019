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
public class Ports {

    //Intake Subystem Ports
    public final static int INNER_INTAKE_MOTOR = 6;
    public final static int RIGHT_KEBAB = 8;
    public final static int LEFT_KEBAB = 11;
    public final static int KEBAB_SOLENOID = 2; 
    public final static int BEAM_BREAK = 0;

    //Hatch Mechanism Subsystem Ports
    public final static int FINGERS_SOLENOID = 7;//7
    public final static int SLIDER_SOLENOID = 6;
    public static final int JACKS_SOLENOID = 1;//1 
    public final static int HATCH_DISTANCE_SENSOR = 3;

    //Elevator Ports
    public final static int RIGHT_ELEVATOR_MOTOR = 7;
    public final static int LEFT_ELEVATOR_MOTOR = 14;
    public static final int ELEVATOR_LIMIT_SWITCH = 9;
    
    //Drivetrain Ports
	public static final int DRIVE_RIGHT_CENTER = 3;
	public static final int DRIVE_RIGHT_FRONT = 4;
	public static final int DRIVE_RIGHT_BACK = 2;
	public static final int DRIVE_LEFT_CENTER = 15;
	public static final int DRIVE_LEFT_FRONT = 12;
    public static final int DRIVE_LEFT_BACK = 13;
    
	//Climb Ports
	public static final int RIGHT_CLIMB_MOTOR = 7;
	public static final int LEFT_CLIMB_MOTOR = 10;
	public static final int VACUUM_MOTOR = 11;


}
