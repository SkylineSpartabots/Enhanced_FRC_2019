/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controllers.Xbox;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.loops.Limelight;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.HatchMechanism.State;
import frc.utils.CrashTracker;
import frc.utils.DriveControl;
import frc.utils.DriveSignal;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  private Drivetrain drive;
  private Elevator elevator;
  private Intake intake;
  private HatchMechanism hatchMech;

  private Superstructure s;
  private SubsystemManager subsystems;
  private RobotState robotState;
  private Limelight limelight;

  private AutoModeExecuter autoModeExecuter;
  private SmartDashboardInteractions smartDashboardInteractions;

  private Looper enabledLooper = new Looper();
  private Looper disabledLooper = new Looper();

  private Xbox driver, operator;
  private DriveControl driveControl;
  private boolean useOneController = false;

  @Override
  public void robotInit() {
    enabledLooper = new Looper();
    disabledLooper = new Looper();

    smartDashboardInteractions = new SmartDashboardInteractions();
    smartDashboardInteractions.initWithDefaults();

    drive = Drivetrain.getInstance();
    intake = Intake.getInstance();
    hatchMech = HatchMechanism.getInstance();
    elevator = elevator.getInstance();
    s = Superstructure.getInstance();
    robotState = RobotState.getInstance();
    subsystems = new SubsystemManager(Arrays.asList(drive, elevator, intake, hatchMech, s));
    limelight = limelight.getInstance();
    driveControl = new DriveControl();
    driver = new Xbox(0);
    operator = new Xbox(1);

    enabledLooper.register(Limelight.getInstance());
    disabledLooper.register(Limelight.getInstance());
    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLoops(disabledLooper);

    drive.zeroSensors();
    elevator.zeroSensors();

    SmartDashboardInteractions.updateOverrides();
    
    //limelight.ledsOn(false);
    //limelight.setDriverMode();
  }

  @Override
  public void robotPeriodic() {
    allPeriodic();
  }

  public void allPeriodic() {
    subsystems.outputToSmartDashboard();
    robotState.outputToSmartDashboard();
    enabledLooper.outputToSmartDashboard();
  }

  @Override
  public void autonomousInit() {
    try {
      drive.zeroSensors();
      elevator.zeroSensors();

      disabledLooper.stop();
      enabledLooper.start();

      /*SmartDashboard.putBoolean("Auto", true);

      autoModeExecuter = new AutoModeExecuter();
      autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
      autoModeExecuter.start();*/


    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
    try {
      driver.update();
      operator.update();

      if (useOneController)
        driveWithOneController();
      else
        driveWithTwoControllers();

      SmartDashboard.putBoolean("Robot Emergency", subsystems.hasEmergency());
      robotState.outputToSmartDashboard();
      allPeriodic();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
      disabledLooper.stop();
      enabledLooper.start();
      //SmartDashboard.putBoolean("Auto", false);

      //needs to be commented out
      drive.zeroSensors();
      elevator.zeroSensors();
      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  @Override
  public void teleopPeriodic() {
    try {
      driver.update();
      operator.update();

      if (useOneController)
        driveWithOneController();
      else {
        driveWithTwoControllers();
      }
        //driveWithTwoControllers();

      SmartDashboard.putBoolean("Robot Emergency", subsystems.hasEmergency());
      robotState.outputToSmartDashboard();
      allPeriodic();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }

  }

  @Override
  public void disabledInit() {
    enabledLooper.stop();
    disabledLooper.start();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  private void driveWithOneController() {

  }

  private boolean areKebabsDown = false;
  private boolean enteringVisionState = false;

  private void driveWithTwoControllers() {

    if (operator.backButton.isBeingPressed()) {

      if(enteringVisionState) {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
        elevator.setOpenLoop(0);
      }
      enteringVisionState = false;

      if (limelight.canVision()) {
        if(RobotState.visionTarget.isTargetVisible()) {
          driver.rumble(0.5);
          operator.rumble(0.5);
        } else {
          driver.rumble(0.0);
          operator.rumble(0.0);
        }

        limelight.ledsOn(true);
        limelight.setVisionMode();
        if (operator.rightBumper.wasActivated()) {
          s.hatchRetrievingState();
        } else if (operator.aButton.wasActivated()) {
          s.deployingState(Superstructure.ElevatorHeights.FIRST_LEVEL);
        } else if (operator.xButton.wasActivated()) {
          s.deployingState(Superstructure.ElevatorHeights.SECOND_LEVEL);
        } else if (operator.yButton.wasActivated()) {
          s.deployingState(Superstructure.ElevatorHeights.THIRD_LEVEL);
        } else if (operator.bButton.wasActivated()) {
          s.deployingState(Superstructure.ElevatorHeights.CARGO_SHIP);
        }
        return;
      } else {
        TelemetryUtil.print("Normal VISION is DISABLED by OVERRIDES", PrintStyle.WARNING);
        if(operator.rightBumper.isBeingPressed()) {
          if(RobotState.visionTarget.isTargetVisible()) {
            driver.rumble(0.4);
            operator.rumble(0.4);
          } else {
            driver.rumble(0.0);
            operator.rumble(0.0);
          }
          s.alignToTarget();
          return;
        }
      }
    }

    if(!enteringVisionState) {
      enteringVisionState = true;
      s.clearRequests();
    }

    driver.rumble(0);
    operator.rumble(0);

    limelight.ledsOn(false);
    limelight.setDriverMode();

    if (operator.startButton.isBeingPressed()) {
      SmartDashboardInteractions.updateOverrides();
      SmartDashboard.putBoolean("Cargo Sensor Override", SmartDashboardInteractions.cargoSensorOverride.get());
      SmartDashboard.putBoolean("Limit Switch Override", SmartDashboardInteractions.elevatorLimitSwitchOverride.get());
      SmartDashboard.putBoolean("Antitip Override", SmartDashboardInteractions.antiTipOverride.get());
      SmartDashboard.putBoolean("Hatch Sensor Override", SmartDashboardInteractions.hatchSensorOverride.get());
      SmartDashboard.putBoolean("Elevator Encoder Override", SmartDashboardInteractions.elevatorEncoderOverride.get());
    }

    double driveThrottle = driver.getY(Hand.kLeft) * elevator.getAntiTipCoeffecient();
    double turn = driver.getX(Hand.kRight) * elevator.getAntiTipCoeffecient();
    double drivePercent = 0.7;
    double turnPercent = 0.4;
    double driveC = 3;
    double turnC = 1;
    drive.setOpenLoop(driveControl.arcadeDrive(driveThrottle * drivePercent, turn * turnPercent));

    boolean hasCargo = intake.hasCargo();
    boolean hasHatch = hatchMech.hasHatch();
    boolean elevatorUp = elevator.isElevatorUp();

    /**
     * The following series of nested if statements is the control logic behind
     * setting the appropriate state of the intake state machine as determined by
     * operator gamepad
     */

    if (operator.dpadUp.wasActivated() || elevatorUp || hasCargo || hasHatch) {
      areKebabsDown = false;
    } else if (operator.dpadDown.wasActivated()) {
      areKebabsDown = true;
    }

    if (areKebabsDown) {
      if (operator.rightTrigger.isBeingPressed()) {
        intake.conformToState(Intake.State.INTAKE_WITH_KEBABS);
      } else if (operator.leftTrigger.isBeingPressed()) {
        intake.conformToState(Intake.State.OUTAKE_WITH_KEBABS);
      } else {
        intake.conformToState(Intake.State.IDLE_WITH_KEBABS);
      }
    } else {
      if (hasHatch) {
        intake.conformToState(Intake.State.CARGO_PHOBIC);
      } else if (operator.rightTrigger.isBeingPressed()) {
        if (hasCargo) {
          if (elevatorUp) {
            intake.conformToState(Intake.State.INTAKE_ELEVATOR_UP);
          } else {
            intake.conformToState(Intake.State.HOLDING);
          }
        } else {
          intake.conformToState(Intake.State.INTAKE_WITHOUT_KEBABS);
        }
      } else if (operator.leftTrigger.isBeingPressed()) {
        if (hasCargo && elevatorUp) {
          intake.conformToState(Intake.State.OUTAKE_ELEVATOR_UP);
        } else {
          intake.conformToState(Intake.State.OUTAKE_WITHOUT_KEBABS);
        }
      } else {
        if (hasCargo && !elevatorUp) {
          intake.conformToState(Intake.State.IDLE_WITHOUT_KEBABS);
        } else {
          intake.conformToState(Intake.State.OFF);
        }
      }
    }

    /**
     * The following is the logic for controlling the state of the hatch mechnanism
     * state machine as determined by operator gamepad
     * 
     * Note: the "Recieving" state of the hatch mechanism automatically transfers to
     * stowed when the hatch limit switch is active. This is not an element of the
     * drive method because this function has utility in auto programs as well
     */

    if (hasCargo) {
      hatchMech.conformToState(HatchMechanism.State.STOWED);
    } else {
      if (driver.backButton.wasActivated()) {
        hatchMech.conformToState(HatchMechanism.State.STOWED);
      } else if (driver.leftBumper.wasActivated()) {
        hatchMech.conformToState(HatchMechanism.State.RECIEVING);
        //System.out.println("conforming to recieved");
      } else if (driver.rightBumper.wasActivated()) {
        hatchMech.conformToState(HatchMechanism.State.SCORING);
      } else if (hatchMech.getState() == HatchMechanism.State.STOWED
          || hatchMech.getState() == HatchMechanism.State.FINGERS_STOWED_EXTENDED) {
        if (operator.leftJoystickButton.isBeingPressed()) {
          hatchMech.conformToState(State.FINGERS_STOWED_EXTENDED);
        } else {
          hatchMech.conformToState(State.STOWED);
          //System.out.println("conforming to stowed");
        }
      }
    }

    /**
     * The following is the logic for controlling the state of the elevator state
     * machine as determined by operator gamepad
     */

    double manualControlJoystick = operator.getY(Hand.kRight);

    if (manualControlJoystick < -0.15) {
      manualControlJoystick = -0.15;
    }
    if (manualControlJoystick != 0) {
      elevator.setOpenLoop(manualControlJoystick);
    } else if (operator.aButton.wasActivated()) {
      elevator.setTargetHeight(Superstructure.ElevatorHeights.FIRST_LEVEL.getHeight());
    } else if (operator.xButton.wasActivated()) {
      elevator.setTargetHeight(Superstructure.ElevatorHeights.SECOND_LEVEL.getHeight());
    } else if (operator.yButton.wasActivated()) {
      elevator.setTargetHeight(Superstructure.ElevatorHeights.THIRD_LEVEL.getHeight());
    } else if (operator.bButton.wasActivated()) {
      elevator.setTargetHeight(Superstructure.ElevatorHeights.CARGO_SHIP.getHeight());
    } else if (operator.leftBumper.wasActivated()) {
      elevator.setTargetHeight(Superstructure.ElevatorHeights.DOWN.getHeight());
    } else if (elevator.isOpenLoop()) {
      elevator.lockHeight();
    }

  }

}
