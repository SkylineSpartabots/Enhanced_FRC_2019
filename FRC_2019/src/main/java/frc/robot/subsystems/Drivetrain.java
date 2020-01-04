/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.RobotState;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.loops.ILooper;
import frc.robot.loops.Limelight;
import frc.robot.loops.Loop;
import frc.robot.paths.CurvedProfile;
import frc.robot.subsystems.requests.Request;
import frc.utils.DriveSignal;
import frc.utils.PIDController;
import frc.utils.TelemetryUtil;
import frc.utils.Util;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {

    private static Drivetrain instance = null;

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public static final double DRIVE_ENCODER_PPR = 1000;
    public static final double DESIRED_TARGET_AREA = 9;
    public static final double VISION_THRESHOLD = 0;

    private DriveControlState state;
    private PeriodicIO periodicIO;

    private PIDController visionDrivePID;
    private PIDController visionTurnPID;

    private PIDController leftCurvedPathController;
    private PIDController rightCurvedPathController;

    private LazyTalonSRX rightMaster, rightSlaveA, rightSlaveB;
    private LazyTalonSRX leftMaster, leftSlaveA, leftSlaveB;
    private List<LazyTalonSRX> motors, masters, slaves;

    private boolean isBrakeMode;

    private Drivetrain() {
        leftMaster = new LazyTalonSRX(Ports.DRIVE_LEFT_CENTER);
        leftSlaveA = new LazyTalonSRX(Ports.DRIVE_LEFT_FRONT);
        leftSlaveB = new LazyTalonSRX(Ports.DRIVE_LEFT_BACK);

        rightMaster = new LazyTalonSRX(Ports.DRIVE_RIGHT_CENTER);
        rightSlaveA = new LazyTalonSRX(Ports.DRIVE_RIGHT_FRONT);
        rightSlaveB = new LazyTalonSRX(Ports.DRIVE_RIGHT_BACK);

        motors = Arrays.asList(rightMaster, rightSlaveA, rightSlaveB, leftMaster, leftSlaveA, leftSlaveB);
        masters = Arrays.asList(rightMaster, leftMaster);
        slaves = Arrays.asList(rightSlaveA, rightSlaveB, leftSlaveA, leftSlaveB);

        leftSlaveA.set(ControlMode.Follower, Ports.DRIVE_LEFT_CENTER);
        leftSlaveB.set(ControlMode.Follower, Ports.DRIVE_LEFT_CENTER);
        rightSlaveA.set(ControlMode.Follower, Ports.DRIVE_RIGHT_CENTER);
        rightSlaveB.set(ControlMode.Follower, Ports.DRIVE_RIGHT_CENTER);

        leftMaster.setInverted(InvertType.None);
        rightMaster.setInverted(InvertType.InvertMotorOutput);
        slaves.forEach((s) -> s.setInverted(InvertType.FollowMaster));

        for (LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configNominalOutputForward(0 / 12.0);
            motor.configNominalOutputReverse(0 / 12.0);
            motor.configOpenloopRamp(0.0);
            motor.configClosedloopRamp(0.0);
        }

        for (LazyTalonSRX master : masters) {
            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            master.setSelectedSensorPosition(0, 0, 50);
            master.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
            master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
            master.configVelocityMeasurementWindow(32, 10);
            master.setSensorPhase(false);
        }


        setCurrentLimit(40);

        DoubleSupplier drivePIDSupplier = () -> RobotState.visionTarget.getTargetArea();
        visionDrivePID = new PIDController(0, 0, 0, 0.005, drivePIDSupplier);
        visionDrivePID.setDesiredValue(DESIRED_TARGET_AREA);
        visionDrivePID.setMinMaxOutput(-0.2, 0.45);
        visionDrivePID.reset();

        DoubleSupplier turnPIDSupplier = () -> RobotState.visionTarget.getXOffset();
        visionTurnPID = new PIDController(0, 0, 0, 1.0, turnPIDSupplier);
        visionTurnPID.setDesiredValue(0);
        visionTurnPID.setMinMaxOutput(-0.35, 0.35);
        visionTurnPID.reset();

        DoubleSupplier leftVelocity = () -> getLeftLinearVelocity();
        leftCurvedPathController = new PIDController(0.003, 0, 0, 0, leftVelocity);
        leftCurvedPathController.setMinMaxOutput(-1, 1);

        DoubleSupplier rightVelocity = () -> getRightLinearVelocity();
        rightCurvedPathController = new PIDController(0.003, 0, 0, 0, rightVelocity);
        rightCurvedPathController.setMinMaxOutput(-1, 1);

    }

    public void setCurrentLimit(int amps) {
        for (LazyTalonSRX motor : motors) {
            motor.configContinuousCurrentLimit(amps);
            motor.configPeakCurrentLimit(amps);
            motor.configPeakCurrentDuration(10);
            motor.enableCurrentLimit(true);
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.driveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSec(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.driveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecToRpm(double inches_per_sec) {
        return inchesToRotations(inches_per_sec) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    public boolean isBrakeMode() {
        return isBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (isBrakeMode != on) {
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            for (LazyTalonSRX motor : motors) {
                motor.setNeutralMode(mode);
            }
            isBrakeMode = on;
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (state != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            leftMaster.configNeutralDeadband(0.0, 0);
            rightMaster.configNeutralDeadband(0.0, 0);
            Limelight.getInstance().ledsOn(false);
            state = DriveControlState.OPEN_LOOP;
        }

        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
    }
 
    public synchronized void setVisionControl(DriveSignal signal) {
        setVisionControl();
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
    }

    public synchronized void setVisionControl() {
        if (state != DriveControlState.VISION) {
            setBrakeMode(false);
            leftMaster.configNeutralDeadband(0.0, 0);
            rightMaster.configNeutralDeadband(0.0, 0);

            visionTurnPID.setConstants(0.013, 0.0, 0.0);
            visionDrivePID.setConstants(0.05, 0, 0);
            Limelight.getInstance().ledsOn(true);
            visionDrivePID.reset();
            visionTurnPID.reset();
            state = DriveControlState.VISION;
            TelemetryUtil.print("Setting vision control", PrintStyle.ERROR);
        }
    }

    private void updateVisionTargetFollower() {
        if (state == DriveControlState.VISION) {
            if (RobotState.visionTarget.isTargetVisible()) {
                double turnSpeed = visionTurnPID.getOutput();
                double driveSpeed = visionDrivePID.getOutput() / (1 + Math.abs((0.03 * visionTurnPID.getError())));
                setVisionControl(new DriveSignal(driveSpeed - turnSpeed, driveSpeed + turnSpeed));
                TelemetryUtil.print("Updating vision", PrintStyle.ERROR);
                SmartDashboard.putNumber("PID Drive Speed", driveSpeed);
            } else {
                setVisionControl(DriveSignal.NEUTRAL);
            }

        } else {
            TelemetryUtil.print("Drive is not in a vision target following state", PrintStyle.ERROR);
        }

    }

    public boolean isDoneWithTargetFollowing() {
        if (state != DriveControlState.VISION) {
            return true;
        }
        return RobotState.visionTarget.hasReachedTarget();
    }

    public synchronized void setCurvedProfileControl(CurvedProfile curvedProfiles[]) {
        if (state != DriveControlState.CURVED_PROFILE_FOLLOWER) {
            setBrakeMode(true);
            leftMaster.configNeutralDeadband(0);
            rightMaster.configNeutralDeadband(0);

            Limelight.getInstance().ledsOn(false);
            Limelight.getInstance().setDriverMode();

            state = DriveControlState.CURVED_PROFILE_FOLLOWER;
        }

            periodicIO.curvedProfileIndex = 0;
            periodicIO.curvedProfiles = curvedProfiles;
            updateAndResetCurvedProfilePID();
        
    }

    private void updateAndResetCurvedProfilePID() {
        periodicIO.desired_left_distance = periodicIO.left_distance + 
            periodicIO.curvedProfiles[periodicIO.curvedProfileIndex].getLeftDistance();
        
        periodicIO.isLeftProfileForward = periodicIO.desired_left_distance < periodicIO.left_distance;
        periodicIO.isRightProfileForward = periodicIO.desired_right_distance < periodicIO.left_distance;

        periodicIO.desired_right_distance = periodicIO.right_distance + 
            periodicIO.curvedProfiles[periodicIO.curvedProfileIndex].getRightDistance();

        leftCurvedPathController.setDesiredValue(periodicIO.curvedProfiles[periodicIO.curvedProfileIndex].getDesiredLeftVelocity());
        rightCurvedPathController.setDesiredValue(periodicIO.curvedProfiles[periodicIO.curvedProfileIndex].getDesiredRightVelocity());
        leftCurvedPathController.reset();
        rightCurvedPathController.reset();
    }

    private boolean hasLeftReachedTarget() {
        return periodicIO.isLeftProfileForward ? periodicIO.desired_left_distance >= periodicIO.left_distance 
            : periodicIO.desired_left_distance <= periodicIO.left_distance;
    }

    private boolean hasRightReachedTarget() {
        return periodicIO.isRightProfileForward ? periodicIO.desired_right_distance >= periodicIO.right_distance 
            : periodicIO.desired_right_distance <= periodicIO.right_distance;
    }

    private void updateCurvedProfileFollower() {
        if (state == DriveControlState.CURVED_PROFILE_FOLLOWER) {
            if(hasLeftReachedTarget() || hasRightReachedTarget()) {
                if(periodicIO.curvedProfiles.length - 1 == periodicIO.curvedProfileIndex) {
                    setOpenLoop(DriveSignal.BRAKE);
                    return;
                }
                periodicIO.curvedProfileIndex++;
                updateAndResetCurvedProfilePID();
            }
            periodicIO.left_demand += leftCurvedPathController.getOutput();
            //periodicIO.left_demand = Util.boundToScope(-1, 1, periodicIO.left_demand);
            periodicIO.right_demand += rightCurvedPathController.getOutput();
            //periodicIO.right_demand = Util.boundToScope(-1, 1, periodicIO.right_demand);
        } else {
            TelemetryUtil.print("Drive is not in a curved profile following state", PrintStyle.ERROR);
        }
    }

    public synchronized void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, 50);
        rightMaster.setSelectedSensorPosition(0, 0, 50);
        periodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return periodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodicIO.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodicIO.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10 / DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.TRACK_WIDTH;
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drivetrain.this) {
                setOpenLoop(new DriveSignal(0.0, 0.0));
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drivetrain.this) {
                switch (state) {
                case OPEN_LOOP:
                    break;
                case VISION:
                    updateVisionTargetFollower();
                    break;
                case CURVED_PROFILE_FOLLOWER:
                    updateCurvedProfileFollower();
                    break;
                default:
                    TelemetryUtil.print("unexpected drive control state", PrintStyle.ERROR);
                    break;
                }
            }

        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.left_position_ticks = leftMaster.getSelectedSensorPosition(0);
        periodicIO.right_position_ticks = rightMaster.getSelectedSensorPosition(0);
        periodicIO.left_velocity_ticks_per_100ms = leftMaster.getSelectedSensorVelocity(0);
        periodicIO.right_velocity_ticks_per_100ms = rightMaster.getSelectedSensorVelocity(0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        leftMaster.set(ControlMode.PercentOutput, periodicIO.left_demand);
        rightMaster.set(ControlMode.PercentOutput, periodicIO.right_demand);
        SmartDashboard.putNumber("Left Demand", periodicIO.left_demand);
        SmartDashboard.putNumber("Right Demand", periodicIO.right_demand);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
    }

    @Override
    public void outputTelemetry() {
        if (Constants.showDebugOutput) {
            SmartDashboard.putNumber("Left Front Current", leftSlaveA.getMotorOutputPercent());
            SmartDashboard.putNumber("Left Center Current", leftMaster.getMotorOutputPercent());
            SmartDashboard.putNumber("Left Back Current", leftSlaveB.getMotorOutputPercent());
            SmartDashboard.putNumber("Right Front Current", rightSlaveA.getMotorOutputPercent());
            SmartDashboard.putNumber("Right Center Current", rightMaster.getMotorOutputPercent());
            SmartDashboard.putNumber("Right Back Current", rightSlaveB.getMotorOutputPercent());
        }

        SmartDashboard.putNumber("Right Encoder", rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Encoder", leftMaster.getSelectedSensorPosition());
    }

    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public Request openLoopRequest(DriveSignal signal) {
        return new Request() {

            @Override
            public void act() {
                setOpenLoop(signal);
            }
        };
    }

    public Request timeDriveRequest(DriveSignal driveSignal, double time) {
        return new Request() {
            double startTime = 0;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                setOpenLoop(driveSignal);
            }

            @Override
            public boolean isFinished() {
                return (Timer.getFPGATimestamp() - startTime) >= time;
            }
        };
    }

    public Request alignToTargetRequest() {
        return new Request() {

            @Override
            public void act() {
                setVisionControl();
            }

            @Override
            public boolean isFinished() {
                return isDoneWithTargetFollowing();
            }
        };
    }

    public Request visionRequest() {
        return new Request() {

            @Override
            public void act() {
                setVisionControl();
            }

        };
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    public enum DriveControlState {
        OPEN_LOOP, CURVED_PROFILE_FOLLOWER, VISION;
    }

    public static class PeriodicIO {
        // Inputs
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;

        // Outputs
        public double left_demand;
        public double right_demand;
        public double desired_left_distance;
        public double desired_right_distance;

        //Profile Data
        public CurvedProfile[] curvedProfiles;
        public int curvedProfileIndex = 0;
        public boolean isLeftProfileForward;
        public boolean isRightProfileForward;

    }

}
