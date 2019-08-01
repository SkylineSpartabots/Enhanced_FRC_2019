/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.CargoToRocket;
import frc.robot.auto.modes.DoubleCargoShip;
import frc.robot.auto.modes.DoubleRocketShip;
import frc.robot.auto.modes.SingleCargoShip;
import frc.robot.auto.modes.StandStill;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class SmartDashboardInteractions {

    private static final AutoOption DEFAULT_MODE = AutoOption.DOUBLE_ROCKET_SHIP;

    public static final Alliance STANDARD_CARPET_SIDE = Alliance.BLUE;
    public static final Alliance NONSTANDARD_CARPET_SIDE = Alliance.RED;

    private SendableChooser<AutoOption> modeChooser;
    private SendableChooser<DriveStation> driveStationChooser;
    private SendableChooser<Alliance> allianceChooser;
    
    public void initWithDefaults() {
        modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.SINGLE_CLOSE_CARGO_SHIP.name, AutoOption.SINGLE_CLOSE_CARGO_SHIP);
        modeChooser.addOption(AutoOption.STAND_STILL.name, AutoOption.STAND_STILL);
        modeChooser.addOption(AutoOption.DOUBLE_CARGO_SHIP.name, AutoOption.DOUBLE_CARGO_SHIP);
        modeChooser.addOption(AutoOption.CLOSE_CARGO_TO_ROCKET.name, AutoOption.CLOSE_CARGO_TO_ROCKET);

        driveStationChooser = new SendableChooser<DriveStation>();
        driveStationChooser.setDefaultOption("Left", DriveStation.LEFT);
        driveStationChooser.addOption("Right", DriveStation.RIGHT);
        driveStationChooser.addOption("Center", DriveStation.CENTER);

        allianceChooser = new SendableChooser<Alliance>();
        allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
        allianceChooser.addOption("Red", Alliance.RED);

        SmartDashboard.putData("Auto Chooser", modeChooser);
        SmartDashboard.putData("Drive Station Chooser", driveStationChooser);
        SmartDashboard.putData("Alliance Chooser", allianceChooser);
    }

    public AutoModeBase getSelectedAutoMode() {
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
        DriveStation selectedSide = (DriveStation) driveStationChooser.getSelected();
        return createAutoMode(selectedOption, selectedSide);
    }

    public String getSelectedMode() {
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
        return selectedOption.name;
    }

    public Alliance getSelectedAlliance() {
        Alliance alliance = (Alliance) allianceChooser.getSelected();
        return alliance;
    }

    public AutoModeBase createAutoMode(AutoOption option, DriveStation side) {
        switch(option) {
            case DOUBLE_ROCKET_SHIP:
                return new DoubleRocketShip(side);
            case DOUBLE_CARGO_SHIP:
                return new DoubleCargoShip(side);
            case SINGLE_CLOSE_CARGO_SHIP:
                return new SingleCargoShip(side);
            case CLOSE_CARGO_TO_ROCKET:
                return new CargoToRocket(side);
            case STAND_STILL:
                return new StandStill();
            default:
                TelemetryUtil.print("unexpected auto mode", PrintStyle.WARNING);
                return new StandStill();
        }
    }

    public void output() {
        SmartDashboard.putString("Selected Auto Mode", getSelectedMode());
    }

    public enum Alliance {
        RED,
        BLUE;
    }

    public enum DriveStation {
        LEFT,
        RIGHT,
        CENTER;
    }

    public enum AutoOption {

        DOUBLE_ROCKET_SHIP("Double Rocket Ship"),
        CLOSE_CARGO_TO_ROCKET("One Close Cargoship, One Rocket"),
        DOUBLE_CARGO_SHIP("One Close Cargoship, One Side Cargoship"),
        SINGLE_CLOSE_CARGO_SHIP("Only One Close Cargoship"),
        STAND_STILL("No Auto");

        public final String name;
        private AutoOption(String name) {
            this.name = name;
        }
    }


    
}
