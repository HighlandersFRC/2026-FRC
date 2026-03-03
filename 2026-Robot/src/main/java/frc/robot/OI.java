// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.
//hi om

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.tools.TriggerButton;

public class OI {
    public static SendableChooser<String> fieldSide = new SendableChooser<String>();
    public static SendableChooser<String> leftRight = new SendableChooser<String>();
    public static SendableChooser<String> auto = new SendableChooser<String>();
    public static XboxController driverController = new XboxController(0);
    public static XboxController operatorController = new XboxController(1);

    public static BooleanSupplier driveRTSupplier = () -> getDriverRTPercent() > Constants.OperatorConstants.RIGHT_TRIGGER_DEADZONE;
    public static BooleanSupplier driverLTSupplier = () -> getDriverLTPercent() > Constants.OperatorConstants.LEFT_TRIGGER_DEADZONE;

    public static BooleanSupplier povUp = () -> getPOV() == 0;
    public static BooleanSupplier povRight = () -> getPOV() == 90;
    public static BooleanSupplier povDown = () -> getPOV() == 180;
    public static BooleanSupplier povLeft = () -> getPOV() == 270;

    public static TriggerButton driverPOVUp = new TriggerButton(povUp);
    public static TriggerButton driverPOVRight = new TriggerButton(povRight);
    public static TriggerButton driverPOVDown = new TriggerButton(povDown);
    public static TriggerButton driverPOVLeft = new TriggerButton(povLeft);

    public static BooleanSupplier operatorPovUp = () -> getOperatorPOV() == 0;
    public static BooleanSupplier operatorPovRight = () -> getOperatorPOV() == 90;
    public static BooleanSupplier operatorPovDown = () -> getOperatorPOV() == 180;
    public static BooleanSupplier operatorPovLeft = () -> getOperatorPOV() == 270;

    public static TriggerButton operatorPOVUp = new TriggerButton(operatorPovUp);
    public static TriggerButton operatorPOVRight = new TriggerButton(operatorPovRight);
    public static TriggerButton operatorPOVDown = new TriggerButton(operatorPovDown);
    public static TriggerButton operatorPOVLeft = new TriggerButton(operatorPovLeft);

    public static TriggerButton driverRT = new TriggerButton(driveRTSupplier);
    public static TriggerButton driverLT = new TriggerButton(driverLTSupplier);

    public static JoystickButton driverA = new JoystickButton(driverController, 1);
    public static JoystickButton driverB = new JoystickButton(driverController, 2);

    public static JoystickButton driverY = new JoystickButton(driverController, 4);
    public static JoystickButton driverX = new JoystickButton(driverController, 3);

    public static JoystickButton driverRB = new JoystickButton(driverController, 6);
    public static JoystickButton driverLB = new JoystickButton(driverController, 5);

    public static JoystickButton driverLJ = new JoystickButton(driverController, 9);
    public static JoystickButton driverRJ = new JoystickButton(driverController, 10);

    public static JoystickButton operatorX = new JoystickButton(operatorController, 3);
    public static JoystickButton operatorB = new JoystickButton(operatorController, 2);

    public static JoystickButton operatorY = new JoystickButton(operatorController, 4);
    public static JoystickButton operatorA = new JoystickButton(operatorController, 1);

    public static BooleanSupplier operatorRTSupplier = () -> getOperatorRTPercent() > Constants.OperatorConstants.RIGHT_TRIGGER_DEADZONE;
    public static BooleanSupplier operatorLTSupplier = () -> getOperatorLTPercent() > Constants.OperatorConstants.LEFT_TRIGGER_DEADZONE;

    public static TriggerButton operatorRT = new TriggerButton(operatorRTSupplier);
    public static TriggerButton operatorLT = new TriggerButton(operatorLTSupplier);

    public static JoystickButton operatorRB = new JoystickButton(operatorController, 6);
    public static JoystickButton operatorLB = new JoystickButton(operatorController, 5);

    public static JoystickButton operatorLJ = new JoystickButton(operatorController, 9);
    public static JoystickButton operatorRJ = new JoystickButton(operatorController, 10);

    public static JoystickButton driverViewButton = new JoystickButton(driverController, 7);

    public static JoystickButton operatorViewButton = new JoystickButton(operatorController, 7);
    public static JoystickButton driverMenuButton = new JoystickButton(driverController, 8);

    public static JoystickButton operatorMenuButton = new JoystickButton(operatorController, 8);

    public static Joystick autoChooser = new Joystick(2);

    public static JoystickButton autoChooserIsBlue = new JoystickButton(autoChooser, 8);

    static {
        fieldSide.addOption("red", "red");
        fieldSide.addOption("blue", "blue");
        fieldSide.setDefaultOption("blue", "blue");
        SmartDashboard.putData("Field Side", fieldSide);
        leftRight.addOption("left", "left");
        leftRight.addOption("right", "right");
        leftRight.setDefaultOption("right", "right");
        SmartDashboard.putData("Left or Right", leftRight);
        for (String path : Constants.Autonomous.paths) {
            auto.addOption(path, path);
        }
        auto.setDefaultOption("None", "None");
        SmartDashboard.putData("Selected Auto", auto);
    }

    public static void printAutoChooserInputs() {
        java.util.logging.Logger.getGlobal().info("Driver Controller Connected: " + driverController.isConnected());
        java.util.logging.Logger.getGlobal().info("Operator Controller Connected: " + operatorController.isConnected());
        java.util.logging.Logger.getGlobal().info("Auto Chooser Connected: " + autoChooser.isConnected());
        java.util.logging.Logger.getGlobal().info("Auto Chooser Num Buttons: " + autoChooser.getButtonCount());
        java.util.logging.Logger.getGlobal().info("Is Blue: " + autoChooserIsBlue.getAsBoolean());
        for (int i = 1; i <= 16; i++) {
            java.util.logging.Logger.getGlobal().info("Auto Chooser Button " + i + " : " + autoChooser.getRawButton(i));
        }
    }

    public static String getSelectedPath() {
        return auto.getSelected();
    }

    public static double getDriverLeftX() {
        double leftX = -driverController.getLeftX(), leftY = driverController.getLeftY();
        if (Math.hypot(leftX, leftY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            leftX = 0;
        }
        return leftX;
    }

    public static double getDriverLeftY() {
        double leftX = -driverController.getLeftX(), leftY = driverController.getLeftY();
        if (Math.hypot(leftX, leftY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            leftY = 0;
        }
        return leftY;
    }

    public static double getDriverRightX() {
        double rightX = driverController.getRightX(), rightY = driverController.getRightY();
        if (Math.hypot(rightX, rightY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            rightX = 0;
        }
        return rightX;
    }

    public static double getDriverRightY() {
        double rightX = driverController.getRightX(), rightY = driverController.getRightY();
        if (Math.hypot(rightX, rightY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            rightY = 0;
        }
        return rightY;
    }

    public static double getOperatorLeftX() {
        double leftX = -operatorController.getLeftX(), leftY = operatorController.getLeftY();
        if (Math.hypot(leftX, leftY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            leftX = 0;
        }
        return leftX;
    }

    public static double getOperatorLeftY() {
        double leftX = -operatorController.getLeftX(), leftY = operatorController.getLeftY();
        if (Math.hypot(leftX, leftY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            leftY = 0;
        }
        return leftY;
    }

    public static double getOperatorRightX() {
        double rightX = operatorController.getRightX(), rightY = operatorController.getRightY();
        if (Math.hypot(rightX, rightY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            rightX = 0;
        }
        return rightX;
    }

    public static double getOperatorRightY() {
        double rightX = operatorController.getRightX(), rightY = operatorController.getRightY();
        if (Math.hypot(rightX, rightY) < Constants.OperatorConstants.LEFT_STICK_DEADZONE) {
            rightY = 0;
        }
        return rightY;
    }

    public static double getDriverRTPercent() {
        return driverController.getRightTriggerAxis();
    }

    // public static double getDriverLTPercent() {
    // return driverController.getLeftTriggerAxis();
    // }

    /**
     * This is for using the backup controller, the LT is 1.0 when it should be at
     * zero
     * Set the trigger depth on the controller to T2 (middle) for LT when using this
     * code
     */
    public static double getDriverLTPercent() {
        double raw = driverController.getLeftTriggerAxis();
        if (raw > 0.9) {
            raw = 0.0;
        }
        double refined = raw * (4.0 / 3.0);
        return refined;
    }

    public static boolean getDriverA() {
        return driverController.getAButton();
    }

    public static boolean getDriverRB() {
        return driverController.getRightBumperButton();
    }

    public static boolean getDriverLB() {
        return driverController.getLeftBumperButton();
    }

    public static double getOperatorRTPercent() {
        return operatorController.getRightTriggerAxis();
    }

    public static double getOperatorLTPercent() {
        return operatorController.getLeftTriggerAxis();
    }

    public static boolean getOperatorLB() {
        return operatorController.getLeftBumperButton();
    }

    public static int getPOV() {
        return driverController.getPOV();
    }

    public static int getOperatorPOV() {
        return operatorController.getPOV();
    }

    public static boolean getPOVUp() {
        if (getPOV() == 0) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean getPOVDown() {
        if (getPOV() == 180) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean isRedSide() {
        if (autoChooserConnected()) {
            return !autoChooser.getRawButton(8);
        } else if (DriverStation.isDSAttached()) {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        } else {
            return true;
        }
    }

    public static boolean isLeftSide() {
        return leftRight.getSelected().equals("left");
    }

    public static boolean isRecalculateMode() {
        return autoChooser.getRawButton(7);
    }

    public static boolean autoChooserConnected() {
        return autoChooser.isConnected();
    }

    public static boolean is4PieceFarBottom231Auto() {
        return autoChooser.getRawButton(2);
    }

    public static boolean is5PieceAuto() {
        return autoChooser.getRawButton(1);
    }

    public static boolean is3PieceFarBottomAuto() {
        return autoChooser.getRawButton(3);
    }

    public static boolean is1PieceAuto() {
        return autoChooser.getRawButton(5);
    }

    public static boolean is4PieceAmpSideAuto() {
        return autoChooser.getRawButton(4);
    }

    public static boolean getDriverRightJoystickPressed() {
        return driverController.getRightStickButton();
    }
}