package frc.robot;

import java.util.logging.Level;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.logging.AdvantageKitMultiLevelLogHandler;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private AdvantageKitMultiLevelLogHandler m_logHandler = new AdvantageKitMultiLevelLogHandler();
  boolean bPressed = false;
  boolean yPressed = false;
  boolean xPressed = false;
  boolean autoChooserCenterSwitch = false;

  @Override
  public void robotInit() {
    Globals.initTime = Timer.getFPGATimestamp();
    /*
     * The Logging Framework built into Java has 5 levels of logging:
     * 
     * Severe: Used for very serious errors that will cause the program to crash
     * (e.g. Auto not loaded).
     * 
     * Warning: Used for potentially harmful situations (e.g. Cameras unable to load
     * Field Layout).
     * 
     * Info: Used for informational messages that highlight the progress of the
     * match (e.g. Auto chose True Path, Robot Init).
     * 
     * Fine: Used for debugging messages that are useful for developers, but don't
     * print every scheduler run (e.g. robot state set to climb).
     * 
     * Finer: Used for very detailed debugging messages that print every scheduler
     * run (e.g. robot position, elevator height)
     * 
     * For Match Logging, use Info Level, and all the other stuff save to Advantage
     * Scope
     * For Development, use Fine or Finer
     * 
     */

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    // The level for logs going to advantage scope. LEAVE THIS AT "ALL"
    java.util.logging.Logger.getLogger("").setLevel(Level.ALL);

    // The level for logs printed to console. CHANGE THIS ONE TO OFF FOR COMP
    java.util.logging.Logger.getLogger("").getHandlers()[0].setLevel(Level.INFO);

    java.util.logging.Logger.getLogger("").addHandler(m_logHandler);

    java.util.logging.Logger.getGlobal().info("Robot Init");

    Globals.fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);
    m_robotContainer = new RobotContainer();

    m_robotContainer.peripherals.init();
    m_robotContainer.drive.init();
    m_robotContainer.lights.init();

    PortForwarder.add(5800, "orangepi1.local", 5800);
    PortForwarder.add(5801, "orangepi1.local", 5801);

    PortForwarder.add(5800, "10.44.99.34", 5800);
    PortForwarder.add(5801, "10.44.99.34", 5801);

    m_robotContainer.lights.clearAnimations();

    // m_robotContainer.lights.setFlashYellow();
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("FieldSide", Globals.fieldSide);
    Logger.recordOutput("Blue Hub", Constants.Field.HUB_POSE_BLUE);

    CommandScheduler.getInstance().run();
    Logger.recordOutput("MT2 Odometry", m_robotContainer.drive.getMt2Pose2d());
    m_robotContainer.superstructure.algaeMode = m_robotContainer.algaeMode;
    m_robotContainer.lights.updateAlgaeMode(m_robotContainer.algaeMode);
    m_robotContainer.lights.updateManualMode(m_robotContainer.manualMode);
    m_robotContainer.drive.algaeMode = m_robotContainer.algaeMode;
    Logger.recordOutput("Algae Mode", m_robotContainer.algaeMode);
    Logger.recordOutput("Manual Mode", m_robotContainer.manualMode);
    Logger.recordOutput("IMU", m_robotContainer.drive.getGyroYaw());
    int index = Constants.Autonomous.getSelectedPathIndex();
    if (index == -1 || index > Constants.Autonomous.paths.length) {
      Logger.recordOutput("Selected Auto", "Do Nothing");
    } else {
      Logger.recordOutput("Selected Auto", Constants.Autonomous.paths[index]);
    }
    Globals.loopPeriodSecs = Timer.getFPGATimestamp() - Globals.prevTimeSecs;
    Globals.prevTimeSecs = Timer.getFPGATimestamp();
    Globals.runTime = Timer.getFPGATimestamp() - Globals.initTime;
    m_robotContainer.lights.periodic();
    m_robotContainer.peripherals.periodic();
    m_logHandler.write();
    Logger.recordOutput("SuperState", m_robotContainer.superstructure.getCurrentSuperState().toString());
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    m_robotContainer.lights.clearAnimations();
    java.util.logging.Logger.getGlobal().info("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    double autoInitTime = Timer.getFPGATimestamp();
    m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
    if (OI.isBlueSide()) {
      java.util.logging.Logger.getGlobal().info("ON BLUE SIDE");
      Globals.fieldSide = "blue";
    } else {
      java.util.logging.Logger.getGlobal().info("ON RED SIDE");
      Globals.fieldSide = "red";
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    java.util.logging.Logger.getGlobal().info("Auto init time" + (Timer.getFPGATimestamp() - autoInitTime));
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.superstructure.setWantedState(SuperState.DEFAULT);
    m_robotContainer.lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (OI.isBlueSide()) {
      Globals.fieldSide = "blue";
    } else {
      Globals.fieldSide = "red";
    }

    // Leave uncommented to use field relative theta system. Instead we are flipping
    // joystick values on red side.
    // if (this.Globals.fieldSide == "red") {
    // this.m_robotContainer.drive.setPigeonAfterAuto();
    // }
    java.util.logging.Logger.getGlobal().info("field side" + Globals.fieldSide);

    this.m_robotContainer.drive.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    if (OI.driverX.getAsBoolean()) {
      if (xPressed) {
        m_robotContainer.manualMode = !m_robotContainer.manualMode;
        xPressed = false;
      }
    } else {
      xPressed = true;
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
