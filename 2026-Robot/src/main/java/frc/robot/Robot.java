package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.logging.Level;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.logging.AdvantageKitMultiLevelLogHandler;
import frc.robot.tools.logging.Elastic;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  private AdvantageKitMultiLevelLogHandler m_logHandler = new AdvantageKitMultiLevelLogHandler();

  File[] autoFiles;
  Command[] autos;
  JSONObject[] autoJSONs;
  JSONArray[] autoPoints;

  JSONObject autoPath;
  PolarAutoFollower autoCommand;

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

    autoFiles = new File[Constants.paths.size()];
    autos = new Command[Constants.paths.size()];
    autoJSONs = new JSONObject[Constants.paths.size()];
    autoPoints = new JSONArray[Constants.paths.size()];
    for (int i = 0; i < Constants.paths.size(); i++) {
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.paths.get(i));
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0)
            .getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i],
            m_robotContainer.drive, m_robotContainer.lights, m_robotContainer.peripherals, m_robotContainer.commandMap,
            m_robotContainer.conditionMap);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH " + Constants.paths.get(i) + ":" + e);
      }
    }
    Elastic.selectTab("Autonomous");
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Physical/FieldSide", Globals.fieldSide);
    Logger.recordOutput("Physical/Blue Hub", Constants.Field.HUB_POSE_BLUE);
    Globals.fieldSide = OI.fieldSide.getSelected();

    CommandScheduler.getInstance().run();
    Logger.recordOutput("Robot/MT2 Odometry", m_robotContainer.drive.getMt2Pose2d());
    Logger.recordOutput("Robot/IMU", m_robotContainer.drive.getGyroYaw());
    int index = Constants.Autonomous.getSelectedPathIndex();
    if (index == -1 || index > Constants.Autonomous.paths.length) {
      Logger.recordOutput("Auto/Selected Auto", "Do Nothing");
    } else {
      Logger.recordOutput("Auto/Selected Auto", Constants.Autonomous.paths[index]);
    }
    if (RobotBase.isSimulation()) {
      LoggedMechanismLigament2d intakeLigament2d = m_robotContainer.intake.getLigament();
      LoggedMechanism2d bot = new LoggedMechanism2d(2.0, 2.6);
      bot.getRoot("Intake", 1.0, Units.inchesToMeters(12.5)).append(intakeLigament2d);
      Logger.recordOutput("Sim/Arm Sim", bot);
    }
    Globals.loopPeriodSecs = Timer.getFPGATimestamp() - Globals.prevTimeSecs;
    Globals.prevTimeSecs = Timer.getFPGATimestamp();
    Globals.runTime = Timer.getFPGATimestamp() - Globals.initTime;
    m_robotContainer.lights.periodic();
    m_robotContainer.peripherals.periodic();
    m_logHandler.write();
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
    Elastic.selectTab("Autonomous");
    double autoInitTime = Timer.getFPGATimestamp();
    m_robotContainer.superstructure.setWantedState(SuperState.IDLE);
    if (Globals.fieldSide == "blue") {
      java.util.logging.Logger.getGlobal().info("ON BLUE SIDE");
    } else {
      java.util.logging.Logger.getGlobal().info("ON RED SIDE");
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
    Elastic.selectTab("Teleoperated");
    m_robotContainer.superstructure.setWantedState(SuperState.ZERO);
    m_robotContainer.lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (OI.isRedSide()) {
      Globals.fieldSide = "red";
    } else {
      Globals.fieldSide = "blue";
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

    // if (m_robotContainer.manualMode) {
    // m_robotContainer.drive.robotCentric = true;
    // Elastic.selectTab("Camera View");
    // } else {
    // m_robotContainer.drive.robotCentric = false;
    // Elastic.selectTab("Teleoperated");
    // }
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
