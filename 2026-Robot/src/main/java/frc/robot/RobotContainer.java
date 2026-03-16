package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbLevelManual;
import frc.robot.commands.ContinuousConditionalCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateComplicatedAfterWait;
import frc.robot.commands.SetRobotStateOnce;
import frc.robot.commands.SetRobotStatePresetShot;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetRobotStateSimpleOnce;
import frc.robot.commands.SetRobotStateTimeout;
import frc.robot.commands.SlowFollower;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.ZeroTurretMidMatch;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Peripherals;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.tools.math.ShotCalculator.ShotSolution;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        // Subsystems
        final Peripherals peripherals = new Peripherals();
        final Drive drive = new Drive(peripherals);
        final Lights lights = new Lights();
        final Shooter shooter = new Shooter();
        final Feeder feeder = new Feeder();
        final Intake intake = new Intake();
        final Climber climber = new Climber();
        Superstructure superstructure = new Superstructure(drive, lights, shooter, intake, feeder, climber);

        HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
                {
                        put("Idle", () -> new SetRobotStateSimple(superstructure, SuperState.IDLE));
                        put("Full Send", () -> new FullSendFollower(drive, null, false));
                        put("Slow Mode", () -> new SlowFollower(drive, null, false));
                        put("Shoot", () -> new SequentialCommandGroup(
                                        new SetRobotStateTimeout(superstructure, SuperState.SHOOT_NO_JIGGLE, 1.5),
                                        new SetRobotStateTimeout(superstructure, SuperState.SHOOT, 2.0)));
                        put("Intake", () -> new SetRobotState(superstructure, SuperState.INTAKING));
                        put("Climb", () -> new SetRobotState(superstructure, SuperState.SHOOT));
                        put("ShootMore", () -> new SequentialCommandGroup(
                                        new SetRobotStateTimeout(superstructure, SuperState.SHOOT_NO_JIGGLE, 1.0),
                                        new SetRobotStateTimeout(superstructure, SuperState.SHOOT, 7.0)));
                }
        };

        File[] autoFiles = new File[Constants.Autonomous.paths.length];
        Command[] autos = new Command[Constants.Autonomous.paths.length];
        JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
        JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];

        HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
                {
                }
        };

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                // Load the Path Files
                for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
                        try {
                                autoFiles[i] = new File(
                                                Filesystem.getDeployDirectory().getPath() + "/"
                                                                + Constants.Autonomous.paths[i]);
                                FileReader scanner = new FileReader(autoFiles[i]);
                                autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
                                autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0)
                                                .getJSONArray("sampled_points");
                                autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap,
                                                conditionMap);
                                java.util.logging.Logger.getGlobal()
                                                .info("Loaded Path: " + Constants.Autonomous.paths[i]);
                        } catch (Exception e) {
                                java.util.logging.Logger.getGlobal()
                                                .severe("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":"
                                                                + e);
                        }
                }
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                // COMPETITION CONTROLS
                // Driver
                OI.driverLT.onTrue(new SetRobotStateSimpleOnce(superstructure, SuperState.SHOOT));
                OI.driverLT.onFalse(new SetRobotStateComplicatedAfterWait(superstructure, SuperState.SHOOTING_NO_FEED,
                                SuperState.DEFAULT, 0.5));

                // OI.driverRT.whileTrue(new SetRobotState(superstructure,
                // SuperState.INTAKING));

                OI.driverRT.whileTrue(new ContinuousConditionalCommand(
                                new DoNothing(),
                                new SetRobotState(superstructure, SuperState.INTAKING),
                                OI.driverLTSupplier));

                OI.driverPOVLeft.whileTrue(new SetRobotStatePresetShot(superstructure,
                                new ShotSolution(new Rotation2d(Math.toRadians(60.0)), 2000, new Rotation2d(Math.PI),
                                                0.0, 0.0)));
                // OI.driverPOVUp.whileTrue(new SetRobotStatePresetShot(superstructure,
                // new ShotSolution(new Rotation2d(Math.toRadians(60.0)), 1000, new
                // Rotation2d(Math.PI),
                // 0.0, 0.0)));
                OI.driverPOVRight.whileTrue(new SetRobotStatePresetShot(superstructure,
                                new ShotSolution(new Rotation2d(Math.toRadians(60.0)), 3000, new Rotation2d(Math.PI),
                                                0.0, 0.0)));

                OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
                OI.driverMenuButton.whileTrue(new ZeroTurretMidMatch(shooter));
                OI.driverPOVUp.whileTrue(new SetRobotStateOnce(superstructure,
                                SuperState.AUTO_L3_CLIMB));

                OI.driverMenuButton.whileTrue(new SetRobotStateSimpleOnce(superstructure, SuperState.ZERO));
                // OI.driverX.whileTrue(new SetRobotStateOnce(superstructure,
                // SuperState.MANUAL_SHOOT));
                OI.driverX.onTrue(new ClimbLevelManual(climber));
                OI.driverRB.whileTrue(new SetRobotState(superstructure, SuperState.MANUAL_CLIMBING));
                OI.driverLB.whileTrue(new SetRobotState(superstructure, SuperState.MANUAL_EXTEND_CLIMBER));
                OI.driverY.whileTrue(new SetRobotStateOnce(superstructure, SuperState.AUTO_PREP_CLIMB));
                // Operator

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                int selectedPath = Constants.Autonomous.getSelectedPathIndex();
                if (selectedPath >= Constants.Autonomous.paths.length) {
                        selectedPath = -1;
                }
                if (selectedPath == -1) {
                        java.util.logging.Logger.getGlobal().info("Selected Path: None");
                        return new DoNothing();
                } else {
                        this.drive.autoInit(autoPoints[selectedPath]);
                        java.util.logging.Logger.getGlobal()
                                        .info("Selected Path: " + Constants.Autonomous.paths[selectedPath]);
                        return this.autos[selectedPath];
                }
        }
}
