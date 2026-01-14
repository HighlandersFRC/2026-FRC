package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateOnce;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Peripherals;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;

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
        Intake intake = new Intake();
        Superstructure superstructure = new Superstructure(drive, lights, shooter, intake);

        public boolean algaeMode = false;
        boolean manualMode = false;
        boolean yPressed = false;
        RobotContainer m_container = this;

        HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
                {
                        put("Idle", () -> new SetRobotStateSimple(superstructure, SuperState.IDLE));
                        put("Full Send", () -> new FullSendFollower(drive, null, false));
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
                OI.driverB.whileTrue(new SetRobotState(superstructure, SuperState.SHOOT));
                OI.driverA.whileTrue(new SetRobotStateOnce(superstructure, SuperState.INTAKING));
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
