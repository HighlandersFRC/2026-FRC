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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCoralGroundPickupFollower;
import frc.robot.commands.AutoPlaceL2Follower;
import frc.robot.commands.AutoPlaceL4Follower;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FeederPickup;
import frc.robot.commands.FeederPickupFollower;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetAlgaeMode;
import frc.robot.commands.SetClimberPivotTorque;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.SetRobotStateComplicated;
import frc.robot.commands.SetRobotStateComplicatedContinuous;
import frc.robot.commands.SetRobotStateOnce;
import frc.robot.commands.SetRobotStateSimple;
import frc.robot.commands.SetRobotStateSimpleOnce;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Peripherals;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.twist.Twist;

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
        Peripherals peripherals = new Peripherals();
        Elevator elevator = new Elevator();
        Drive drive = new Drive(peripherals, elevator);
        Manipulator manipulator = new Manipulator();
        Lights lights = new Lights();
        Pivot pivot = new Pivot();
        Twist twist = new Twist();
        Climber climber = new Climber();
        Intake intake = new Intake();
        Superstructure superstructure = new Superstructure(drive, elevator, manipulator, pivot, twist, climber, lights,
                        intake);

        public boolean algaeMode = false;
        boolean manualMode = false;
        boolean yPressed = false;
        RobotContainer m_container = this;

        HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
                {
                        put("AutoPlaceL2", () -> new AutoPlaceL2Follower(superstructure, drive, 3.3));
                        put("AutoPlaceL4", () -> new AutoPlaceL4Follower(superstructure, drive, 3.3));
                        put("AutoFeeder", () -> new FeederPickupFollower(superstructure, drive));
                        put("FeederIntake", () -> new FeederPickup(superstructure));
                        put("Outake", () -> new SetRobotStateSimple(superstructure, SuperState.OUTAKE));
                        put("L1", () -> new SetRobotStateSimple(superstructure, SuperState.AUTO_L1_PLACE));
                        put("Idle", () -> new SetRobotStateSimple(superstructure, SuperState.IDLE));
                        put("Full Send", () -> new FullSendFollower(drive, null, false));
                        put("IntakeLollipop", () -> new SetRobotState(superstructure, SuperState.LOLLIOP_PICKUP));
                        put("Net", () -> new SetRobotStateSimple(superstructure, SuperState.NET));
                        put("GroundIntake", () -> new SetRobotStateComplicatedContinuous(superstructure,
                                        SuperState.GROUND_CORAL_PICKUP_FRONT, SuperState.PASSOFF_IDLE));
                        put("ReefAlgaeL2", () -> new SetRobotState(superstructure, SuperState.L2_ALGAE_PICKUP));
                        put("ReefAlgaeL3", () -> new SetRobotState(superstructure, SuperState.L3_ALGAE_PICKUP));
                        // put("ReefAlgae", () -> new ReefAlgaePickupFollower(superstructure, drive,
                        // 5.0, m_container));
                        put("AutoIntake", () -> new AutoCoralGroundPickupFollower(superstructure, drive, 4.0));
                        put("PassoffOutakeIdle", () -> new SetRobotStateSimpleOnce(superstructure,
                                        SuperState.PASSOFF_OUTAKE_IDLE));
                        put("ToggleAlgaeMode", () -> new SetAlgaeMode(m_container));
                }
        };

        File[] autoFiles = new File[Constants.Autonomous.paths.length];
        Command[] autos = new Command[Constants.Autonomous.paths.length];
        JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
        JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];

        HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
                {
                        put("HasCoral", () -> manipulator.hasCoralSticky() || intake.hasCoral());
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
                OI.driverMenuButton.onTrue(new SetRobotStateSimpleOnce(superstructure, SuperState.ZERO));
                OI.driverViewButton.whileTrue(new ConditionalCommand(new ZeroAngleMidMatch(
                                drive),
                                new SetRobotStateSimple(superstructure, SuperState.RUN_CLIMB_BACK),
                                () -> (!manualMode))); // zero pidgeon and elevator

                // OI.driverRT.whileTrue(new SetRobotState(superstructure,
                // SuperState.GROUND_CORAL_PICKUP_FRONT));
                OI.driverRT.whileTrue(
                                new SetRobotStateOnce(superstructure, SuperState.GROUND_CORAL_PICKUP_FRONT));
                // OI.driverRB.whileTrue(new SetRobotState(superstructure,
                // SuperState.GROUND_CORAL_PICKUP_BACK));
                OI.driverRB.whileTrue(new ConditionalCommand(new InstantCommand(),
                                new ConditionalCommand(new SetRobotState(superstructure,
                                                SuperState.GROUND_ALGAE_PICKUP_BACK),
                                                new SetRobotState(superstructure, SuperState.GROUND_CORAL_PICKUP_BACK),
                                                () -> (algaeMode)),
                                () -> (superstructure.getCurrentSuperState() == SuperState.L4_PLACE || superstructure
                                                .getCurrentSuperState() == SuperState.L3_PLACE
                                                || superstructure.getCurrentSuperState() == SuperState.L2_PLACE)));

                // OI.driverLT.whileTrue(new SetIntakeState(intake, IntakeState.OUTAKE));
                // OI.driverLT.onFalse(new SetIntakeState(intake, IntakeState.DEFAULT));

                OI.driverLB.whileTrue(new ConditionalCommand(
                                new InstantCommand(),
                                new ConditionalCommand(
                                                new SetRobotState(superstructure,
                                                                SuperState.FEEDER),
                                                new SetRobotState(superstructure,
                                                                SuperState.FEEDER_AUTO),
                                                () -> manualMode),
                                () -> (superstructure.getCurrentSuperState() == SuperState.L4_PLACE || superstructure
                                                .getCurrentSuperState() == SuperState.L3_PLACE
                                                || superstructure.getCurrentSuperState() == SuperState.L2_PLACE
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.AUTO_L4_PLACE
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.AUTO_L3_PLACE
                                                || superstructure.getCurrentSuperState() == SuperState.AUTO_L2_PLACE
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.SCORE_L2
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.SCORE_L3
                                                || superstructure.getCurrentSuperState() == SuperState.SCORE_L4
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.AUTO_SCORE_L2
                                                || superstructure
                                                                .getCurrentSuperState() == SuperState.AUTO_SCORE_L3
                                                || superstructure.getCurrentSuperState() == SuperState.AUTO_SCORE_L4
                                                || algaeMode)));

                // OI.driverMenuButton.whileTrue(new SetRobotState(superstructure,
                // SuperState.DEFAULT));

                // OI.driverA.onTrue(new SetRobotStateSimpleOnce(superstructure,
                // SuperState.CLIMB));
                // OI.driverA.whileTrue(
                // new ConditionalCommand(new InstantCommand(),
                // new SetRobotStateSimple(superstructure, SuperState.RUN_CLIMB_BACK),
                // () -> (!manualMode)));
                // OI.driverY.onTrue(new SetRobotStateSimpleOnce(superstructure,
                // SuperState.DEPLOY_CLIMBER));
                // OI.driverY.whileTrue(
                // new ConditionalCommand(new SetRobotStateSimpleOnce(superstructure,
                // SuperState.DEPLOY_CLIMBER),
                // new SetClimberPivotTorque(climber, -80, 1.0), () -> (!manualMode)));
                OI.driverY.whileTrue(new ConditionalCommand(
                                new ConditionalCommand(
                                                new SetRobotStateSimpleOnce(superstructure, SuperState.AUTO_CLIMB),
                                                new SetRobotStateSimpleOnce(superstructure, SuperState.CLIMB),
                                                () -> (yPressed)),
                                new SetClimberPivotTorque(climber, -80, 1.0), () -> (!manualMode)));
                // OI.driverY.whileTrue(new SetClimberPivotTorque(climber, 60, 0.2));
                // OI.driverA.whileTrue(new SetClimberPivotTorque(climber, -60, 0.2));
                // OI.driverB.whileTrue(new SetClimberPivotTorque(climber, 20, 0.15));

                // ********OFFICIAL COMPETITION CONTROLS*********
                // DO NOT DELETE

                OI.driverPOVUp.whileTrue(
                                new ConditionalCommand(
                                                new SetRobotStateSimple(superstructure, SuperState.L1_PLACE),
                                                new ConditionalCommand(
                                                                new ConditionalCommand(
                                                                                new SetRobotState(superstructure,
                                                                                                SuperState.PROCESSOR),
                                                                                new SetRobotStateComplicated(
                                                                                                superstructure,
                                                                                                SuperState.AUTO_PROCESSOR,
                                                                                                SuperState.DEFAULT),
                                                                                () -> manualMode),
                                                                new SetRobotStateOnce(superstructure,
                                                                                SuperState.AUTO_L1_PLACE),
                                                                () -> algaeMode),
                                                () -> (manualMode && !algaeMode)));

                OI.driverPOVUp.onFalse(
                                new ConditionalCommand(new SetRobotStateSimple(superstructure, SuperState.DEFAULT),
                                                new InstantCommand(), () -> (manualMode && !algaeMode)));

                OI.driverPOVLeft.whileTrue(new ConditionalCommand(
                                new SetRobotStateSimple(superstructure, SuperState.L2_PLACE),
                                new ConditionalCommand(new ConditionalCommand(
                                                new SetRobotState(superstructure, SuperState.L2_ALGAE_PICKUP),
                                                new SetRobotStateComplicated(superstructure,
                                                                SuperState.AUTO_ALGAE_PICKUP, SuperState.DEFAULT),
                                                () -> manualMode),
                                                new SetRobotStateSimple(superstructure, SuperState.AUTO_L2_PLACE),
                                                () -> algaeMode),
                                () -> (manualMode && !algaeMode)));

                OI.driverPOVLeft.onFalse(
                                new ConditionalCommand(
                                                new SetRobotStateSimpleOnce(superstructure, SuperState.DEFAULT),
                                                new ConditionalCommand(
                                                                new SetRobotStateSimple(superstructure,
                                                                                SuperState.SCORE_L2),
                                                                new InstantCommand(), () -> (manualMode && !algaeMode)),
                                                () -> (!manualMode && !algaeMode)));

                OI.driverPOVDown.whileTrue(new ConditionalCommand(
                                new SetRobotStateSimple(superstructure, SuperState.L3_PLACE),
                                new ConditionalCommand(new ConditionalCommand(
                                                new SetRobotState(superstructure, SuperState.L3_ALGAE_PICKUP),
                                                new SetRobotStateComplicated(superstructure,
                                                                SuperState.AUTO_ALGAE_PICKUP, SuperState.DEFAULT),
                                                () -> manualMode),
                                                new SetRobotStateSimple(superstructure, SuperState.AUTO_L3_PLACE),
                                                () -> algaeMode),
                                () -> (manualMode && !algaeMode)));

                OI.driverPOVDown.onFalse(
                                new ConditionalCommand(
                                                new SetRobotStateSimpleOnce(superstructure, SuperState.DEFAULT),
                                                new ConditionalCommand(
                                                                new SetRobotStateSimple(superstructure,
                                                                                SuperState.SCORE_L3),
                                                                new InstantCommand(), () -> (manualMode && !algaeMode)),
                                                () -> (!manualMode && !algaeMode)));

                OI.driverPOVRight.whileTrue(new ConditionalCommand(
                                new SetRobotStateSimple(superstructure, SuperState.L4_PLACE),
                                new ConditionalCommand(
                                                new ConditionalCommand(
                                                                new SetRobotState(superstructure, SuperState.NET),
                                                                new SetRobotStateSimple(superstructure,
                                                                                SuperState.AUTO_NET),
                                                                () -> manualMode),
                                                new SetRobotStateSimple(superstructure, SuperState.AUTO_L4_PLACE),
                                                () -> algaeMode),
                                () -> (manualMode && !algaeMode)));

                OI.driverPOVRight.onFalse(
                                new ConditionalCommand(
                                                new SetRobotStateSimpleOnce(superstructure, SuperState.DEFAULT),
                                                new ConditionalCommand(
                                                                new SetRobotStateSimple(superstructure,
                                                                                SuperState.SCORE_L4),
                                                                new InstantCommand(), () -> (manualMode && !algaeMode)),
                                                () -> (!manualMode)));

                // OI.operatorLT.onFalse(new ConditionalCommand(new
                // SetRobotStateSimple(superstructure, SuperState.SCORE_L4), , () ->
                // superstructure.getCurrentSuperState() == SuperState.AUTO_SCORE_L4));
                OI.operatorY.whileTrue(new ConditionalCommand(
                                new SetRobotState(superstructure, SuperState.L3_ALGAE_PICKUP),
                                new SetRobotStateComplicated(superstructure,
                                                SuperState.AUTO_ALGAE_PICKUP, SuperState.DEFAULT),
                                () -> manualMode));

                OI.operatorX.whileTrue(new ConditionalCommand(
                                new SetRobotState(superstructure,
                                                SuperState.PROCESSOR),
                                new SetRobotStateComplicated(
                                                superstructure,
                                                SuperState.AUTO_PROCESSOR,
                                                SuperState.DEFAULT),
                                () -> manualMode));

                OI.operatorA.whileTrue(new ConditionalCommand(
                                new SetRobotState(superstructure, SuperState.L2_ALGAE_PICKUP),
                                new SetRobotStateComplicated(superstructure,
                                                SuperState.AUTO_ALGAE_PICKUP, SuperState.DEFAULT),
                                () -> manualMode));

                OI.operatorB.whileTrue(new ConditionalCommand(
                                new SetRobotState(superstructure, SuperState.NET),
                                new SetRobotStateComplicated(superstructure,
                                                SuperState.AUTO_NET, SuperState.DEFAULT),
                                () -> manualMode));
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
