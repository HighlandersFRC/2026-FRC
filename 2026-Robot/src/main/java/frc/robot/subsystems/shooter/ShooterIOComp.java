package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.tools.logging.TunableNumber;

class ShooterIOComp implements ShooterIO {
    private final TalonFX flywheelMaster = new TalonFX(Constants.CANInfo.FLYWHEEL_MASTER_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX flywheelFollower = new TalonFX(Constants.CANInfo.FLYWHEEL_SLAVE_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX turretMotor = new TalonFX(Constants.CANInfo.TURRET_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
    private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.HOOD_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

    private final VelocityVoltage flywheelControl = new VelocityVoltage(0.0);

    private final CANcoder encoderOne = new CANcoder(Constants.CANInfo.TURRET_CANCODER_ONE_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANcoder encoderTwo = new CANcoder(Constants.CANInfo.TURRET_CANCODER_TWO_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final double SLOPE = ((double) (Constants.Physical.Shooter.TURRET_GEAR_1_TOOTH_COUNT
            - Constants.Physical.Shooter.TURRET_GEAR_2_TOOTH_COUNT))
            / (double) Constants.Physical.Shooter.TURRET_GEAR_2_TOOTH_COUNT;

    private TunableNumber turretP = new TunableNumber("Turret Position kP", Constants.PIDConstants.Turret.kP0);
    private TunableNumber turretI = new TunableNumber("Turret Position kI", Constants.PIDConstants.Turret.kI0);
    private TunableNumber turretD = new TunableNumber("Turret Position kD", Constants.PIDConstants.Turret.kD0);
    private TunableNumber turretS = new TunableNumber("Turret Position kS", Constants.PIDConstants.Turret.kS0);
    private TunableNumber turretVelocity = new TunableNumber("Turret Position Velocity", 2);
    private TunableNumber turretAcceleration = new TunableNumber("Turret Position Acceleration", 2);

    private TunableNumber hoodP = new TunableNumber("Hood Position kP", Constants.PIDConstants.Hood.kP0);
    private TunableNumber hoodI = new TunableNumber("Hood Position kI", Constants.PIDConstants.Hood.kI0);
    private TunableNumber hoodD = new TunableNumber("Hood Position kD", Constants.PIDConstants.Hood.kD0);
    private TunableNumber hoodS = new TunableNumber("Hood Position kS", Constants.PIDConstants.Hood.kS0);
    private TunableNumber hoodG = new TunableNumber("Hood Position kG", Constants.PIDConstants.Hood.kG0);
    private TunableNumber hoodCruiseVelocity = new TunableNumber("Hood Position Velocity", 2.0);
    private TunableNumber hoodAcceleration = new TunableNumber("Hood Position Acceleration", 2.0);
    private final double hoodProfileScalarFactor = 1.0;

    private final DynamicMotionMagicVoltage hoodMotionProfileRequest = new DynamicMotionMagicVoltage(0,
            hoodCruiseVelocity.get(),
            hoodAcceleration.get(),
            0.0);

    private TunableNumber flywheelP = new TunableNumber("Flywheel Position kP", Constants.PIDConstants.Flywheel.kP0);
    private TunableNumber flywheelI = new TunableNumber("Flywheel Position kI", Constants.PIDConstants.Flywheel.kI0);
    private TunableNumber flywheelD = new TunableNumber("Flywheel Position kD", Constants.PIDConstants.Flywheel.kD0);
    private TunableNumber flywheelS = new TunableNumber("Flywheel Position kS", Constants.PIDConstants.Flywheel.kS0);
    private TunableNumber flywheelV = new TunableNumber("Flywheel Position kV", Constants.PIDConstants.Flywheel.kV0);
    private TunableNumber flywheelVelocity = new TunableNumber("Flywheel Position Velocity", 2.0);
    private TunableNumber flywheelAcceleration = new TunableNumber("Flywheel Position Acceleration", 2.0);

    public ShooterIOComp() {
        // Hood Motor Configuration //TODO: Gotta tune all of the configs
        System.out.println("slope: " + SLOPE);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = Constants.PIDConstants.Hood.kP0;
        hoodConfig.Slot0.kI = Constants.PIDConstants.Hood.kI0;
        hoodConfig.Slot0.kD = Constants.PIDConstants.Hood.kD0;
        hoodConfig.Slot0.kS = Constants.PIDConstants.Hood.kS0;
        hoodConfig.Slot0.kG = Constants.PIDConstants.Hood.kG0;
        hoodConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Units
                .radiansToRotations(Constants.Physical.Shooter.HOOD_ACCELERATION_RAD_S);
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Units
                .radiansToRotations(Constants.Physical.Shooter.HOOD_MAX_SPEED_RAD_S);
        hoodConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
        hoodConfig.Feedback.RotorToSensorRatio = 1.0;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 67;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 67;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .radiansToRotations(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS
                        - Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS);
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.setPosition(0.0);

        // Flywheel Configuration
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = Constants.PIDConstants.Flywheel.kP0;
        flywheelConfig.Slot0.kI = Constants.PIDConstants.Flywheel.kI0;
        flywheelConfig.Slot0.kD = Constants.PIDConstants.Flywheel.kD0;
        flywheelConfig.Slot0.kS = Constants.PIDConstants.Flywheel.kS0;
        flywheelConfig.Slot0.kV = Constants.PIDConstants.Flywheel.kV0;
        flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
        flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80;
        flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
        flywheelMaster.getConfigurator().apply(flywheelConfig);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelFollower.getConfigurator().apply(flywheelConfig);
        flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

        // Turret Motor Configuration
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Slot0.kP = Constants.PIDConstants.Turret.kP0;
        turretConfig.Slot0.kI = Constants.PIDConstants.Turret.kI0;
        turretConfig.Slot0.kD = Constants.PIDConstants.Turret.kD0;
        turretConfig.Slot0.kS = Constants.PIDConstants.Turret.kS0;
        turretConfig.MotionMagic.MotionMagicAcceleration = 2.0;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;
        turretConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.TURRET_GEAR_RATIO;
        turretConfig.Feedback.RotorToSensorRatio = 1.0;
        turretConfig.CurrentLimits.StatorCurrentLimit = 67;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 67;
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretMotor.getConfigurator().apply(turretConfig);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        // CANcoder Configuration
        CANcoderConfiguration encoderOneConfig = new CANcoderConfiguration();
        encoderOneConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderOneConfig.MagnetSensor.MagnetOffset = -0.271240234375; // TODO: Try calculating offset from previous zero
                                                                      // data
        encoderOneConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderOne.getConfigurator().apply(encoderOneConfig);
        CANcoderConfiguration encoderTwoConfig = new CANcoderConfiguration();
        encoderTwoConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderTwoConfig.MagnetSensor.MagnetOffset = -0.516357421875;
        encoderTwoConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderTwo.getConfigurator().apply(encoderTwoConfig);

        turretMotor.setPosition(getRelativeTurretAngleRadians());
    }

    @Override
    public Rotation2d getHoodAngle() {
        return Constants.SetPoints.Hood
                .motorAngleToHoodAngle(Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble())); // TODO:
                                                                                                              // try
                                                                                                              // getLatencyCompensatedValueAsDouble()
    }

    @Override
    public Rotation2d getTurretAngle() {
        // return new Rotation2d(getRelativeTurretAngleRadians());
        return new Rotation2d(Math.PI / 2);
    }

    @Override
    public double getFlywheelRPM() {
        return flywheelMaster.getVelocity().getValueAsDouble() * 60 / Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
    }

    @Override
    public void moveHoodToAngle(Rotation2d angle) {
        if (angle.getDegrees() > 85.0) {
            angle = Rotation2d.fromDegrees(85.0);
        }
        if (angle.getDegrees() < 55.0) {
            angle = Rotation2d.fromDegrees(55.0);
        }
        double wantedAngle = Constants.SetPoints.Hood.hoodAngleToMotorAngle(angle).getRotations();
        hoodMotor.setControl(this.hoodMotionProfileRequest
                .withPosition(
                        wantedAngle)
                .withVelocity(this.hoodCruiseVelocity.get() * hoodProfileScalarFactor)
                .withAcceleration(this.hoodAcceleration.get() * hoodProfileScalarFactor)
                .withSlot(0));
    }

    @Override
    public void setTurretAngle(double angle) {
        turretMotor.setControl(new MotionMagicTorqueCurrentFOC(Units.radiansToRotations(angle)));
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        double velocitySetpoint = rpm / 60 * Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
        flywheelMaster.setControl(flywheelControl.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true));
        flywheelFollower.setControl(flywheelControl.withVelocity(-velocitySetpoint).withSlot(0).withEnableFOC(true));
    }

    @Override
    public double getRelativeTurretAngleRadians() {
        double r1 = encoderOne.getAbsolutePosition().getValueAsDouble();
        double r2 = encoderTwo.getAbsolutePosition().getValueAsDouble();
        Logger.recordOutput("encoderOnePosition", r1 * 4096);
        Logger.recordOutput("encoderTwoPosition", r2 * 4096);
        double difference = r1 - r2;
        if (difference > 0.5) {
            difference -= 1.0;
        }
        if (difference < -0.5) {
            difference += 1.0;
        }
        Logger.recordOutput("encoder difference", difference);
        double gear1Rotations = -difference / SLOPE;
        double turretRelativeRotations = gear1Rotations * Constants.Physical.Shooter.TURRET_PULLEY_1_TOOTH_COUNT
                / Constants.Physical.Shooter.TURRET_PULLEY_0_TOOTH_COUNT;
        return Units.rotationsToRadians(turretRelativeRotations);
    }

    @Override
    public void setHoodAngle(Rotation2d angle) { // DON'T USE
        // hoodMotor.setPosition(angle.getRotations());
    }

    @Override
    public void setFlywheelPercent(double percent) {
        flywheelMaster.set(percent);
        flywheelFollower.set(-percent);
    }

    @Override
    public void updateInputs() {
        Constants.Vision.updateLimelightPoseFromTurret(getTurretAngle(), Constants.Physical.Shooter.SHOOTER_POSITION,
                Constants.Vision.LIMELIGHT_TO_TURRET_OFFSET, Constants.Vision.LIMELIGHT_ROTATION_RELATIVE_TO_TURRET,
                Constants.Vision.LIMELIGHT_NAME);
        Logger.recordOutput("Relative Turret Angle", Math.toDegrees(getRelativeTurretAngleRadians()));
        Logger.recordOutput("Motor Turret Angle",
                Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble()));
        if (turretP.changed() || turretI.changed() || turretD.changed() || turretS.changed() || turretVelocity.changed()
                || turretAcceleration.changed()) {
            System.out.println("Updating Turret PID Constants");
            TalonFXConfiguration turretConfig = new TalonFXConfiguration();
            turretConfig.Slot0.kP = turretP.get();
            turretConfig.Slot0.kI = turretI.get();
            turretConfig.Slot0.kD = turretD.get();
            turretConfig.Slot0.kS = turretS.get();
            turretConfig.MotionMagic.MotionMagicAcceleration = turretAcceleration.get();
            turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretVelocity.get();
            turretConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.TURRET_GEAR_RATIO;
            turretConfig.Feedback.RotorToSensorRatio = 1.0;
            turretConfig.CurrentLimits.StatorCurrentLimit = 67;
            turretConfig.CurrentLimits.SupplyCurrentLimit = 67;
            turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            turretMotor.getConfigurator().apply(turretConfig);
        }
        if (hoodP.changed() || hoodI.changed() || hoodD.changed() || hoodS.changed() || hoodG.changed()
                || hoodCruiseVelocity.changed()
                || hoodAcceleration.changed()) {
            System.out.println("Updating Hood PID Constants");
            TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
            hoodConfig.Slot0.kP = hoodP.get();
            hoodConfig.Slot0.kI = hoodI.get();
            hoodConfig.Slot0.kD = hoodD.get();
            hoodConfig.Slot0.kS = hoodS.get();
            hoodConfig.Slot0.kG = hoodG.get();
            hoodConfig.MotionMagic.MotionMagicAcceleration = hoodAcceleration.get();
            hoodConfig.MotionMagic.MotionMagicCruiseVelocity = hoodCruiseVelocity.get();
            hoodConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
            hoodConfig.Feedback.RotorToSensorRatio = 1.0;
            hoodConfig.CurrentLimits.StatorCurrentLimit = 67;
            hoodConfig.CurrentLimits.SupplyCurrentLimit = 67;
            hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            hoodMotor.getConfigurator().apply(hoodConfig);
        }
        if (flywheelP.changed() || flywheelI.changed() || flywheelD.changed() || flywheelS.changed() || flywheelV
                .changed()) {
            System.out.println("Updating Flywheel PID Constants");
            TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
            flywheelConfig.Slot0.kP = flywheelP.get();
            flywheelConfig.Slot0.kI = flywheelI.get();
            flywheelConfig.Slot0.kD = flywheelD.get();
            flywheelConfig.Slot0.kS = flywheelS.get();
            flywheelConfig.Slot0.kV = flywheelV.get();
            flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
            flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
            flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
            flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80;
            flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
            flywheelMaster.getConfigurator().apply(flywheelConfig);
            flywheelFollower.getConfigurator().apply(flywheelConfig);
        }
    }
}
