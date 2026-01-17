package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.tools.controlloops.PID;

class ShooterIOComp implements ShooterIO {
    private final TalonFX flywheelMaster = new TalonFX(Constants.CANInfo.FLYWHEEL_MASTER_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX flywheelSlave = new TalonFX(Constants.CANInfo.FLYWHEEL_SLAVE_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX turretMotor = new TalonFX(Constants.CANInfo.TURRET_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
    private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.HOOD_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

    private final CANcoder encoderOne = new CANcoder(Constants.CANInfo.TURRET_CANCODER_ONE_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANcoder encoderTwo = new CANcoder(Constants.CANInfo.TURRET_CANCODER_TWO_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final double SLOPE = ((double) (Constants.Physical.Shooter.TURRET_GEAR_1_TOOTH_COUNT
            - Constants.Physical.Shooter.TURRET_GEAR_2_TOOTH_COUNT))
            / (double) Constants.Physical.Shooter.TURRET_GEAR_2_TOOTH_COUNT;
    private final double turretP = Constants.PIDConstants.Turret.kP0;
    private final double turretI = Constants.PIDConstants.Turret.kI0;
    private final double turretD = Constants.PIDConstants.Turret.kD0;
    private final double turretS = Constants.PIDConstants.Turret.kS0;
    private final PID turret = new PID(turretP, turretI, turretD);

    public ShooterIOComp() {
        // Hood Motor Configuration //TODO: Gotta tune all of the configs
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = Constants.PIDConstants.Hood.kP0;
        hoodConfig.Slot0.kI = Constants.PIDConstants.Hood.kI0;
        hoodConfig.Slot0.kD = Constants.PIDConstants.Hood.kD0;
        hoodConfig.Slot0.kS = Constants.PIDConstants.Hood.kS0;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Units
                .radiansToRotations(Constants.Physical.Shooter.HOOD_ACCELERATION_RAD_S);
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Units
                .radiansToRotations(Constants.Physical.Shooter.HOOD_MAX_SPEED_RAD_S);
        hoodConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
        hoodConfig.Feedback.RotorToSensorRatio = 1.0;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 67;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 67;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        // Flywheel Configuration
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = Constants.PIDConstants.Flywheel.kP0;
        flywheelConfig.Slot0.kI = Constants.PIDConstants.Flywheel.kI0;
        flywheelConfig.Slot0.kD = Constants.PIDConstants.Flywheel.kD0;
        flywheelConfig.Slot0.kS = Constants.PIDConstants.Flywheel.kS0;
        flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
        flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80;
        flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flywheelMaster.getConfigurator().apply(flywheelConfig);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.getConfigurator().apply(flywheelConfig);
        flywheelSlave.setNeutralMode(NeutralModeValue.Coast);

        // Turret Motor Configuration
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Slot0.kP = Constants.PIDConstants.Turret.kP1;
        turretConfig.Slot0.kI = Constants.PIDConstants.Turret.kI1;
        turretConfig.Slot0.kD = Constants.PIDConstants.Turret.kD1;
        turretConfig.Slot0.kS = Constants.PIDConstants.Turret.kS1;
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
        encoderOneConfig.MagnetSensor.MagnetOffset = 0.0; // TODO: Try calculating offset from previous zero data
        encoderOneConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderOne.getConfigurator().apply(encoderOneConfig);
        CANcoderConfiguration encoderTwoConfig = new CANcoderConfiguration();
        encoderTwoConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderOneConfig.MagnetSensor.MagnetOffset = 0.0;
        encoderOneConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderTwo.getConfigurator().apply(encoderTwoConfig);
    }

    @Override
    public Rotation2d getHoodAngle() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble())); // TODO: try
                                                                                                     // getLatencyCompensatedValueAsDouble()
    }

    @Override
    public Rotation2d getTurretAngle() {
        return new Rotation2d(getRelativeTurretAngleRadians());
    }

    @Override
    public double getFlywheelRPM() {
        return flywheelMaster.getVelocity().getValueAsDouble() * 60 / Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
    }

    @Override
    public void moveHoodToAngle(Rotation2d angle) {
        hoodMotor.setControl(new MotionMagicDutyCycle(angle.getRotations()));
    }

    @Override
    public void setTurretAngle(double angle) {
        turret.setSetPoint(angle);
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        double velocitySetpoint = rpm / 60 * Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
        flywheelMaster.setControl(new VelocityDutyCycle(velocitySetpoint));
        flywheelSlave.setControl(new Follower(Constants.CANInfo.FLYWHEEL_MASTER_ID, true));
    }

    @Override
    public double getRelativeTurretAngleRadians() {
        double r1 = encoderOne.getAbsolutePosition().getValueAsDouble();
        double r2 = encoderTwo.getAbsolutePosition().getValueAsDouble();
        double difference = r1 - r2;
        if (difference > 0.5) {
            difference -= 1.0;
        }
        if (difference < -0.5) {
            difference += 1.0;
        }
        double gear1RelRotations = difference * SLOPE;
        double turretRelativeRotations = gear1RelRotations * Constants.Physical.Shooter.TURRET_PULLEY_1_TOOTH_COUNT
                / Constants.Physical.Shooter.TURRET_GEAR_1_TOOTH_COUNT;
        return Units.rotationsToRadians(turretRelativeRotations);
    }

    @Override
    public void setHoodAngle(Rotation2d angle) {
        hoodMotor.setPosition(angle.getRotations());
    }

    @Override
    public void updateInputs() {
        double currentAngle = getRelativeTurretAngleRadians();
        double output = turret.updatePID(currentAngle);
        output += Math.copySign(turretS, output);
        turretMotor.setControl(new VelocityDutyCycle(Units.radiansToRotations(output)));
        flywheelSlave.setControl(new Follower(Constants.CANInfo.FLYWHEEL_MASTER_ID, true));
    }
}
