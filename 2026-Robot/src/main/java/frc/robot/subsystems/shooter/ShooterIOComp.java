package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

class ShooterIOComp implements ShooterIO {
    private final Shooter shooter;

    private final TalonFX shooterMotorMaster = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MOTOR_MASTER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX shooterMotorFollower = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MOTOR_FOLLOWER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.SHOOTER_HOOD_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration();

    private final double hoodJerk = 0.0;
    private final double hoodAcceleration = 1.0 / Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
    private final double hoodCruiseVelocity = 1.0 / Constants.Ratios.Shooter.HOOD_GEAR_RATIO;;

    private final DynamicMotionMagicVoltage hoodMotionProfileRequest = new DynamicMotionMagicVoltage(0,
            hoodCruiseVelocity,
            hoodAcceleration,
            hoodJerk);

    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0); // rps

    private final double hoodProfileScalarFactor = 1.0;

    ShooterIOComp(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void init() {

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 10.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 5.0;

        hoodConfig.Slot1.kP = 30.0;
        hoodConfig.Slot1.kI = 0.0;
        hoodConfig.Slot1.kD = 5.0;

        hoodConfig.Slot2.kP = 50.0;
        hoodConfig.Slot2.kI = 0.0;
        hoodConfig.Slot2.kD = 15.0;
        hoodConfig.MotionMagic.MotionMagicJerk = this.hoodJerk;
        hoodConfig.MotionMagic.MotionMagicAcceleration = this.hoodAcceleration;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = this.hoodCruiseVelocity;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodConfig.Feedback.SensorToMechanismRatio = 1.0;
        hoodConfig.Feedback.RotorToSensorRatio = 1.0;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodMotor.setPosition(0.0);
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        shooterMotorConfiguration.Slot0.kP = 10.0;
        shooterMotorConfiguration.Slot0.kI = 0.0;
        shooterMotorConfiguration.Slot0.kD = 0.0;
        // shooterMotorConfiguration.Slot0.kS = 1.0;
        // shooterMotorConfiguration.Slot0.kV = 0.2;
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 140;
        shooterMotorConfiguration.CurrentLimits.StatorCurrentLimit = 140;
        shooterMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorMaster.getConfigurator().apply(shooterMotorConfiguration);
        shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorFollower.getConfigurator().apply(shooterMotorConfiguration);
        shooterMotorFollower.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs() {
        // // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public Rotation2d getHoodAngle() {
        return new Rotation2d(
                hoodMotor.getPosition().getValueAsDouble() * Math.PI * 2.0 * Constants.Ratios.Shooter.HOOD_GEAR_RATIO);
    }

    @Override
    public Rotation2d getTurretAngle() {
        return new Rotation2d(0.0);
    }

    @Override
    public double getFlywheelRPM() {
        return shooterMotorMaster.getRotorVelocity().getValueAsDouble() * 60.0
                / Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
    }

    @Override
    public void setHoodAngle(Rotation2d angle) {
        System.out.println("angle" + angle.getDegrees());
        hoodMotor.setControl(this.hoodMotionProfileRequest
                .withPosition(
                        angle.getRotations() / Constants.Ratios.Shooter.HOOD_GEAR_RATIO)
                .withVelocity(this.hoodCruiseVelocity * hoodProfileScalarFactor)
                .withAcceleration(this.hoodAcceleration * hoodProfileScalarFactor)
                .withJerk(
                        this.hoodJerk * hoodProfileScalarFactor)
                .withSlot(0));

        Logger.recordOutput("hood amps", hoodMotor.getTorqueCurrent().getValueAsDouble());
    }

    @Override
    public void setTurretAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTurretAngle'");
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        shooterMotorMaster.setControl(flywheelVelocityRequest
                .withVelocity(Constants.RPMToRPS(-rpm) * Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO));
        shooterMotorFollower.setControl(flywheelVelocityRequest
                .withVelocity(Constants.RPMToRPS(rpm) * Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO));

        Logger.recordOutput("shooter speed error: ", shooterMotorMaster.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("shooter amps", shooterMotorMaster.getTorqueCurrent().getValueAsDouble());
    }

    @Override
    public void setShooterPercent(double percent) {
        shooterMotorMaster.set(-percent);
        shooterMotorFollower.set(percent);
    }

    @Override
    public void setHoodPercent(double percent) {
        hoodMotor.set(percent);
    }

    @Override
    public double getRelativeTurretAngleRadians() {
        return 0.0;
    }

    @Override
    public double getShooterStatorCurrent() {
        return shooterMotorMaster.getStatorCurrent().getValueAsDouble();
    }

}
