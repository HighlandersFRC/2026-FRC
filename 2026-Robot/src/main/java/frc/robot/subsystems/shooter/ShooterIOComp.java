package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;

class ShooterIOComp implements ShooterIO {
    private final TalonFX flywheelMaster = new TalonFX(Constants.CANInfo.FLYWHEEL_MASTER_ID, "canivore");
    private final TalonFX flywheelSlave = new TalonFX(Constants.CANInfo.FLYWHEEL_SLAVE_ID, "canivore");
    private final TalonFX turretMotor = new TalonFX(Constants.CANInfo.TURRET_MOTOR_ID, "canivore");
    private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.HOOD_MOTOR_ID, "canivore");

    private final CANcoder encoderOne = new CANcoder(Constants.CANInfo.TURRET_CANCODER_ONE_ID, "canivore");
    private final CANcoder encoderTwo = new CANcoder(Constants.CANInfo.TURRET_CANCODER_TWO_ID, "canivore");

    public ShooterIOComp() {
        flywheelSlave.setControl(new Follower(Constants.CANInfo.FLYWHEEL_MASTER_ID, false));
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Slot0.kP = Constants.PIDConstants.Hood.kP0;
        turretConfig.Slot0.kI = Constants.PIDConstants.Hood.kI0;
        turretConfig.Slot0.kD = Constants.PIDConstants.Hood.kD0;
        turretConfig.Slot0.kS = Constants.PIDConstants.Hood.kS0;
        turretConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.HOOD_GEAR_RATIO;
        turretConfig.Feedback.RotorToSensorRatio = 2.0 - 1.0;
        turretConfig.CurrentLimits.StatorCurrentLimit = 40;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretMotor.getConfigurator().apply(turretConfig);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public Rotation2d getHoodAngle() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble())); // TODO: try
                                                                                                     // getLatencyCompensatedValueAsDouble()
    }

    @Override
    public Rotation2d getTurretAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTurretAngle'");
    }

    @Override
    public double getFlywheelRPM() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelRPM'");
    }

    @Override
    public void moveHoodToAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHoodAngle'");
    }

    @Override
    public void setTurretAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTurretAngle'");
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFlywheelRPM'");
    }

    @Override
    public double getRelativeTurretAngleRadians() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRelativeTurretAngleRadians'");
    }

    @Override
    public void setHoodAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHoodAngle'");
    }

    @Override
    public void updateInputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }
}
