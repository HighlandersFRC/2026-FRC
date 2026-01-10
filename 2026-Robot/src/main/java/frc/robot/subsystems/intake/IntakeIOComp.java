package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;

public class IntakeIOComp implements IntakeIO {
    private final TalonFX roller = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TalonFX pivot = new TalonFX(Constants.CANInfo.INTAKE_PIVOT_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
    private final PositionTorqueCurrentFOC m_positionTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0.0);
    private final TorqueCurrentFOC m_torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    @Override
    public void init() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;
        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.Slot0.kP = 4.068;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kG = 0;
        config.MotionMagic.MotionMagicAcceleration = Constants.SetPoints.IntakeSetpoints.INTAKE_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY;
        roller.getConfigurator().apply(config);
        roller.setNeutralMode(NeutralModeValue.Brake);
        pivot.getConfigurator().apply(config);
        pivot.setNeutralMode(NeutralModeValue.Brake);
        pivot.setPosition(0);
    }

    @Override
    public void setPivotTorque(double current, double maxPercent) {
        pivot.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public void setPivotPosition(double pivotRotations) {
        pivot.setControl(m_positionTorqueCurrentFOCRequest
                .withPosition(pivotRotations * Constants.Ratios.INTAKE_PIVOT_GEAR_RATIO)
                .withVelocity(Constants.SetPoints.IntakeSetpoints.INTAKE_CRUISE_VELOCITY
                        * Constants.SetPoints.IntakeSetpoints.INTAKE_MOTION_PROFILE_SCALAR)
                .withSlot(0));
    }

    @Override
    public void setRollerCurrent(double amps, double maxPercent) {
        roller.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public void setRollerPercent(double percent) {
        roller.set(percent);
    }

    @Override
    public double getPivotPosition() {
        return pivot.getPosition().getValueAsDouble() / Constants.Ratios.INTAKE_PIVOT_GEAR_RATIO;
    }

    @Override
    public double getPivotStatorCurrent() {
        return pivot.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getPivotVelocity() {
        return pivot.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRollerStatorCurrent() {
        return roller.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getRollerVelocity() {
        return roller.getVelocity().getValueAsDouble();
    }

    @Override
    public void setPivotEncoderPosition(double d) {
        pivot.setPosition(d);
    }

    @Override
    public void setPivotCurrent(double amps, double maxPercent) {
        pivot.setControl(m_torqueCurrentFOCRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public double getRollerTorqueCurrent() {
        return roller.getTorqueCurrent().getValueAsDouble();
    }
}
