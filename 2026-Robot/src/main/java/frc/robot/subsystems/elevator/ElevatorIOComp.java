package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorIOComp implements ElevatorIO {
    private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);
    private final double elevatorAcceleration = 1482542976.0;
    private final double elevatorCruiseVelocity = 449929104911.0;
    private final MotionMagicTorqueCurrentFOC elevatorMotionProfileRequest = new MotionMagicTorqueCurrentFOC(0);

    @Override
    public void updateInputs(ElevatorState systemState) {
        Logger.recordOutput("Elevator State: ", systemState);
        Logger.recordOutput("Elevator Position", getElevatorPosition());
        Logger.recordOutput("Elevator Desired Pos",
                Constants.Ratios
                        .elevatorRotationsToMeters(elevatorMotorMaster.getClosedLoopReference().getValueAsDouble()));
    }

    @Override
    public void teleopInit() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 60;
        currentLimitsConfigs.SupplyCurrentLimit = 60;
        elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
        elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
    }

    @Override
    public void autoInit() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 60;
        currentLimitsConfigs.SupplyCurrentLimit = 60;
        elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
        elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
    }

    @Override
    public void setCurrentLimit(double stator, double supply) {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = stator;
        currentLimitsConfigs.SupplyCurrentLimit = supply;
        elevatorMotorMaster.getConfigurator().apply(currentLimitsConfigs);
        elevatorMotorFollower.getConfigurator().apply(currentLimitsConfigs);
        System.out.println("settting elevator current " + stator + supply);
    }

    @Override
    public void init() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.Slot0.kP = Constants.PIDConstants.Elevator.kP0;
        elevatorConfig.Slot0.kI = Constants.PIDConstants.Elevator.kI0;
        elevatorConfig.Slot0.kD = Constants.PIDConstants.Elevator.kD0;
        elevatorConfig.Slot0.kG = Constants.PIDConstants.Elevator.kG0;
        elevatorConfig.Slot1.kP = Constants.PIDConstants.Elevator.kP1;
        elevatorConfig.Slot1.kI = Constants.PIDConstants.Elevator.kI1;
        elevatorConfig.Slot1.kD = Constants.PIDConstants.Elevator.kD1;
        elevatorConfig.Slot1.kG = Constants.PIDConstants.Elevator.kG1;
        elevatorConfig.Slot2.kP = Constants.PIDConstants.Elevator.kP2;
        elevatorConfig.Slot2.kI = Constants.PIDConstants.Elevator.kI2;
        elevatorConfig.Slot2.kD = Constants.PIDConstants.Elevator.kD2;
        elevatorConfig.Slot2.kG = Constants.PIDConstants.Elevator.kG2;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = this.elevatorAcceleration;
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = this.elevatorCruiseVelocity;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        elevatorMotorMaster.getConfigurator().apply(elevatorConfig);
        elevatorMotorFollower.getConfigurator().apply(elevatorConfig);
        elevatorMotorMaster.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorFollower.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorMaster.setPosition(0.0);
        elevatorMotorFollower.setPosition(0.0);
    }

    @Override
    public void moveWithPercent(double percent) {
        elevatorMotorMaster.set(percent);
        elevatorMotorFollower.set(-percent);
    }

    @Override
    public void moveWithTorque(double current, double maxPercent) {
        elevatorMotorMaster.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
        elevatorMotorFollower.setControl(torqueCurrentFOCRequest.withOutput(-current).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public void setElevatorPosition(double position, int slot) {
        elevatorMotorMaster.setControl(
                elevatorMotionProfileRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position))
                        .withSlot(slot));
        elevatorMotorFollower.setControl(
                elevatorMotionProfileRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position))
                        .withSlot(slot));
    }

    @Override
    public double getElevatorPosition() {
        return Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getPosition().getValueAsDouble());
    }

    @Override
    public void setElevatorEncoderPosition(double position) {
        elevatorMotorMaster.setPosition(position);
        elevatorMotorFollower.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return elevatorMotorMaster.getVelocity().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return elevatorMotorMaster.getStatorCurrent().getValueAsDouble();
    }

}
