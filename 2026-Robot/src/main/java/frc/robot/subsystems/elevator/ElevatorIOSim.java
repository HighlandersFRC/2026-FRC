package frc.robot.subsystems.elevator;

import java.util.function.BiFunction;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.tools.controlloops.PID;

public class ElevatorIOSim implements ElevatorIO {
    DCMotor gearbox = DCMotor.getKrakenX60Foc(Constants.Physical.Elevator.NUM_MOTORS)
            .withReduction(Constants.Physical.Elevator.MOTOR_TO_DRUM_REDUCTION);
    private final Matrix<N2, N2> A_1 = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp
                    / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.Elevator.STAGE_1_MOI));
    private final Vector<N2> B_1 = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.Elevator.STAGE_1_MOI);
    private final Matrix<N2, N2> A_2 = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp
                    / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.Elevator.STAGE_2_MOI));
    private final Vector<N2> B_2 = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.Elevator.STAGE_2_MOI);

    double inputTorqueCurrent = 0.0;
    double positionSetpointRad = 0.0;
    boolean closedLoop = true;
    private Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
    private PID slot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Elevator.kP0), Units.radiansToRotations(
            Constants.PIDConstants.Elevator.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Elevator.kD0));
    private double kG0 = Units.radiansToRotations(Constants.PIDConstants.Elevator.kG0);
    private PID slot1 = new PID(Units.radiansToRotations(
            Constants.PIDConstants.Elevator.kP1),
            Units.radiansToRotations(
                    Constants.PIDConstants.Elevator.kI1),
            Units.radiansToRotations(Constants.PIDConstants.Elevator.kD1));
    private double kG1 = Units.radiansToRotations(Constants.PIDConstants.Elevator.kG1);
    private PID slot2 = new PID(Units.radiansToRotations(
            Constants.PIDConstants.Elevator.kP2),
            Units.radiansToRotations(
                    Constants.PIDConstants.Elevator.kI2),
            Units.radiansToRotations(Constants.PIDConstants.Elevator.kD2));
    private double kG2 = Units.radiansToRotations(Constants.PIDConstants.Elevator.kG2);
    private int slot = 0;
    private double minCurrent = -gearbox.stallCurrentAmps;
    private double maxCurrent = gearbox.stallCurrentAmps;

    @Override
    public void teleopInit() {
    }

    @Override

    public void updateInputs(ElevatorState systemState) {
        Logger.recordOutput("elevator setpoint", Units.radiansToDegrees(positionSetpointRad));
        if (!closedLoop) {
            update(Globals.loopPeriodSecs);
        } else {
            double dt = Globals.loopPeriodSecs;
            int numSteps = (int) Math.floor(dt / Constants.closedLoopSimResolution);
            slot0.setSetPoint(positionSetpointRad);
            slot1.setSetPoint(positionSetpointRad);
            slot2.setSetPoint(positionSetpointRad);
            double wantedSpeed = 0.0;
            for (int i = 0; i < numSteps; i++) {
                double pidOutput;
                if (slot == 0) {
                    pidOutput = slot0.updatePID(simState.get(0));
                } else if (slot == 1) {
                    pidOutput = slot1.updatePID(simState.get(0));
                } else {
                    pidOutput = slot2.updatePID(simState.get(0));
                }
                double feedforward = (slot == 0 ? kG0 : (slot == 1 ? kG1 : kG2));
                wantedSpeed = pidOutput + feedforward;
                inputTorqueCurrent = Math.copySign(
                        gearbox.getCurrent(simState.get(0), wantedSpeed / gearbox.KvRadPerSecPerVolt), wantedSpeed);
                update(dt / numSteps);
            }
            Logger.recordOutput("elevator wanted speed",
                    Constants.Ratios.elevatorRotationsToMeters(Units.radiansToRotations(wantedSpeed)));
            Logger.recordOutput("elevator sim error", Constants.Ratios
                    .elevatorRotationsToMeters(Units.radiansToRotations(positionSetpointRad - simState.get(0))));
            Logger.recordOutput("elevator sim current", inputTorqueCurrent);
        }
    }

    @Override
    public void autoInit() {
    }

    @Override
    public void setCurrentLimit(double stator, double supply) {
        minCurrent = -stator;
        maxCurrent = stator;
    }

    @Override
    public void init() {
    }

    @Override
    public void moveWithPercent(double percent) {
        double velocity = simState.get(1);
        double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts * (Kv in rad/s/V) = rad/s */);
        inputTorqueCurrent = currentRequired;
        closedLoop = false;
    }

    @Override
    public void moveWithTorque(double current, double maxPercent) {
        inputTorqueCurrent = Math.min(current, gearbox.getCurrent(simState.get(1), 24 * maxPercent));
        closedLoop = false;
    }

    @Override
    public void setElevatorPosition(double position, int slot) {
        positionSetpointRad = Units.rotationsToRadians(Constants.Ratios.elevatorMetersToRotations(position));
        this.slot = slot;
        closedLoop = true;
    }

    @Override
    public double getElevatorPosition() {
        return Constants.Ratios.elevatorRotationsToMeters(Units.radiansToRotations(simState.get(0)));

    }

    @Override
    public void setElevatorEncoderPosition(double position) {
        simState.set(0, 0, Units.rotationsToRadians(Constants.Ratios.elevatorMetersToRotations(position)));
    }

    @Override
    public double getCurrent() {
        return MathUtil.clamp(inputTorqueCurrent, minCurrent, maxCurrent);
    }

    @Override
    public double getVelocity() {
        return Constants.Ratios.elevatorRotationsToMeters(Units.radiansToRotations(simState.get(1)));
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, minCurrent, maxCurrent);
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> stateFunction = (x, u) -> {
            if (x.get(0, 0) < Units.rotationsToRadians(Constants.Ratios.ELEVATOR_MOTOR_ROTATIONS_FOR_FIRST_STAGE)) {
                return A_1.times(x).plus(B_1.times(u)).plus(
                        VecBuilder.fill(
                                0.0,
                                -Constants.G
                                        * Units.lbsToKilograms(Constants.Physical.Elevator.CARRIAGE_MASS_LB)
                                        * Units.inchesToMeters(Constants.Physical.Elevator.DRUM_DIAMETER_INCHES / 2)
                                        / Constants.Physical.Elevator.STAGE_1_MOI));
            } else {
                return A_2.times(x).plus(B_2.times(u)).plus(
                        VecBuilder.fill(
                                0.0,
                                -Constants.G
                                        * Units.lbsToKilograms(Constants.Physical.Elevator.CARRIAGE_MASS_LB
                                                + Constants.Physical.Elevator.STAGE_2_MASS_LB)
                                        * Units.inchesToMeters(Constants.Physical.Elevator.DRUM_DIAMETER_INCHES / 2)
                                        / Constants.Physical.Elevator.STAGE_2_MOI));
            }
        };
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                stateFunction,
                simState,
                VecBuilder.fill(inputTorqueCurrent
                        * Constants.Physical.Elevator.MOTOR_TO_DRUM_REDUCTION
                        * Constants.Physical.Elevator.MOTOR_TO_DRUM_REDUCTION * Constants.Physical.Elevator.NUM_MOTORS
                        * 4 / 3), // Multiply
                // current
                // by
                // reduction
                // squared
                // to get torque at drum and then multiply by motors. 4/3 is a magical constant
                dt);
        if (updatedState.get(0, 0) < Units.rotationsToRadians(
                Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M))
                && updatedState.get(1, 0) < 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(
                    Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_BOTTOM_POSITION_M)));
            updatedState.set(1, 0, 0.0);
        } else if (updatedState.get(0, 0) > Units.rotationsToRadians(
                Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_MAX_HEIGHT))
                && updatedState.get(1, 0) > 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(
                    Constants.Ratios.elevatorMetersToRotations(Constants.SetPoints.ELEVATOR_MAX_HEIGHT)));
            updatedState.set(1, 0, 0.0);
        }
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
}