package frc.robot.subsystems.intake;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.tools.controlloops.PID;
import org.littletonrobotics.junction.Logger;

class IntakeIOSim implements IntakeIO {
    DCMotor gearbox = Constants.MotorSpecs.x44.getX44Gearbox(Constants.Physical.Intake.NUM_INTAKE_MOTORS)
            .withReduction(Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO);
    private final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.Intake.MOI));
    private final Vector<N2> B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.Intake.MOI);
    double inputTorqueCurrent = 0.0;
    double positionSetpointRad = 0.0;
    boolean closedLoop = true;
    private Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
    private PID slot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Intake.kP0),
            Units.radiansToRotations(
                    Constants.PIDConstants.Intake.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Intake.kD0));
    private double kG0 = Constants.PIDConstants.Intake.kG0; // Gravity feedforward gain

    @Override
    public void init() {
    }

    @Override
    public void updateInputs(IntakeState systemState) {
        if (!closedLoop) {
            update(Globals.loopPeriodSecs);
        } else {
            double dt = Globals.loopPeriodSecs;
            int numSteps = (int) Math.floor(dt / Constants.closedLoopSimResolution);
            slot0.setSetPoint(positionSetpointRad);
            for (int i = 0; i < numSteps; i++) {
                double pidOutput;
                pidOutput = slot0.updatePID(simState.get(0));
                double feedforward = -kG0 * Math.sin(simState.get(0));
                double wantedSpeed = pidOutput + feedforward;
                inputTorqueCurrent = Math.copySign(
                        gearbox.getCurrent(simState.get(0), wantedSpeed / gearbox.KvRadPerSecPerVolt), wantedSpeed);
                update(dt / numSteps);
            }
            Logger.recordOutput("intake sim error", Units.radiansToDegrees(positionSetpointRad - simState.get(0)));
            Logger.recordOutput("intake sim current", inputTorqueCurrent);
        }
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
                simState,
                VecBuilder.fill(inputTorqueCurrent),
                dt);
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }

    @Override
    public void setIntakePosition(double rotations) {
        positionSetpointRad = Units.rotationsToRadians(rotations);
        closedLoop = true;
    }

    @Override
    public void setRollerPercent(double percent) { // TODO: do we need to simulate the intake rollers?
        // double velocity = simState.get(1);
        // double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts *
        // (Kv in rad/s/V) = rad/s */);
        // inputTorqueCurrent = currentRequired;
        // closedLoop = false;
    }

    @Override
    public double getIntakePosition() {
        return Units.radiansToRotations(simState.get(0));
    }

    @Override
    public void setRollerTorque(double amps) {

    }
}