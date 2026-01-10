package frc.robot.subsystems.manipulator;

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
import frc.robot.Constants;
import frc.robot.Globals;

public class ManipulatorIOSim implements ManipulatorIO {
    DCMotor gearbox = Constants.MotorSpecs.x44.getX44Gearbox(Constants.Physical.Manipulator.NUM_MOTORS)
            .withReduction(Constants.Physical.Manipulator.REDUCTION);
    private final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp
                    / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.Manipulator.MOI_KG_M2));
    private final Vector<N2> B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.Manipulator.MOI_KG_M2);
    double inputTorqueCurrent = 0.0;
    double positionSetpointRad = 0.0;
    double prevVel = 0.0;
    double acceleration = 0.0;
    private Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
    private final double b = Constants.Physical.Manipulator.VISCOUS_DAMPING_COEFF; // viscous damping coefficient

    @Override
    public void init() {
    }

    @Override
    public void setTorque(double current, double maxPercent) {
        inputTorqueCurrent = Math.min(current, gearbox.getCurrent(simState.get(1), 24 * maxPercent));
    }

    @Override
    public double getTorqueCurrent() {
        return inputTorqueCurrent;
    }

    @Override
    public double getVelocity() {
        return simState.get(1) * Constants.Physical.Manipulator.REDUCTION;
    }

    @Override
    public double getAcceleration() {
        return acceleration * Constants.Physical.Manipulator.REDUCTION;
    }

    @Override
    public void setPercent(double percent) {
        double velocity = simState.get(1);
        double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts * (Kv in rad/s/V) = rad/s */);
        inputTorqueCurrent = currentRequired;
    }

    @Override
    public double getPosition() {
        return simState.get(0) * Constants.Physical.Manipulator.REDUCTION;
    }

    @Override
    public double getStatorCurrent() {
        return Math.abs(inputTorqueCurrent);
    }

    @Override
    public void updateInputs() {
        System.out.println("simming manipulator");
        update(Globals.loopPeriodSecs);
        double dt = Globals.loopPeriodSecs;
        double dv = simState.get(1) - prevVel;
        acceleration = dv / dt;
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> {
                    double theta = x.get(0, 0);
                    double omega = x.get(1, 0);
                    double viscous = -b * omega;
                    return A.times(x)
                            .plus(B.times(u))
                            .plus(VecBuilder.fill(0, viscous));
                },
                simState,
                VecBuilder.fill(inputTorqueCurrent),
                dt);
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
}
