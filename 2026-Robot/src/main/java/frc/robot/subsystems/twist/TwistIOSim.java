package frc.robot.subsystems.twist;

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
import frc.robot.subsystems.twist.Twist.TwistState;
import frc.robot.tools.controlloops.PID;

public class TwistIOSim implements TwistIO {
    DCMotor gearbox = Constants.MotorSpecs.x44.getX44Gearbox(Constants.Physical.TWIST_MOTOR_COUNT)
            .withReduction(Constants.Ratios.TWIST_GEAR_RATIO_ROTOR);
    private final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * Constants.Physical.TWIST_MOI));
    private final Vector<N2> B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / Constants.Physical.TWIST_MOI);
    double inputTorqueCurrent = 0.0;
    double positionSetpointRad = 0.0;
    boolean closedLoop = true;
    private Vector<N2> simState;
    private PID slot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Twist.kP0), Units.radiansToRotations(
            Constants.PIDConstants.Twist.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Twist.kD0));
    private double kS0 = Units.radiansToRotations(Constants.PIDConstants.Twist.kS0);
    private PID slot1 = new PID(Units.radiansToRotations(
            Constants.PIDConstants.Twist.kP1),
            Units.radiansToRotations(
                    Constants.PIDConstants.Twist.kI1),
            Units.radiansToRotations(Constants.PIDConstants.Twist.kD1));
    private double kS1 = Units.radiansToRotations(Constants.PIDConstants.Twist.kS1);
    int slot = 0;

    public TwistIOSim() {
        simState = VecBuilder.fill(Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_SIDE), 0.0);
    }

    @Override
    public void init() {
    }

    @Override
    public void updateInputs(TwistState systemState) {
        Logger.recordOutput("twist setpoint", Units.radiansToDegrees(positionSetpointRad));
        if (!closedLoop) {
            update(Globals.loopPeriodSecs);
        } else {
            double dt = Globals.loopPeriodSecs;
            int numSteps = (int) Math.floor(dt / Constants.closedLoopSimResolution);
            slot0.setSetPoint(positionSetpointRad);
            slot1.setSetPoint(positionSetpointRad);
            for (int i = 0; i < numSteps; i++) {
                double pidOutput;
                if (slot == 0) {
                    pidOutput = slot0.updatePID(simState.get(0));
                } else {
                    pidOutput = slot1.updatePID(simState.get(0));
                }
                double feedforward = Math.copySign((slot == 0 ? kS0 : kS1), pidOutput);
                double wantedSpeed = pidOutput + feedforward;
                inputTorqueCurrent = Math.copySign(
                        gearbox.getCurrent(simState.get(0), wantedSpeed / gearbox.KvRadPerSecPerVolt), wantedSpeed);
                update(dt / numSteps);
            }
            Logger.recordOutput("twist sim error", Units.radiansToDegrees(positionSetpointRad - simState.get(0)));
            Logger.recordOutput("twist sim current", inputTorqueCurrent);
        }
    }

    @Override
    public void setPercent(double percent) {
        double velocity = simState.get(1);
        double currentRequired = gearbox.getCurrent(velocity, 24 * percent /* volts * (Kv in rad/s/V) = rad/s */);
        inputTorqueCurrent = currentRequired;
        closedLoop = false;
    }

    @Override
    public void setTorque(double torque, double maxPercent) {
        inputTorqueCurrent = Math.min(torque, gearbox.getCurrent(simState.get(1), 24 * maxPercent));
        closedLoop = false;
    }

    @Override
    public double getPosition() {
        return Units.radiansToRotations(simState.get(0)) * 360;
    }

    @Override
    public void setEncoderPosition(double position) {
        simState.set(0, 0, Units.rotationsToRadians(position));
    }

    @Override
    public void setPosition(double rotations, int slot) {
        positionSetpointRad = Units.rotationsToRadians(rotations);
        this.slot = slot;
        closedLoop = true;
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
                simState,
                VecBuilder.fill(inputTorqueCurrent),
                dt);
        if (updatedState.get(0, 0) < Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_UP)
                && updatedState.get(1, 0) < 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_UP));
            updatedState.set(1, 0, 0.0);
        } else if (updatedState.get(0, 0) > Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_DOWN)
                && updatedState.get(1, 0) > 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.TwistSetpoints.TWIST_DOWN));
            updatedState.set(1, 0, 0.0);
        }
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
}
