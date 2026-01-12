package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.tools.controlloops.PID;

class ShooterIOSim extends ShooterIO {
    private double flywheelVelocity = 0.0;
    private double flywheelWantedVelocity = 0.0;
    private DCMotor hoodGearbox = Constants.MotorSpecs.x44.getX44Gearbox(Constants.Physical.Shooter.HOOD_MOTOR_COUNT)
            .withReduction(Constants.Ratios.Shooter.HOOD_GEAR_RATIO);
    private final Matrix<N2, N2> hoodA = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -hoodGearbox.KtNMPerAmp
                    / (hoodGearbox.KvRadPerSecPerVolt * hoodGearbox.rOhms * Constants.Physical.Shooter.HOOD_MOI));
    private final Vector<N2> hoodB = VecBuilder.fill(0.0, hoodGearbox.KtNMPerAmp / Constants.Physical.Shooter.HOOD_MOI);
    private double hoodInputTorqueCurrent = 0.0;
    private double hoodPositionSetpointRad = 0.0;
    private boolean hoodClosedLoop = true;
    private PID hoodSlot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Hood.kP0),
            Units.radiansToRotations(
                    Constants.PIDConstants.Hood.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Hood.kD0));
    private double kHoodS0 = Units.radiansToRotations(Constants.PIDConstants.Hood.kS0);
    private Vector<N2> hoodSimState;
    DCMotor turretGearbox = Constants.MotorSpecs.x44.getX44Gearbox(Constants.Physical.Shooter.TURRET_MOTOR_COUNT)
            .withReduction(Constants.Ratios.Shooter.TURRET_GEAR_RATIO);
    private final Matrix<N2, N2> turretA = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -turretGearbox.KtNMPerAmp
                    / (turretGearbox.KvRadPerSecPerVolt * turretGearbox.rOhms * Constants.Physical.Shooter.TURRET_MOI));
    private final Vector<N2> turretB = VecBuilder.fill(0.0,
            turretGearbox.KtNMPerAmp / Constants.Physical.Shooter.TURRET_MOI);
    double turretInputTorqueCurrent = 0.0;
    double turretPositionSetpointRad = 0.0;
    boolean turretClosedLoop = true;
    private PID turretSlot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Turret.kP0),
            Units.radiansToRotations(
                    Constants.PIDConstants.Turret.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Turret.kD0));
    private double kTurretS0 = Units.radiansToRotations(Constants.PIDConstants.Turret.kS0);
    private Vector<N2> turretSimState;
    private final Shooter shooter;
    private Vector3D _trajectorySetpoint = new Vector3D(0, 0, 0);

    ShooterIOSim(Shooter shooter) {
        this.shooter = shooter;
        hoodSimState = VecBuilder.fill(Units.rotationsToRadians(Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS), 0.0);
        turretSimState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    protected void shoot(Vector3D initialVelocity) {
        this._trajectorySetpoint = initialVelocity;
        setHoodAngle(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint));
        setTurretAngle(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));
        setFlywheelRPM(Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
    }

    @Override
    protected boolean readyToShoot() {
        double hoodAngleError = Math
                .abs(getHoodAngle()
                        .minus(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint))
                        .getRadians());
        double turretAngleError = Math.abs(
                getTurretAngle()
                        .minus(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint))
                        .getRadians());
        double flywheelRPMError = Math
                .abs(getFlywheelRPM()
                        - Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
        return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
                && turretAngleError < Constants.SetPoints.Turret.TURRET_PRECISION
                && flywheelRPMError < Constants.SetPoints.Flywheel.FLYWHEEL_RPM_PRECISION;
    }

    @Override
    protected Rotation2d getHoodAngle() {
        return new Rotation2d(hoodSimState.get(0, 0));
    }

    @Override
    protected Rotation2d getTurretAngle() {
        return new Rotation2d(turretSimState.get(0, 0));
    }

    @Override
    protected double getFlywheelRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelVelocity);
    }

    @Override
    protected void setHoodAngle(Rotation2d angle) {
        hoodSlot0.setSetPoint(shooter.getRelativeAngleFromRotation2d(angle));
    }

    @Override
    protected void setTurretAngle(Rotation2d angle) {
        turretSlot0.setSetPoint(shooter.getRelativeAngleFromRotation2d(angle));
    }

    @Override
    protected void setFlywheelRPM(double rpm) {
        flywheelWantedVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    @Override
    protected double getRelativeTurretAngleRadians() {
        return turretSimState.get(0, 0);
    }

    private void updateTurret(double dt) {
        turretInputTorqueCurrent = MathUtil.clamp(turretInputTorqueCurrent, -turretGearbox.stallCurrentAmps,
                turretGearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> turretA.times(x).plus(turretB.times(u)),
                turretSimState,
                VecBuilder.fill(turretInputTorqueCurrent),
                dt);
        if (updatedState.get(0, 0) < Constants.SetPoints.Turret.TURRET_MIN_ANGLE_RADIANS
                && updatedState.get(1, 0) < 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.Turret.TURRET_MIN_ANGLE_RADIANS));
            updatedState.set(1, 0, 0.0);
        } else if (updatedState.get(0, 0) > Units.rotationsToRadians(
                Constants.SetPoints.Turret.TURRET_MAX_ANGLE_RADIANS)
                && updatedState.get(1, 0) > 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.Turret.TURRET_MAX_ANGLE_RADIANS));
            updatedState.set(1, 0, 0.0);
        }
        turretSimState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }

    private void updateHood(double dt) {
        hoodInputTorqueCurrent = MathUtil.clamp(hoodInputTorqueCurrent, -hoodGearbox.stallCurrentAmps,
                hoodGearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> hoodA.times(x).plus(hoodB.times(u)),
                hoodSimState,
                VecBuilder.fill(hoodInputTorqueCurrent),
                dt);
        if (updatedState.get(0, 0) < Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS
                && updatedState.get(1, 0) < 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS));
            updatedState.set(1, 0, 0.0);
        } else if (updatedState.get(0, 0) > Units.rotationsToRadians(
                Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS)
                && updatedState.get(1, 0) > 0) {
            updatedState.set(0, 0, Units.rotationsToRadians(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS));
            updatedState.set(1, 0, 0.0);
        }
        hoodSimState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }

    private void updateFlywheel(double dt) {
        double acceleration = Math.signum(flywheelWantedVelocity - flywheelVelocity)
                * Constants.Physical.Shooter.SHOOTER_FLYWHEEL_ACCELERATION_RAD_S;
        flywheelVelocity += acceleration * dt
                - Constants.Physical.Shooter.SHOOTER_FRICTION_COEFFICIENT * flywheelVelocity * dt;
    }

    @Override
    protected void updateInputs() {
        if (!turretClosedLoop) {
            updateTurret(Globals.loopPeriodSecs);
        } else {
            double dt = Globals.loopPeriodSecs;
            int numSteps = (int) Math.floor(dt / Constants.closedLoopSimResolution);
            turretSlot0.setSetPoint(turretPositionSetpointRad);
            for (int i = 0; i < numSteps; i++) {
                double pidOutput = turretSlot0.updatePID(turretSimState.get(0));
                double feedforward = Math.copySign(kTurretS0, pidOutput);
                double wantedSpeed = pidOutput + feedforward;
                turretInputTorqueCurrent = Math.copySign(
                        turretGearbox.getCurrent(turretSimState.get(0), wantedSpeed / turretGearbox.KvRadPerSecPerVolt),
                        wantedSpeed);
                updateTurret(dt / numSteps);
            }
        }
        if (!hoodClosedLoop) {
            updateHood(Globals.loopPeriodSecs);
        } else {
            double dt = Globals.loopPeriodSecs;
            int numSteps = (int) Math.floor(dt / Constants.closedLoopSimResolution);
            hoodSlot0.setSetPoint(hoodPositionSetpointRad);
            for (int i = 0; i < numSteps; i++) {
                double pidOutput = hoodSlot0.updatePID(hoodSimState.get(0));
                double feedforward = Math.copySign(kHoodS0, pidOutput);
                double wantedSpeed = pidOutput + feedforward;
                hoodInputTorqueCurrent = Math.copySign(
                        hoodGearbox.getCurrent(hoodSimState.get(0), wantedSpeed / hoodGearbox.KvRadPerSecPerVolt),
                        wantedSpeed);
                updateHood(dt / numSteps);
            }
        }
        updateFlywheel(Globals.loopPeriodSecs);
        Logger.recordOutput("Hood SP", Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint));
        Logger.recordOutput("Turret SP",
                Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));
        Logger.recordOutput("Flywheel RPM SP", Constants.SetPoints.Flywheel
                .getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
        Logger.recordOutput("Hood Angle", getHoodAngle());
        Logger.recordOutput("Turret Angle", getTurretAngle());
        Logger.recordOutput("Flywheel RPM", getFlywheelRPM());
        Logger.recordOutput("Ready to Shoot", readyToShoot());
    }
}