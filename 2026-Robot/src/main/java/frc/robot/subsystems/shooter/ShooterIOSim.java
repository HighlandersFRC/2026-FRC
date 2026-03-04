package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.tools.controlloops.PID;

class ShooterIOSim implements ShooterIO {
    private double flywheelVelocity = 0.0;
    private double flywheelWantedVelocity = 0.0;
    private double hoodPositionSetpointRad = Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS;
    private double hoodPositionRad = Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS;
    private double hoodVelocityRadPerSec = 0.0;
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
    private PID turretSlot0 = new PID(Units.radiansToRotations(Constants.PIDConstants.Turret.kP0),
            Units.radiansToRotations(
                    Constants.PIDConstants.Turret.kI0),
            Units.radiansToRotations(Constants.PIDConstants.Turret.kD0));
    private double kTurretS0 = Units.radiansToRotations(Constants.PIDConstants.Turret.kS0);
    private Vector<N2> turretSimState;

    ShooterIOSim() {
        turretSimState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    public Rotation2d getHoodAngle() {
        return new Rotation2d(hoodPositionRad);
    }

    @Override
    public Rotation2d getTurretAngle() {
        return new Rotation2d(turretSimState.get(0, 0));
    }

    @Override
    public double getFlywheelRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelVelocity);
    }

    @Override
    public void moveHoodToAngle(Rotation2d angle) {
        hoodPositionSetpointRad = angle.getRadians();
    }

    @Override
    public void setHoodAngle(Rotation2d angle) {
        hoodPositionRad = angle.getRadians();
    }

    @Override
    public void setTurretAngle(double angle) {
        turretPositionSetpointRad = angle;
    }

    @Override
    public void setFlywheelRPM(double rpm) {
        flywheelWantedVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    @Override
    public double getRelativeTurretAngleRadians() {
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
        double acceleration = Math.signum(hoodPositionSetpointRad - hoodPositionRad)
                * Constants.Physical.Shooter.HOOD_ACCELERATION_RAD_S;
        hoodVelocityRadPerSec += acceleration * dt
                - Constants.Physical.Shooter.HOOD_FRICTION_COEFFICIENT * hoodVelocityRadPerSec * dt;
        hoodPositionRad += hoodVelocityRadPerSec * dt;
    }

    private void updateFlywheel(double dt) {
        double acceleration = Math.signum(flywheelWantedVelocity - flywheelVelocity)
                * Constants.Physical.Shooter.SHOOTER_FLYWHEEL_ACCELERATION_RAD_S;
        flywheelVelocity += acceleration * dt
                - Constants.Physical.Shooter.SHOOTER_FRICTION_COEFFICIENT * flywheelVelocity * dt;
    }

    @Override
    public double getFlywheelCurrent() {
        return 0.0;
    }

    @Override
    public double getFlywheelAcceleration() {
        return 0;
    }

    @Override
    public void setFlywheelPercent(double percent) {

    }

    @Override
    public void zeroTurretToEncoder() {

    }

    @Override
    public void updateInputs() {
        double dt = Globals.loopPeriodSecs;
        int numSteps = Math.max(1, (int) Math.floor(dt / Constants.Simulation.closedLoopSimResolution));
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
        updateHood(dt);
        updateFlywheel(dt);

        double turretRad = getTurretAngle().getRadians();
        double hoodRad = getHoodAngle().getRadians();

        Pose3d shooterPose = new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, turretRad));
        Logger.recordOutput("Sim/shooter pose3d", shooterPose);

        Rotation3d hoodRotation = new Rotation3d(0.0, -hoodRad, turretRad);
        Translation3d PIVOT_OFFSET_HOOD = new Translation3d(0.0, -0.06, 0.01);

        double rx = hoodRotation.getX();
        double ry = hoodRotation.getY();
        double rz = hoodRotation.getZ();

        double cx = Math.cos(rx), sx = Math.sin(rx);
        double cy = Math.cos(ry), sy = Math.sin(ry);
        double cz = Math.cos(rz), sz = Math.sin(rz);

        double x1 = PIVOT_OFFSET_HOOD.getX();
        double y1 = PIVOT_OFFSET_HOOD.getY() * cx - PIVOT_OFFSET_HOOD.getZ() * sx;
        double z1 = PIVOT_OFFSET_HOOD.getY() * sx + PIVOT_OFFSET_HOOD.getZ() * cx;

        double x2 = x1 * cy + z1 * sy;
        double y2 = y1;
        double z2 = -x1 * sy + z1 * cy;

        double x3 = x2 * cz - y2 * sz;
        double y3 = x2 * sz + y2 * cz;
        double z3 = z2;

        Translation3d rotatedPivotHood = new Translation3d(x3, y3, z3);
        Translation3d correctionHood = new Translation3d(
                PIVOT_OFFSET_HOOD.getX() - rotatedPivotHood.getX(),
                PIVOT_OFFSET_HOOD.getY() - rotatedPivotHood.getY(),
                PIVOT_OFFSET_HOOD.getZ() - rotatedPivotHood.getZ());

        Pose3d hoodPose = new Pose3d(correctionHood, hoodRotation);
        Logger.recordOutput("Sim/hood pose3d", hoodPose);

        Logger.recordOutput("Sim/hood pivot_rotated_x", rotatedPivotHood.getX());
        Logger.recordOutput("Sim/hood pivot_rotated_y", rotatedPivotHood.getY());
        Logger.recordOutput("Sim/hood pivot_rotated_z", rotatedPivotHood.getZ());
        Logger.recordOutput("Sim/hood pivot_correction_x", correctionHood.getX());
        Logger.recordOutput("Sim/hood pivot_correction_y", correctionHood.getY());
        Logger.recordOutput("Sim/hood pivot_correction_z", correctionHood.getZ());
        double corrMag = Math.sqrt(correctionHood.getX() * correctionHood.getX()
                + correctionHood.getY() * correctionHood.getY()
                + correctionHood.getZ() * correctionHood.getZ());
        Logger.recordOutput("Sim/hood pivot_correction_magnitude", corrMag);
    }
}