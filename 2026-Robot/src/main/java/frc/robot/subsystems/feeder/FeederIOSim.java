package frc.robot.subsystems.feeder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import org.littletonrobotics.junction.Logger;

class FeederIOSim implements FeederIO {
    private double dyeRotorVelocity = 0.0;
    private double dyeRotorWantedVelocity = 0.0;
    private double dyeRotorAngleRad = 0.0;

    @Override
    public void setDyeRotorPercent(double percent) {
        dyeRotorWantedVelocity = percent * Constants.Physical.Feeder.DYE_ROTOR_MAX_SPEED_MPS;
    }

    @Override
    public double getDyeRotorRPM() {
        double radius = Constants.Physical.Feeder.DYE_ROTOR_WHEEL_DIAMETER_M / 2.0;
        if (radius <= 0.0) {
            return 0.0;
        }
        double rotationsPerSecond = dyeRotorVelocity / (2.0 * Math.PI * radius);
        return rotationsPerSecond * 60.0;
    }

    @Override
    public void updateInputs(FeederState systemState) {
        double dt = Globals.loopPeriodSecs;
        double sign = Math.signum(dyeRotorWantedVelocity - dyeRotorVelocity);
        double accelCmd = sign * Constants.Physical.Feeder.DYE_ROTOR_ACCELERATION_MPS2;
        double friction = Constants.Physical.Feeder.DYE_ROTOR_FRICTION_COEFFICIENT * dyeRotorVelocity;
        double netAcc = accelCmd - friction;
        dyeRotorVelocity += netAcc * dt;
        double maxSpeed = Constants.Physical.Feeder.DYE_ROTOR_MAX_SPEED_MPS;
        if (Math.abs(dyeRotorVelocity) > maxSpeed) {
            dyeRotorVelocity = Math.copySign(maxSpeed, dyeRotorVelocity);
        }
        double distance = dyeRotorVelocity * dt;
        double radius = Constants.Physical.Feeder.DYE_ROTOR_WHEEL_DIAMETER_M / 2.0;
        if (radius > 0.0) {
            dyeRotorAngleRad += distance / radius;
        }
        Logger.recordOutput("Sim/feeder dyeRotorVelocity_mps", dyeRotorVelocity);
        Logger.recordOutput("Sim/feeder dyeRotorAngle_rad", dyeRotorAngleRad);
        Pose3d pose = new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, -dyeRotorAngleRad));
        Logger.recordOutput("Sim/feeder pose3d", pose);
    }

    @Override
    public void setDyeRotorTorque(double amps, double maxPercent) {
        double pct;
        if (amps > 0.0) {
            pct = Math.abs(maxPercent);
        } else if (amps < 0.0) {
            pct = -Math.abs(maxPercent);
        } else {
            pct = 0.0;
        }
        dyeRotorWantedVelocity = pct * Constants.Physical.Feeder.DYE_ROTOR_MAX_SPEED_MPS;
    }
}