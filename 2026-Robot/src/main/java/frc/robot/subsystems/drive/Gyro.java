package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Gyro {
    private final Pigeon2 pigeon = new Pigeon2(0, "Canivore");

    private final Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();

    private double pitchOffset = 0.0;

    public void init() {
        // Set the mount pose configuration for the IMU // IN CORRECT ORDER NOW
        pigeonConfig.MountPose.MountPoseYaw = -90.16925048828125;
        pigeonConfig.MountPose.MountPosePitch = 0.9936295747756958;
        pigeonConfig.MountPose.MountPoseRoll = -179.11083984375;

        pigeon.getConfigurator().apply(pigeonConfig);

        zeroYaw();
        setPitchOffsetDegrees(getPitchDegrees());
    }

    public void zeroYaw() {
        setYaw(0.0);
    }

    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    public double getYawDegrees() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getYawRadians() {
        return Math.toRadians(getYawDegrees());
    }

    public Rotation2d getYaw() {
        return new Rotation2d(getYawRadians());
    }

    public double getAngularVelocityZWorldRadPerSec() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getAngularVelocityZDeviceDegPerSec() {
        return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
    }

    public double getPitchDegrees() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getPitchAdjustedDegrees() {
        return getPitchDegrees() - pitchOffset;
    }

    public void setPitchOffsetDegrees(double offsetDeg) {
        pitchOffset = offsetDeg;
    }

    public Vector getLinearAccelGVector() {
        Vector v = new Vector();
        v.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        v.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        return v;
    }
}
