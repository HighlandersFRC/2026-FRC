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
        pigeonConfig.MountPose.MountPoseYaw = 177.6891326904297;
        pigeonConfig.MountPose.MountPosePitch = -0.8074852228164673;
        pigeonConfig.MountPose.MountPoseRoll = -1.2519879341125488;

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

    public Pigeon2 getPigeon() {
        return pigeon;
    }

    public double getAngularVelocityZWorldDegPerSec() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getAngularVelocityZDeviceDegPerSec() {
        return pigeon.getAngularVelocityZDevice().getValueAsDouble();
    }

    public double getAngularVelocityXDeviceDegPerSec() {
        return pigeon.getAngularVelocityXDevice().getValueAsDouble();
    }

    public double getAngularVelocityYDeviceDegPerSec() {
        return pigeon.getAngularVelocityYDevice().getValueAsDouble();
    }

    public double getAngularVelocityXWorldDegPerSec() {
        return pigeon.getAngularVelocityXWorld().getValueAsDouble();
    }

    public double getAngularVelocityYWorldDegPerSec() {
        return pigeon.getAngularVelocityYWorld().getValueAsDouble();
    }

    public double getPitchDegrees() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getPitchAdjustedDegrees() {
        return getPitchDegrees() - pitchOffset;
    }

    public double getRollDegrees() {
        return pigeon.getRoll().getValueAsDouble();
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

    public boolean isOnline() {
        return pigeon.isConnected();
    }
}
