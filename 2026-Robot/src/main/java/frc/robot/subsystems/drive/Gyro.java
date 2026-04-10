package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Gyro {
    private final Pigeon2 pigeon = new Pigeon2(0, "rio");
    private final Pigeon2 pigeon2 = new Pigeon2(1, "rio");

    private final Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    private final Pigeon2Configuration pigeon2Config = new Pigeon2Configuration();

    private double pitchOffset = 0.0;
    private double rollOffset = 0.0;

    public void init() {
        // Set the mount pose configuration for the IMU // IN CORRECT ORDER NOW
        pigeonConfig.MountPose.MountPoseYaw = 94.9007339477539;
        pigeonConfig.MountPose.MountPosePitch = 0.034411586821079254;
        pigeonConfig.MountPose.MountPoseRoll = 179.55764770507812;

        pigeon.getConfigurator().apply(pigeonConfig);

        pigeon2Config.MountPose.MountPoseYaw = -82.46958923339844;
        pigeon2Config.MountPose.MountPosePitch = 17.953596115112305;
        pigeon2Config.MountPose.MountPoseRoll = -179.22665405273438;

        pigeon2.getConfigurator().apply(pigeon2Config);

        zeroYaw();
        setPitchOffsetDegrees(getPitchDegrees());
        setRollOffsetDegrees(getRollDegrees());
    }

    public void zeroYaw() {
        setYaw(0.0);
    }

    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
        pigeon2.setYaw(degrees);
    }

    public double getYawDegrees() {
        if (!pigeon.isConnected()) {
            return pigeon2.getYaw().getValueAsDouble();
        }
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getYawRadians() {
        return Math.toRadians(getYawDegrees());
    }

    public Rotation2d getYaw() {
        return new Rotation2d(getYawRadians());
    }

    public Pigeon2 getPigeon() {
        if (!pigeon.isConnected()) {
            Logger.recordOutput("Robot/Gyro2Used", true);
            return pigeon2;
        }
        Logger.recordOutput("Robot/Gyro2Used", false);
        return pigeon;
    }

    public double getAngularVelocityZWorldDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
        }
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getAngularVelocityZDeviceDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityZDevice().getValueAsDouble();
        }
        return pigeon.getAngularVelocityZDevice().getValueAsDouble();
    }

    public double getAngularVelocityXDeviceDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityXDevice().getValueAsDouble();
        }
        return pigeon.getAngularVelocityXDevice().getValueAsDouble();
    }

    public double getAngularVelocityYDeviceDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityYDevice().getValueAsDouble();
        }
        return pigeon.getAngularVelocityYDevice().getValueAsDouble();
    }

    public double getAngularVelocityXWorldDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityXWorld().getValueAsDouble();
        }
        return pigeon.getAngularVelocityXWorld().getValueAsDouble();
    }

    public double getAngularVelocityYWorldDegPerSec() {
        if (!pigeon.isConnected()) {
            return pigeon2.getAngularVelocityYWorld().getValueAsDouble();
        }
        return pigeon.getAngularVelocityYWorld().getValueAsDouble();
    }

    public double getPitchDegrees() {
        if (!pigeon.isConnected()) {
            return pigeon2.getPitch().getValueAsDouble();
        }
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getPitchAdjustedDegrees() {
        return getPitchDegrees() - pitchOffset;
    }

    public double getRollDegrees() {
        if (!pigeon.isConnected()) {
            return pigeon2.getRoll().getValueAsDouble() - rollOffset;
        }
        return pigeon.getRoll().getValueAsDouble() - rollOffset;
    }

    public void setPitchOffsetDegrees(double offsetDeg) {
        pitchOffset = offsetDeg;
    }

    public void setRollOffsetDegrees(double offsetDeg) {
        rollOffset = offsetDeg;
    }

    public Vector getLinearAccelGVector() {
        if (!pigeon.isConnected()) {
            Vector v = new Vector();
            v.setI(pigeon2.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
            v.setJ(pigeon2.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
            return v;
        }
        Vector v = new Vector();
        v.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        v.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
        return v;
    }

    public boolean isOnline() {
        return pigeon.isConnected();
    }

    public double getPigeon2Yaw() {
        return pigeon2.getYaw().getValueAsDouble();
    }

    public double getPigeon1Yaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getPigeon2Roll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    public double getPigeon1Roll() {
        return pigeon.getRoll().getValueAsDouble();
    }

    public double getPigeon2Pitch() {
        return pigeon2.getPitch().getValueAsDouble();
    }

    public double getPigeon1Pitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public boolean is2Online() {
        return pigeon2.isConnected();
    }
}
