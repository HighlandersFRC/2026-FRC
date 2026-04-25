package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.tools.logging.BatteryLogger;
import frc.robot.tools.logging.TunableNumber;

class ShooterIOComp implements ShooterIO {
        private final TalonFX flywheelMaster = new TalonFX(Constants.CANInfo.FLYWHEEL_MASTER_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX flywheelFollower = new TalonFX(Constants.CANInfo.FLYWHEEL_SLAVE_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX turretMotor = new TalonFX(Constants.CANInfo.TURRET_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX hoodMotor = new TalonFX(Constants.CANInfo.HOOD_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

        private final VelocityVoltage flywheelControl = new VelocityVoltage(0.0).withUpdateFreqHz(1000)
                        .withLimitReverseMotion(true);
        private final VelocityTorqueCurrentFOC flywheelControl2 = new VelocityTorqueCurrentFOC(0.0)
                        .withUpdateFreqHz(1000)
                        .withLimitReverseMotion(true);

        private final CANcoder encoderOne = new CANcoder(Constants.CANInfo.TURRET_CANCODER_ONE_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final CANcoder encoderTwo = new CANcoder(Constants.CANInfo.TURRET_CANCODER_TWO_ID,
                        Constants.CANInfo.CANBUS_NAME);

        private LinearFilter filterTurret = LinearFilter.movingAverage(10);

        private TunableNumber turretP = new TunableNumber("Turret Position kP",
                        Constants.PIDConstants.Turret.kP0);
        private TunableNumber turretI = new TunableNumber("Turret Position kI",
                        Constants.PIDConstants.Turret.kI0);
        private TunableNumber turretD = new TunableNumber("Turret Position kD",
                        Constants.PIDConstants.Turret.kD0);
        private TunableNumber turretS = new TunableNumber("Turret Position kS",
                        Constants.PIDConstants.Turret.kS0);
        private TunableNumber turretV = new TunableNumber("Turret Position kV",
                        Constants.PIDConstants.Turret.kV0);
        private TunableNumber turretA = new TunableNumber("Turret Position kA",
                        Constants.PIDConstants.Turret.kA0);
        private TunableNumber turretVelocity = new TunableNumber("Turret Position Velocity", 20.0);
        private TunableNumber turretAcceleration = new TunableNumber("Turret Position Acceleration", 25.0);
        private TunableNumber turretFFScalar = new TunableNumber("Turret Feedforward Scalar",
                        Constants.PIDConstants.Turret.FF_SCALAR);
        // private double turretVelocity = 3.0;
        // private double turretAcceleration = 10.0;
        // private TunableNumber hoodP = new TunableNumber("Hood Position kP",
        // Constants.PIDConstants.Hood.kP0);
        // private TunableNumber hoodI = new TunableNumber("Hood Position kI",
        // Constants.PIDConstants.Hood.kI0);
        // private TunableNumber hoodD = new TunableNumber("Hood Position kD",
        // Constants.PIDConstants.Hood.kD0);
        // private TunableNumber hoodS = new TunableNumber("Hood Position kS",
        // Constants.PIDConstants.Hood.kS0);
        // private TunableNumber hoodG = new TunableNumber("Hood Position kG",
        // Constants.PIDConstants.Hood.kG0);
        // private TunableNumber hoodCruiseVelocity = new TunableNumber("Hood Position
        // Velocity", 2.0);
        // private TunableNumber hoodAcceleration = new TunableNumber("Hood Position
        // Acceleration", 2.0);

        private double hoodCruiseVelocity = 2.0;
        private double hoodAcceleration = 2.0;
        private final double hoodProfileScalarFactor = 1.0;

        private final DynamicMotionMagicVoltage hoodMotionProfileRequest = new DynamicMotionMagicVoltage(0,
                        hoodCruiseVelocity,
                        hoodAcceleration);

        private TunableNumber flywheelP = new TunableNumber("Flywheel Position kP",
                        Constants.PIDConstants.Flywheel.kP0);
        private TunableNumber flywheelI = new TunableNumber("Flywheel Position kI",
                        Constants.PIDConstants.Flywheel.kI0);
        private TunableNumber flywheelD = new TunableNumber("Flywheel Position kD",
                        Constants.PIDConstants.Flywheel.kD0);
        private TunableNumber flywheelS = new TunableNumber("Flywheel Position kS",
                        Constants.PIDConstants.Flywheel.kS0);
        private TunableNumber flywheelV = new TunableNumber("Flywheel Position kV",
                        Constants.PIDConstants.Flywheel.kV0);
        private TunableNumber flyWheelA = new TunableNumber("Flywheel Position kA",
                        Constants.PIDConstants.Flywheel.kA0);

        private TunableNumber flywheelP1 = new TunableNumber("Flywheel Position kP1",
                        Constants.PIDConstants.Flywheel.kP1);
        private TunableNumber flywheelI1 = new TunableNumber("Flywheel Position kI1",
                        Constants.PIDConstants.Flywheel.kI1);
        private TunableNumber flywheelD1 = new TunableNumber("Flywheel Position kD1",
                        Constants.PIDConstants.Flywheel.kD1);
        private TunableNumber flywheelS1 = new TunableNumber("Flywheel Position kS1",
                        Constants.PIDConstants.Flywheel.kS1);
        private TunableNumber flywheelV1 = new TunableNumber("Flywheel Position kV1",
                        Constants.PIDConstants.Flywheel.kV1);
        private TunableNumber flyWheelA1 = new TunableNumber("Flywheel Position kA1",
                        Constants.PIDConstants.Flywheel.kA1);

        // private TunableNumber flywheelVelocity = new TunableNumber("Flywheel Position
        // Velocity", 2.0);
        // private TunableNumber flywheelAcceleration = new TunableNumber("Flywheel
        // Position Acceleration", 2.0);

        public ShooterIOComp() {
                initializingTurret = true;
                initLoops = 0;
                // Hood Motor Configuration //TODO: Gotta tune all of the configs
                // System.out.println("slope: " + SLOPE);
                TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
                hoodConfig.Slot0.kP = Constants.PIDConstants.Hood.kP0;
                hoodConfig.Slot0.kI = Constants.PIDConstants.Hood.kI0;
                hoodConfig.Slot0.kD = Constants.PIDConstants.Hood.kD0;
                hoodConfig.Slot0.kS = Constants.PIDConstants.Hood.kS0;
                hoodConfig.Slot0.kG = Constants.PIDConstants.Hood.kG0;
                hoodConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                hoodConfig.MotionMagic.MotionMagicAcceleration = Units
                                .radiansToRotations(Constants.Physical.Shooter.HOOD_ACCELERATION_RAD_S);
                hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Units
                                .radiansToRotations(Constants.Physical.Shooter.HOOD_MAX_SPEED_RAD_S);
                hoodConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.HOOD_ENCODER_TO_MECHANISM_GEAR_RATIO;
                hoodConfig.Feedback.RotorToSensorRatio = Constants.Ratios.Shooter.HOOD_MOTOR_TO_ENCODER_GEAR_RATIO;
                hoodConfig.CurrentLimits.StatorCurrentLimit = 10;
                hoodConfig.CurrentLimits.SupplyCurrentLimit = 10;
                hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                hoodConfig.Feedback.FeedbackRemoteSensorID = Constants.CANInfo.HOOD_CANCODER_ID;
                hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                                .radiansToRotations(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS
                                                - Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS);
                hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                hoodMotor.getConfigurator().apply(hoodConfig);
                hoodMotor.setNeutralMode(NeutralModeValue.Brake);
                hoodMotor.setPosition(0.0);

                // Flywheel Configuration
                TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
                flywheelConfig.Slot0.kP = Constants.PIDConstants.Flywheel.kP0;
                flywheelConfig.Slot0.kI = Constants.PIDConstants.Flywheel.kI0;
                flywheelConfig.Slot0.kD = Constants.PIDConstants.Flywheel.kD0;
                flywheelConfig.Slot0.kS = Constants.PIDConstants.Flywheel.kS0;
                flywheelConfig.Slot0.kV = Constants.PIDConstants.Flywheel.kV0;
                flywheelConfig.Slot0.kA = Constants.PIDConstants.Flywheel.kA0;
                flywheelConfig.Slot1.kP = Constants.PIDConstants.Flywheel.kP1;
                flywheelConfig.Slot1.kI = Constants.PIDConstants.Flywheel.kI1;
                flywheelConfig.Slot1.kD = Constants.PIDConstants.Flywheel.kD1;
                flywheelConfig.Slot1.kS = Constants.PIDConstants.Flywheel.kS1;
                flywheelConfig.Slot1.kV = Constants.PIDConstants.Flywheel.kV1;
                flywheelConfig.Slot1.kA = Constants.PIDConstants.Flywheel.kA1;
                flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
                flywheelConfig.CurrentLimits.StatorCurrentLimit = 70;
                flywheelConfig.CurrentLimits.SupplyCurrentLimit = 70;
                flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
                flywheelConfig.Voltage.PeakReverseVoltage = 0.0;
                flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                flywheelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                flywheelConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                // flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                flywheelMaster.getConfigurator().apply(flywheelConfig);
                flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
                // flywheelMaster.getVelocity().setUpdateFrequency(100);

                TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
                flywheelFollowerConfig.Slot0.kP = Constants.PIDConstants.Flywheel.kP0;
                flywheelFollowerConfig.Slot0.kI = Constants.PIDConstants.Flywheel.kI0;
                flywheelFollowerConfig.Slot0.kD = Constants.PIDConstants.Flywheel.kD0;
                flywheelFollowerConfig.Slot0.kS = Constants.PIDConstants.Flywheel.kS0;
                flywheelFollowerConfig.Slot0.kV = Constants.PIDConstants.Flywheel.kV0;
                flywheelFollowerConfig.Slot0.kA = Constants.PIDConstants.Flywheel.kA0;
                flywheelFollowerConfig.Slot1.kP = Constants.PIDConstants.Flywheel.kP1;
                flywheelFollowerConfig.Slot1.kI = Constants.PIDConstants.Flywheel.kI1;
                flywheelFollowerConfig.Slot1.kD = Constants.PIDConstants.Flywheel.kD1;
                flywheelFollowerConfig.Slot1.kS = Constants.PIDConstants.Flywheel.kS1;
                flywheelFollowerConfig.Slot1.kV = Constants.PIDConstants.Flywheel.kV1;
                flywheelFollowerConfig.Slot1.kA = Constants.PIDConstants.Flywheel.kA1;
                flywheelFollowerConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                flywheelFollowerConfig.Feedback.RotorToSensorRatio = 1.0;
                flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 70;
                flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 70;
                flywheelFollowerConfig.Voltage.PeakForwardVoltage = 12.0;
                flywheelFollowerConfig.Voltage.PeakReverseVoltage = 0.0;
                flywheelFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                flywheelFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                flywheelFollowerConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                flywheelFollowerConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                flywheelFollowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                flywheelFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                // flywheelFollowerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
                flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

                // Turret Motor Configuration
                TalonFXConfiguration turretConfig = new TalonFXConfiguration();
                turretConfig.Slot0.kP = Constants.PIDConstants.Turret.kP0;
                turretConfig.Slot0.kI = Constants.PIDConstants.Turret.kI0;
                turretConfig.Slot0.kD = Constants.PIDConstants.Turret.kD0;
                turretConfig.Slot0.kS = Constants.PIDConstants.Turret.kS0;
                turretConfig.Slot0.kV = Constants.PIDConstants.Turret.kV0;
                turretConfig.MotionMagic.MotionMagicAcceleration = turretAcceleration.get();
                turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretVelocity.get();
                turretConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.TURRET_GEAR_RATIO;
                turretConfig.Feedback.RotorToSensorRatio = 1.0;
                turretConfig.CurrentLimits.StatorCurrentLimit = 67;
                turretConfig.CurrentLimits.SupplyCurrentLimit = 67;
                turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                turretMotor.getConfigurator().apply(turretConfig);
                turretMotor.setNeutralMode(NeutralModeValue.Coast);

                // CANcoder Configuration
                CANcoderConfiguration encoderOneConfig = new CANcoderConfiguration();
                encoderOneConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                encoderOneConfig.MagnetSensor.MagnetOffset = -0.36962890625; // TODO: Try calculating offset from
                                                                             // previous zero
                                                                             // data
                encoderOneConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
                encoderOne.getConfigurator().apply(encoderOneConfig);
                CANcoderConfiguration encoderTwoConfig = new CANcoderConfiguration();
                encoderTwoConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
                encoderTwoConfig.MagnetSensor.MagnetOffset = -0.349609375;
                encoderTwoConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
                encoderTwo.getConfigurator().apply(encoderTwoConfig);

                turretMotor.setPosition(Units.radiansToRotations(_getRelativeTurretAngleRadians()));
                // turretMotor.setPosition(0.0);
        }

        private double getTurretHubVelocityFieldCentric() {
                double x = RobotState.getInstance().getEstimatedPose().getX();
                double y = RobotState.getInstance().getEstimatedPose().getY();
                double dx = RobotState.getInstance().getFieldVelocity().vxMetersPerSecond;
                double dy = RobotState.getInstance().getFieldVelocity().vyMetersPerSecond;
                return ((x * dy) + (y * dx)) / ((x * x) + (y * y)); // might be negative idk
        }

        private double getTurretAngularVelocity() {
                return -RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;
        }

        private double getADjustedTurretFeedForward() {
                return getTurretAngularVelocity() + getTurretHubVelocityFieldCentric();
        }

        @Override
        public Rotation2d getHoodAngle() {
                return Constants.SetPoints.Hood
                                .motorAngleToHoodAngle(
                                                Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble())); // TODO:
                                                                                                                       // try
                                                                                                                       // getLatencyCompensatedValueAsDouble()
        }

        @Override
        public void zeroTurretToEncoder() {
                // turretMotor.setPosition(Units.radiansToRotations(getRelativeTurretAngleRadians()));
        }

        @Override
        public Rotation2d getTurretAngle() {
                return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
        }

        @Override
        public double getFlywheelRPM() {
                return flywheelMaster.getVelocity().getValueAsDouble() * 60.0;
        }

        @Override
        public void moveHoodToAngle(Rotation2d angle) {
                // Logger.recordOutput("Shooter/Hood target angle", angle.getDegrees());
                if (angle.getRadians() > Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS) {
                        angle = Rotation2d.fromRadians(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS);
                }
                if (angle.getRadians() < Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS) {
                        angle = Rotation2d.fromRadians(Constants.SetPoints.Hood.HOOD_MIN_ANGLE_RADIANS);
                }
                double wantedAngle = Constants.SetPoints.Hood.hoodAngleToMotorAngle(angle).getRotations();
                hoodMotor.setControl(this.hoodMotionProfileRequest
                                .withPosition(
                                                wantedAngle)
                                .withVelocity(hoodCruiseVelocity * hoodProfileScalarFactor)
                                .withAcceleration(hoodAcceleration * hoodProfileScalarFactor)
                                .withSlot(0));
        }

        @Override
        public void setTurretAngle(double angle) {
                double adjustedAngle = angle;
                if (Math.abs(OI.getOperatorRightX()) > 0.1 ||
                                Math.abs(OI.getOperatorRightY()) > 0.3) {
                        adjustedAngle -= ((Math.toRadians(15.0)) * OI.getOperatorRightX());
                }
                Logger.recordOutput("Shooter/Adjusted Turret Setpoint", Math.toDegrees(adjustedAngle));
                turretMotor.setControl(
                                new DynamicMotionMagicVoltage(Units.radiansToRotations(
                                                adjustedAngle),
                                                turretVelocity.get(),
                                                turretAcceleration.get())
                                                .withFeedForward(
                                                                turretFFScalar.get() * getADjustedTurretFeedForward()));
        }

        @Override
        public void setFlywheelRPM(double rpm) {
                // Logger.recordOutput("Shooter/Goal flywheel RPM", rpm);
                double adjustedRPM = rpm;
                if (Math.abs(OI.getOperatorLeftY()) > 0.1 || Math.abs(OI.getOperatorLeftX()) > 0.3) {
                        adjustedRPM -= 200.0 * OI.getOperatorLeftY();
                }
                Logger.recordOutput("Shooter/Adjusted RPM Setpoint", adjustedRPM);

                double velocitySetpoint = adjustedRPM / 60.0;
                // Logger.recordOutput("Flywheel master setpoint",
                // flywheelMaster.getClosedLoopReference().getValueAsDouble() * 60.0);
                // Logger.recordOutput("Flywheel slave setpoint",
                // flywheelFollower.getClosedLoopReference().getValueAsDouble() * 60.0);

                // if (adjustedRPM - getFlywheelRPM() > 200.0) {
                // Logger.recordOutput("Shooter/Shoot Type", "BANG BANG");
                // flywheelMaster.setControl(

                // flywheelControl.withVelocity(velocitySetpoint).withSlot(1).withEnableFOC(true));
                // flywheelFollower.setControl(

                // flywheelControl.withVelocity(velocitySetpoint).withSlot(1).withEnableFOC(true));

                // } else {
                Logger.recordOutput("Shooter/Shoot Type", "PID");

                Logger.recordOutput("Shooter/Update Hz", flywheelControl2.UpdateFreqHz);
                // flywheelMaster.setControl(
                // flywheelControl.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true)
                // .withLimitReverseMotion(true));
                // flywheelFollower.setControl(
                // flywheelControl.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true));

                flywheelMaster.setControl(
                                flywheelControl2.withVelocity(velocitySetpoint).withSlot(1));
                flywheelFollower.setControl(
                                flywheelControl2.withVelocity(velocitySetpoint).withSlot(1));

                // }
        }

        private double _getRelativeTurretAngleRadians() {
                // double aMeas = encoderOne.getAbsolutePosition().getValueAsDouble();
                // double bMeas = encoderTwo.getAbsolutePosition().getValueAsDouble();
                double aMeas = encoderTwo.getAbsolutePosition().getValueAsDouble();
                double bMeas = encoderOne.getAbsolutePosition().getValueAsDouble();

                double k1 = Constants.Physical.Shooter.TURRET_PULLEY_0_TOOTH_COUNT
                                / Constants.Physical.Shooter.TURRET_PULLEY_1_TOOTH_COUNT;

                double k2 = (Constants.Physical.Shooter.TURRET_GEAR_1_TOOTH_COUNT
                                / Constants.Physical.Shooter.TURRET_GEAR_2_TOOTH_COUNT) * k1;

                double minTheta = Units.radiansToRotations(
                                Constants.SetPoints.Turret.TURRET_MIN_ANGLE_RADIANS);
                double maxTheta = Units.radiansToRotations(
                                Constants.SetPoints.Turret.TURRET_MAX_ANGLE_RADIANS);

                double bestTheta = 0.0;
                double bestError = Double.POSITIVE_INFINITY;

                // Compute reasonable bounds on n
                int nMin = (int) Math.floor(k1 * minTheta + aMeas - 1);
                int nMax = (int) Math.ceil(k1 * maxTheta + aMeas);

                for (int n = nMin; n <= nMax; n++) {
                        double theta = (aMeas + n) / k1;

                        if (theta < minTheta || theta > maxTheta) {
                                continue;
                        }

                        double bPred = wrap(k2 * theta);
                        double err = wrapDiff(bMeas, bPred);

                        double error = err * err;

                        if (error < bestError) {
                                bestError = error;
                                bestTheta = theta;
                        }
                }

                return Units.rotationsToRadians(bestTheta);
        }

        @Override
        public double getRelativeTurretAngleRadians() {
                return Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());
        }

        private double wrap(double x) {
                return x - Math.floor(x);
        }

        private double wrapDiff(double a, double b) {
                double d = a - b;
                if (d > 0.5)
                        d -= 1.0;
                if (d < -0.5)
                        d += 1.0;
                return d;
        }

        @Override
        public void setHoodAngle(Rotation2d angle) { // DON'T USE
                // hoodMotor.setPosition(angle.getRotations());
        }

        @Override
        public void setFlywheelPercent(double percent) {
                flywheelMaster.set(percent);
                flywheelFollower.set(-percent);
        }

        @Override
        public double getFlywheelCurrent() {
                return flywheelMaster.getStatorCurrent().getValueAsDouble();
        }

        @Override
        public double getFlywheelAcceleration() {
                return flywheelMaster.getAcceleration().getValueAsDouble();
        }

        @Override
        public double getHoodCurrent() {
                return hoodMotor.getStatorCurrent().getValueAsDouble();
        }

        @Override
        public double getTurretCurrent() {
                return turretMotor.getStatorCurrent().getValueAsDouble();
        }

        private boolean initializingTurret;
        private int initLoops;
        private ArrayList<Double> firstTurretAngles = new ArrayList<>();
        private int numberSkips;
        private final BatteryLogger batteryLogger = BatteryLogger.getInstance();

        @Override
        public void updateInputs() {
                Globals.turretAngle = getTurretAngle();
                // Logger.recordOutput("Shooter/Turret vel unfiltered",
                // Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()));
                filterTurret.calculate(Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()));
                Globals.turretVelocity = filterTurret.lastValue();
                Logger.recordOutput("Shooter/Turret Feedforward", getADjustedTurretFeedForward());
                Logger.recordOutput("Shooter/Turret FeedForward Translational", getTurretHubVelocityFieldCentric());
                Logger.recordOutput("Shooter/Turret FeedForward Rotational", getTurretAngularVelocity());
                batteryLogger.reportCurrentUsage("Shooter Flywheel/Master",
                                flywheelMaster.getSupplyCurrent().getValueAsDouble());
                batteryLogger.reportCurrentUsage("Shooter Flywheel/Follower",
                                flywheelFollower.getSupplyCurrent().getValueAsDouble());

                batteryLogger.reportCurrentUsage("Shooter Hood Motor", hoodMotor.getSupplyCurrent().getValueAsDouble());

                batteryLogger.reportCurrentUsage("Shooter/Turret Motor",
                                turretMotor.getSupplyCurrent().getValueAsDouble());

                // Logger.recordOutput("Testing/initializingTurret", initializingTurret);
                // Logger.recordOutput("Testing/initLoops", initLoops);
                // Logger.recordOutput("Testing/firstTurretAngles",
                // firstTurretAngles.toString());
                // Logger.recordOutput("Testing/numberSkips", numberSkips);
                Logger.recordOutput("Shooter/Relative Turret Angle",
                                Math.toDegrees(_getRelativeTurretAngleRadians()));
                Logger.recordOutput("Shooter/Motor Turret Angle",
                                Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble()));
                Logger.recordOutput("Shooter/Turret Error Degrees",
                                turretMotor.getClosedLoopError().getValueAsDouble() * 360.0);

                Logger.recordOutput("Online/Hood Motor Online", hoodMotor.isConnected());
                Logger.recordOutput("Online/Turret Motor Online", turretMotor.isConnected());
                Logger.recordOutput("Online/Flywheel Master Motor Online", flywheelMaster.isConnected());
                Logger.recordOutput("Online/Flywheel Follower Motor Online", flywheelFollower.isConnected());

                Logger.recordOutput("Online/Encoder One Online", encoderOne.isConnected());
                Logger.recordOutput("Online/Encoder Two Online", encoderTwo.isConnected());
                Logger.recordOutput("Online/Hood CanCoder",
                                !hoodMotor.getFault_RemoteSensorDataInvalid().getValue());
                if (turretP.changed() || turretI.changed() || turretD.changed() ||
                                turretS.changed() || turretV.changed() || turretA.changed() || turretVelocity.changed()
                                || turretAcceleration.changed()) {
                        System.out.println("Updating Turret PID Constants");

                        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
                        turretConfig.Slot0.kP = turretP.get();
                        turretConfig.Slot0.kI = turretI.get();
                        turretConfig.Slot0.kD = turretD.get();
                        turretConfig.Slot0.kS = turretS.get();
                        turretConfig.Slot0.kV = turretV.get();
                        turretConfig.MotionMagic.MotionMagicAcceleration = turretAcceleration.get();
                        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretVelocity.get();
                        turretConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.TURRET_GEAR_RATIO;
                        turretConfig.Feedback.RotorToSensorRatio = 1.0;
                        turretConfig.CurrentLimits.StatorCurrentLimit = 67;
                        turretConfig.CurrentLimits.SupplyCurrentLimit = 67;
                        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                        turretMotor.getConfigurator().apply(turretConfig);
                        turretMotor.setNeutralMode(NeutralModeValue.Coast);
                }
                // if (hoodP.changed() || hoodI.changed() || hoodD.changed() || hoodS.changed()
                // || hoodG.changed()) {
                // System.out.println("Updating Hood PID Constants");
                // TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
                // hoodConfig.Slot0.kP = hoodP.get();
                // hoodConfig.Slot0.kI = hoodI.get();
                // hoodConfig.Slot0.kD = hoodD.get();
                // hoodConfig.Slot0.kS = hoodS.get();
                // hoodConfig.Slot0.kG = hoodG.get();
                // hoodConfig.Feedback.SensorToMechanismRatio =
                // Constants.Ratios.Shooter.HOOD_ENCODER_TO_MECHANISM_GEAR_RATIO;
                // hoodConfig.Feedback.RotorToSensorRatio =
                // Constants.Ratios.Shooter.HOOD_MOTOR_TO_ENCODER_GEAR_RATIO;
                // hoodConfig.CurrentLimits.StatorCurrentLimit = 10;
                // hoodConfig.CurrentLimits.SupplyCurrentLimit = 10;
                // hoodConfig.Feedback.FeedbackSensorSource =
                // FeedbackSensorSourceValue.RemoteCANcoder;
                // hoodConfig.Feedback.FeedbackRemoteSensorID =
                // Constants.CANInfo.HOOD_CANCODER_ID;
                // hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                // hoodMotor.getConfigurator().apply(hoodConfig);
                // }
                if (flywheelP.changed() || flywheelI.changed() || flywheelD.changed() ||
                                flywheelS.changed() || flywheelV
                                                .changed()
                                || flyWheelA.changed()) {
                        System.out.println("Updating Flywheel PID Constants");
                        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
                        flywheelConfig.Slot0.kP = flywheelP.get();
                        flywheelConfig.Slot0.kI = flywheelI.get();
                        flywheelConfig.Slot0.kD = flywheelD.get();
                        flywheelConfig.Slot0.kS = flywheelS.get();
                        flywheelConfig.Slot0.kV = flywheelV.get();
                        flywheelConfig.Slot0.kA = flyWheelA.get();
                        flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                        flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
                        flywheelConfig.CurrentLimits.StatorCurrentLimit = 70;
                        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 70;
                        flywheelConfig.Voltage.PeakForwardVoltage = 32.0;
                        flywheelConfig.Voltage.PeakReverseVoltage = 0.0;
                        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                        flywheelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                        flywheelConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                        flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                        // flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                        flywheelMaster.getConfigurator().apply(flywheelConfig);
                        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);

                        TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
                        flywheelFollowerConfig.Slot0.kP = flywheelP.get();
                        flywheelFollowerConfig.Slot0.kI = flywheelI.get();
                        flywheelFollowerConfig.Slot0.kD = flywheelD.get();
                        flywheelFollowerConfig.Slot0.kS = flywheelS.get();
                        flywheelFollowerConfig.Slot0.kV = flywheelV.get();
                        flywheelFollowerConfig.Slot0.kA = flyWheelA.get();
                        flywheelFollowerConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                        flywheelFollowerConfig.Feedback.RotorToSensorRatio = 1.0;
                        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 70;
                        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 70;
                        flywheelFollowerConfig.Voltage.PeakForwardVoltage = 32.0;
                        flywheelFollowerConfig.Voltage.PeakReverseVoltage = 0.0;
                        flywheelFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                        flywheelFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                        flywheelFollowerConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                        flywheelFollowerConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                        flywheelFollowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                        flywheelFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                        // flywheelFollowerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                        flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
                        flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
                }

                if (flywheelP1.changed() || flywheelI1.changed() || flywheelD1.changed() ||
                                flywheelS1.changed() || flywheelV1
                                                .changed()
                                || flyWheelA1.changed()) {
                        System.out.println("Updating Flywheel PID Constants");
                        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
                        flywheelConfig.Slot1.kP = flywheelP1.get();
                        flywheelConfig.Slot1.kI = flywheelI1.get();
                        flywheelConfig.Slot1.kD = flywheelD1.get();
                        flywheelConfig.Slot1.kS = flywheelS1.get();
                        flywheelConfig.Slot1.kV = flywheelV1.get();
                        flywheelConfig.Slot1.kA = flyWheelA1.get();
                        flywheelConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                        flywheelConfig.Feedback.RotorToSensorRatio = 1.0;
                        flywheelConfig.CurrentLimits.StatorCurrentLimit = 70;
                        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 70;
                        flywheelConfig.Voltage.PeakForwardVoltage = 32.0;
                        flywheelConfig.Voltage.PeakReverseVoltage = 0.0;
                        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                        flywheelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                        flywheelConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                        flywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                        // flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                        flywheelMaster.getConfigurator().apply(flywheelConfig);
                        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);

                        TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
                        flywheelFollowerConfig.Slot1.kP = flywheelP1.get();
                        flywheelFollowerConfig.Slot1.kI = flywheelI1.get();
                        flywheelFollowerConfig.Slot1.kD = flywheelD1.get();
                        flywheelFollowerConfig.Slot1.kS = flywheelS1.get();
                        flywheelFollowerConfig.Slot1.kV = flywheelV1.get();
                        flywheelFollowerConfig.Slot1.kA = flyWheelA1.get();
                        flywheelFollowerConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Shooter.FLYWHEEL_GEAR_RATIO;
                        flywheelFollowerConfig.Feedback.RotorToSensorRatio = 1.0;
                        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 70;
                        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 70;
                        flywheelFollowerConfig.Voltage.PeakForwardVoltage = 32.0;
                        flywheelFollowerConfig.Voltage.PeakReverseVoltage = 0.0;
                        flywheelFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
                        flywheelFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                        flywheelFollowerConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                        flywheelFollowerConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                        flywheelFollowerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                        flywheelFollowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                        // flywheelFollowerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
                        flywheelFollower.getConfigurator().apply(flywheelFollowerConfig);
                        flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
                }
        }
}
