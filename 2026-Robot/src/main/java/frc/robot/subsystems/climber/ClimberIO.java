package frc.robot.subsystems.climber;

public interface ClimberIO {
    abstract void setPower(double amps, double percent);

    abstract void stop();

    abstract double getPosition();

    abstract double getSlaveCurrent();

    abstract double getMasterCurrent();

    abstract void update();
}
