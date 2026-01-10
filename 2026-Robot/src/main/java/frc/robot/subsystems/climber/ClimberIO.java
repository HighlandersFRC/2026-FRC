package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.Climber.ClimbState;

public interface ClimberIO {
	// Initialize hardware/sim
	void init();

	// Optional per-cycle updates (keep signature similar to Pivot IO)
	void updateInputs(ClimbState systemState);

	// Apply torque control on the climber motor
	void setTorque(double current, double maxPercent);

	// Read climber motor position (rotations)
	double getPosition();
}
