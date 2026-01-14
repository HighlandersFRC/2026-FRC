package frc.robot.tools.math;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class PhysicsModel {
    /**
     * Calculates the initial velocity vector required for a projectile to travel
     * from an initial position to a final position while reaching a specified
     * maximum height.
     *
     * @param initialPosition The starting position of the projectile as a
     *                        {@link Translation3d}.
     * @param finalPosition   The target position of the projectile as a
     *                        {@link Translation3d}.
     * @param maxHeight       The maximum height the projectile should reach during
     *                        its trajectory.
     * @return A {@link Vector3D} representing the initial velocity vector (vxi,
     *         vyi, vzi)
     *         required to achieve the specified trajectory.
     * @throws IllegalArgumentException If the calculated trajectory is not
     *                                  physically possible
     *                                  (e.g., maxHeight is less than the initial or
     *                                  final z-coordinate).
     */
    public static Translation3d getHeightBoundTrajectory(
            Translation3d initialPosition,
            Translation3d finalPosition, double maxHeight) {
        if (maxHeight <= initialPosition.getZ() || maxHeight <= finalPosition.getZ()) {
            throw new IllegalArgumentException("Max height must be greater than both initial and final z-coordinates.");
        }
        double vzi = Math.sqrt(2 * Constants.G * (maxHeight - initialPosition.getZ()));
        double dx = finalPosition.getX() - initialPosition.getX();
        double dy = finalPosition.getY() - initialPosition.getY();
        double timeUp = vzi / Constants.G;
        double timeDown = Math.sqrt(2 * (maxHeight - finalPosition.getZ()) / Constants.G);
        double totalTime = timeUp + timeDown;
        double vxi = dx / totalTime;
        double vyi = dy / totalTime;
        return new Translation3d(vxi, vyi, vzi);
    }
}
