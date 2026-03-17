package frc.robot.tools.tests;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Locale;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.tools.math.ShotCalculator;

public class ShotCalculatorTest {

    public static void writeSweepCSV() throws Exception {
        Pose2d turret = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

        final double targetXMin = 1.0;
        final double targetXMax = 8.0;
        final double targetXStep = 0.5;

        final double targetYMin = -3.0;
        final double targetYMax = 3.0;
        final double targetYStep = 0.5;

        final double[] speedMags = { 0.0, 0.33, 0.67 };

        final int headingSamples = 12; 

        final double[] omegas = { 0.0, Math.PI / 8.0, Math.PI / 4.0 };

        final int maxIter = 1000; 
        final double tol = 1e-4; 

        File logsDir = new File("/home/lvuser/logs");
        if (!logsDir.exists()) {
            logsDir.mkdirs();
        }

        File file = new File(logsDir, "shot_sweep.csv");

        try (PrintWriter pw = new PrintWriter(new FileWriter(file))) {
            pw.println("targetX,targetY,linearMag,headingRad,velVx,velVy,omega,converged,convergedIter,"
                    + "hoodDeg,flywheelRPM,turretDeg,distanceToTarget,timeOfFlight");

            for (double tx = targetXMin; tx <= targetXMax + 1e-9; tx += targetXStep) {
                for (double ty = targetYMin; ty <= targetYMax + 1e-9; ty += targetYStep) {
                    Translation2d target = new Translation2d(tx, ty);

                    for (double mag : speedMags) {
                        for (int h = 0; h < headingSamples; h++) {
                            double heading = (2.0 * Math.PI * h) / headingSamples;
                            double vx = mag * Math.cos(heading);
                            double vy = mag * Math.sin(heading);

                            for (double omega : omegas) {
                                ChassisSpeeds vel = new ChassisSpeeds(vx, vy, omega);

                                ShotCalculator.ShotSolution prev = null;
                                ShotCalculator.ShotSolution curr = null;
                                int convergedIter = -1;
                                boolean converged = false;

                                for (int iter = 1; iter <= maxIter; iter++) {
                                    ShotCalculator.setIterations(iter);
                                    try {
                                        curr = ShotCalculator.calculateHubShot(turret, target, vel);
                                    } catch (Exception ex) {
                                        curr = null;
                                    }

                                    if (curr == null) {
                                        prev = null;
                                        continue;
                                    }

                                    if (prev != null) {
                                        double dHood = Math
                                                .abs(curr.hoodAngle.getDegrees() - prev.hoodAngle.getDegrees());
                                        double dFly = Math.abs(curr.flywheelRPM - prev.flywheelRPM);
                                        double dTurret = Math
                                                .abs(curr.turretAngle.getDegrees() - prev.turretAngle.getDegrees());
                                        double dDist = Math.abs(curr.distanceToTarget - prev.distanceToTarget);
                                        double dTOF = Math.abs(curr.timeOfFlight - prev.timeOfFlight);

                                        double maxDiff = Math.max(Math.max(dHood, dFly),
                                                Math.max(Math.max(dTurret, dDist), dTOF));

                                        if (maxDiff <= tol) {
                                            converged = true;
                                            convergedIter = iter;
                                            break; 
                                        }
                                    }

                                    prev = curr;
                                } 

                                double hoodDeg = curr != null ? curr.hoodAngle.getDegrees() : Double.NaN;
                                double flyRPM = curr != null ? curr.flywheelRPM : Double.NaN;
                                double turretDeg = curr != null ? curr.turretAngle.getDegrees() : Double.NaN;
                                double dist = curr != null ? curr.distanceToTarget : Double.NaN;
                                double tof = curr != null ? curr.timeOfFlight : Double.NaN;

                                pw.printf(Locale.US,
                                        "%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%b,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                                        tx, 
                                        ty, 
                                        mag, 
                                        heading,
                                        vx, 
                                        vy, 
                                        omega, 
                                        converged,
                                        convergedIter, 
                                        hoodDeg,
                                        flyRPM,
                                        turretDeg,
                                        dist,
                                        tof);

                                pw.flush();
                            }
                        } 
                    } 
                } 
            } 
        }

        System.out.println("CSV written to /home/lvuser/logs/shot_sweep.csv");
    }
}