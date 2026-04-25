package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Globals {
    public static double loopPeriodSecs = 0.0;
    public static double prevTimeSecs = 0.0;
    public static double runTime = 0.0;
    public static double initTime = 0.0;
    public static String fieldSide = "red";
    public static Rotation2d turretAngle = new Rotation2d(0);
    public static double turretVelocity = 0.0;

    public static double loopAvgSumSecs = 0.0;
    public static long loopAvgCount = 0;
    public static boolean loopAvgActive = false;
    // Time when robot was enabled; used to delay averaging for a warmup period
    public static double loopAvgEnableTimestamp = 0.0;
    // Set to true once averaging has actually started (after the delay)
    public static boolean loopAvgStarted = false;
}
