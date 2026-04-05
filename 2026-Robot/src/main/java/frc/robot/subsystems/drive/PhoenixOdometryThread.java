package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.Logger;

public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock();
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static final boolean isCANFD = new CANBus("*").isNetworkFD();
    private static PhoenixOdometryThread instance;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
    }

    @Override
    public void start() {
        if (timestampQueues.size() > 0) {
            super.start();
        }
    }

    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(50);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(50);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(50);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        Threads.setCurrentThreadPriority(true, 1);

        // Diagnostics for verifying 250 Hz operation
        long loopStartTime = System.nanoTime();
        int loopCount = 0;

        while (true) {
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / DriveConstants.odometryFrequency, phoenixSignals);
                } else {
                    Thread.sleep((long) (1000.0 / DriveConstants.odometryFrequency));
                    if (phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            Drive.odometryLock.lock();
            try {
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                Drive.odometryLock.unlock();
            }

            // Diagnostics: every 250 iterations (1 second at 250 Hz), calculate and log
            // frequency
            loopCount++;
            if (loopCount >= 250) {
                long currentTime = System.nanoTime();
                double elapsedSeconds = (currentTime - loopStartTime) / 1e9;
                double measuredFrequency = loopCount / elapsedSeconds;
                double periodMs = (elapsedSeconds / loopCount) * 1000;

                // Log to AdvantageKit
                Logger.recordOutput("Diagnostics/Odometry/FrequencyHz", measuredFrequency);
                Logger.recordOutput("Diagnostics/Odometry/PeriodMs", periodMs);

                loopStartTime = currentTime;
                loopCount = 0;
            }
        }
    }
}
