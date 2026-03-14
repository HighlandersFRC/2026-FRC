package frc.robot.tools.logging;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShiftManager {

    private static ShiftManager instance;

    public static ShiftManager getInstance() {
        if (instance == null) {
            instance = new ShiftManager();
        }
        return instance;
    }

    private char autoWinner = 'U';
    private boolean hasGameData = false;

    private ShiftManager() {
    }

    public void update() {

        if (!hasGameData) {
            String gameData = DriverStation.getGameSpecificMessage();

            if (gameData != null && gameData.length() > 0) {
                autoWinner = gameData.charAt(0);
                hasGameData = true;
            }
        }

        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        int shift = getCurrentShift();
        boolean ourShift = isOurShift();
        double matchTime = getMatchTimeRemaining();
        double shiftTimeRemaining = getShiftTimeRemaining();

        Logger.recordOutput("ShiftManager/HasGameData", hasGameData);
        Logger.recordOutput("ShiftManager/AutoWinnerChar", String.valueOf(autoWinner));
        Logger.recordOutput(
                "ShiftManager/AutoWinnerAlliance",
                getAutoWinner().map(Enum::name).orElse("Unknown"));

        Logger.recordOutput(
                "ShiftManager/Alliance",
                allianceOpt.map(Enum::name).orElse("Unknown"));

        Logger.recordOutput("ShiftManager/CurrentShift", shift);
        Logger.recordOutput("ShiftManager/IsOurShift", ourShift);

        Logger.recordOutput("ShiftManager/MatchTimeRemaining", matchTime);
        Logger.recordOutput("ShiftManager/ShiftTimeRemaining", shiftTimeRemaining);
    }

    public Optional<Alliance> getAutoWinner() {

        if (!hasGameData)
            return Optional.empty();

        if (autoWinner == 'R')
            return Optional.of(Alliance.Red);
        if (autoWinner == 'B')
            return Optional.of(Alliance.Blue);

        return Optional.empty();
    }

    public int getCurrentShift() {

        double time = DriverStation.getMatchTime();

        if (time > 130)
            return 0; // auto + first 10s teleop
        if (time <= 30)
            return 0; // endgame

        if (time <= 130 && time > 105)
            return 1;
        if (time <= 105 && time > 80)
            return 2;
        if (time <= 80 && time > 55)
            return 3;
        if (time <= 55 && time > 30)
            return 4;

        return 0;
    }

    public double getShiftTimeRemaining() {

        int shift = getCurrentShift();
        double time = DriverStation.getMatchTime();

        switch (shift) {
            case 1:
                return time - 105;
            case 2:
                return time - 80;
            case 3:
                return time - 55;
            case 4:
                return time - 30;
            default:
                return 0;
        }
    }

    public boolean isOurShift() {

        double time = DriverStation.getMatchTime();

        if (time > 130)
            return true;
        if (time <= 30)
            return true;

        if (!hasGameData)
            return false;

        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty())
            return false;

        Alliance ourAlliance = allianceOpt.get();

        boolean autoWinnerIsUs = (autoWinner == 'R' && ourAlliance == Alliance.Red) ||
                (autoWinner == 'B' && ourAlliance == Alliance.Blue);

        int shift = getCurrentShift();

        boolean loserActive;

        switch (shift) {
            case 1:
                loserActive = true;
                break;
            case 2:
                loserActive = false;
                break;
            case 3:
                loserActive = true;
                break;
            case 4:
                loserActive = false;
                break;
            default:
                return false;
        }

        boolean weLostAuto = !autoWinnerIsUs;

        return weLostAuto ? loserActive : !loserActive;
    }

    public double getMatchTimeRemaining() {
        return DriverStation.getMatchTime();
    }

    public boolean hasGameData() {
        return hasGameData;
    }
}
