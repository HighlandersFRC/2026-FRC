package frc.robot.tools.logging;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class ShiftManager {

    private static ShiftManager instance;

    public static ShiftManager getInstance() {
        if (instance == null) {
            instance = new ShiftManager();
        }
        return instance;
    }

    private char activeAutoWinner = 'U';
    private char fmsAutoWinner = 'U';
    private char humanAutoWinner = 'U';

    private boolean hasGameData = false;
    private boolean humanOverride = false;

    private final SendableChooser<String> autoWinnerChooser = new SendableChooser<>();

    private ShiftManager() {

        autoWinnerChooser.setDefaultOption("Neutral", "U");
        autoWinnerChooser.addOption("Red Won Auto", "R");
        autoWinnerChooser.addOption("Blue Won Auto", "B");

        SmartDashboard.putData("ShiftManager/AutoWinnerChooser", autoWinnerChooser);
    }

    public void update() {

        // Input from the FMS
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData != null && gameData.length() > 0) {

            char fmsVal = gameData.charAt(0);

            if (!hasGameData && (fmsVal == 'R' || fmsVal == 'B')) {

                fmsAutoWinner = fmsVal;
                activeAutoWinner = fmsVal;
                hasGameData = true;

                SmartDashboard.putString("ShiftManager/AutoWinnerFromFMS",
                        fmsVal == 'R' ? "Red" : "Blue");
            }
        }

        // Human Input
        String selected = autoWinnerChooser.getSelected();

        if (selected != null) {

            char humanVal = selected.charAt(0);

            if (humanVal != humanAutoWinner) {

                humanAutoWinner = humanVal;

                if (humanVal != 'U') {
                    humanOverride = true;
                    activeAutoWinner = humanVal;
                }
            }
        }

        // Log
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        int shift = getCurrentShift();
        boolean ourShift = isOurShift();
        double matchTime = getMatchTimeRemaining();
        double shiftTimeRemaining = getShiftTimeRemaining();
        double timeUntilNextShift = getTimeUntilNextShift();

        Logger.recordOutput("ShiftManager/HasGameData", hasGameData);
        Logger.recordOutput("ShiftManager/HumanOverride", humanOverride);
        Logger.recordOutput("ShiftManager/FMSAutoWinner", String.valueOf(fmsAutoWinner));
        Logger.recordOutput("ShiftManager/HumanAutoWinner", String.valueOf(humanAutoWinner));
        Logger.recordOutput("ShiftManager/ActiveAutoWinner", String.valueOf(activeAutoWinner));

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
        Logger.recordOutput("ShiftManager/TimeUntilNextShift", timeUntilNextShift);

        handleRumble(timeUntilNextShift);
    }

    private void handleRumble(double timeUntilNextShift) {

        double rumbleValue = 0;

        if (timeUntilNextShift <= 5 && timeUntilNextShift > 0) {

            double fractional = timeUntilNextShift - Math.floor(timeUntilNextShift);

            if (fractional < 0.5) {
                rumbleValue = 1.0;
            }
        }

        OI.driverController.setRumble(RumbleType.kBothRumble, rumbleValue);
    }

    public Optional<Alliance> getAutoWinner() {

        if (activeAutoWinner == 'R')
            return Optional.of(Alliance.Red);

        if (activeAutoWinner == 'B')
            return Optional.of(Alliance.Blue);

        return Optional.empty();
    }

    public int getCurrentShift() {

        double time = DriverStation.getMatchTime();

        if (time > 130)
            return 0;

        if (time <= 30)
            return 0;

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

    public double getTimeUntilNextShift() {

        double time = DriverStation.getMatchTime();

        if (time > 130)
            return time - 130;

        if (time > 105)
            return time - 105;

        if (time > 80)
            return time - 80;

        if (time > 55)
            return time - 55;

        if (time > 30)
            return time - 30;

        return Double.POSITIVE_INFINITY;
    }

    public boolean isOurShift() {

        double time = DriverStation.getMatchTime();

        if (time > 130)
            return true;

        if (time <= 30)
            return true;

        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isEmpty())
            return false;

        Optional<Alliance> winnerOpt = getAutoWinner();

        if (winnerOpt.isEmpty())
            return false;

        Alliance ourAlliance = allianceOpt.get();
        boolean autoWinnerIsUs = winnerOpt.get() == ourAlliance;

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
}