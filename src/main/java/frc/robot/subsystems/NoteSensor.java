package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIOConfig;

public class NoteSensor extends SubsystemBase {
    private final DigitalInput intakeBeamBreak;
    private final DigitalInput indexerBeamBreak;
    private final DigitalInput shooterBeamBreak;

    /** Detects notes in the intake, indexer, and shooter */
    public NoteSensor() {
        intakeBeamBreak = new DigitalInput(DIOConfig.INTAKE_BEAM_BREAK);
        indexerBeamBreak = new DigitalInput(DIOConfig.INDEXER_BEAM_BREAK);
        shooterBeamBreak = new DigitalInput(DIOConfig.SHOOTER_BEAM_BREAK);

        ShuffleboardHelper.addOutput("Intake Break", Constants.DRIVER_TAB, () -> noteInIntake()).withPosition(0, 1)
                .asNotDebug();
        ShuffleboardHelper.addOutput("Indexer Break", Constants.DRIVER_TAB, () -> noteInIndexer()).withPosition(0, 2)
                .asNotDebug();
        ShuffleboardHelper.addOutput("Shooter Break", Constants.DRIVER_TAB, () -> noteInShooter()).withPosition(1, 0)
                .withSize(9, 4).asNotDebug();
    }

    /** Ends as soon as a note is detected in the intake */
    public Command waitForNoteInIntake() {
        return Commands.waitUntil(() -> noteInIntake());
    }

    /** Ends as soon as a note is detected in the indexer */
    public Command waitForNoteInIndexer() {
        return Commands.waitUntil(() -> noteInIndexer());
    }

    /** Ends as soon as a note is detected in the indexer */
    public Command waitForNoteInShooter() {
        return Commands.waitUntil(() -> noteInShooter());
    }

    public Command waitForNoNoteInShooter() {
        return Commands.waitUntil(() -> !noteInShooter());
    }

    /** @returns true if there is a note in the intake */
    public boolean noteInIntake() {
        return !intakeBeamBreak.get();
    }

    /** @returns true if there is a note in the indexer */
    public boolean noteInIndexer() {
        return !indexerBeamBreak.get();
    }

    /** @returns true if there is a note in the shooter */
    public boolean noteInShooter() {
        return !shooterBeamBreak.get();
    }
}
