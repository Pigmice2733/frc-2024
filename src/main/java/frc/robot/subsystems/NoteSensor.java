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

    private NoteState noteState = NoteState.NONE;

    /**
     * Detects when the shooter is carrying a note. If it is, the shooter prepares
     * to shoot, and the intake stops running.
     */
    public NoteSensor() {
        intakeBeamBreak = new DigitalInput(DIOConfig.INTAKE_BEAM_BREAK);
        indexerBeamBreak = new DigitalInput(DIOConfig.INDEXER_BEAM_BREAK);
        shooterBeamBreak = new DigitalInput(DIOConfig.SHOOTER_BEAM_BREAK);

        ShuffleboardHelper.addOutput("Intake Break", Constants.DRIVER_TAB, () -> noteInIntake());
        ShuffleboardHelper.addOutput("Indexer Break", Constants.DRIVER_TAB, () -> noteInIndexer());
        ShuffleboardHelper.addOutput("Shooter Break", Constants.DRIVER_TAB, () -> noteInShooter());
    }

    @Override
    public void periodic() {
        // Check for a note
        if (noteInIntake())
            noteState = NoteState.INTAKE;
        else if (noteInIndexer())
            noteState = NoteState.INDEXER;
        else if (noteInShooter())
            noteState = NoteState.SHOOTER;
        else
            noteState = NoteState.NONE;
    }

    /** @return the current note state */
    public NoteState getNoteState() {
        return noteState;
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

    /** Ends as soon as any beam break sensor detects a note */
    public Command waitForNoNote() {
        return Commands.waitUntil(() -> noteState == NoteState.NONE);
    }

    public boolean noteInIntake() {
        return !intakeBeamBreak.get();
    }

    public boolean noteInIndexer() {
        return !indexerBeamBreak.get();
    }

    public boolean noteInShooter() {
        return !shooterBeamBreak.get();
    }

    public enum NoteState {
        NONE,
        INTAKE,
        INDEXER,
        SHOOTER
    }
}
