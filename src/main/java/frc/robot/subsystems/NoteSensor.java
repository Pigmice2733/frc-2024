package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOConfig;

public class NoteSensor extends SubsystemBase {
    private final DigitalInput intakeBeamBreak;
    private final DigitalInput indexerBeamBreak;

    private NoteState noteState = NoteState.NONE;

    /**
     * Detects when the shooter is carrying a note. If it is, the shooter prepares
     * to shoot, and the intake stops running.
     */
    public NoteSensor() {
        intakeBeamBreak = new DigitalInput(DIOConfig.INTAKE_BEAM_BREAK);
        indexerBeamBreak = new DigitalInput(DIOConfig.INDEXER_BEAM_BREAK);
    }

    @Override
    public void periodic() {
        // Check for a note in the intake and indexer
        if (intakeBeamBreak.get())
            noteState = NoteState.INTAKE;
        else if (indexerBeamBreak.get())
            noteState = NoteState.INDEXER;
        else
            noteState = NoteState.NONE;
    }

    public NoteState getNoteState() {
        return noteState;
    }

    /** Ends as soon as a note is detected in the intake */
    public Command waitForNoteInIntake() {
        return Commands.waitUntil(() -> noteState == NoteState.INTAKE);
    }

    /** Ends as soon as a note is detected in the indexer */
    public Command waitForNoteInIndexer() {
        return Commands.waitUntil(() -> noteState == NoteState.INDEXER);
    }

    public Command waitForNoNote() {
        return Commands.waitUntil(() -> noteState == NoteState.NONE);
    }

    public enum NoteState {
        NONE,
        INTAKE,
        INDEXER
    }
}
