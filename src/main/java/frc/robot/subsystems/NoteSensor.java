package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteSensor extends SubsystemBase {
    static boolean noteState;

    /**
     * Detects when the shooter is carrying a note. If it is, the shooter prepares
     * to shoot, and the intake stops running.
     */
    public NoteSensor() {
        // initiate the sensor object
        noteState = false;
    }

    @Override
    public void periodic() {
        // sensor logic, set noteState
    }

    public static boolean getNoteState() {
        return noteState;
    }
}
