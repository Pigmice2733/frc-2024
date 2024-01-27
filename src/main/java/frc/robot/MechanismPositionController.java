package frc.robot;

import java.util.ArrayList;
import java.util.Hashtable;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.pigmice.frc.lib.utils.Range;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismPositionController {
    private final Hashtable<PIDSubsystemBase, Mechanism> mechanisms = new Hashtable<PIDSubsystemBase, Mechanism>();

    /**
     * The purpose of this class is to control the position of different mechanisms,
     * avoiding collisions between them
     */
    public MechanismPositionController(PIDSubsystemBase... subsystems) {
        for (var subsystem : subsystems) {
            Mechanism mechanism = new Mechanism(subsystem);
            mechanisms.put(subsystem, mechanism);
        }
    }

    // TODO: call somewhere
    public void periodic() {
        // Update all mechanisms
        for (var mechanism : mechanisms.values()) {
            mechanism.periodic();
        }
    }

    /**
     * Add a restraint between two mechanism
     * 
     * @param subsystemA the first subsystem
     * @param subsystemB the second subsystem
     * @param rangeA     the overlapping position range of the first subsystem
     * @param rangeB     the overlapping position range of the first subsystem
     */
    public void addRestraint(PIDSubsystemBase subsystemA, PIDSubsystemBase subsystemB, Range rangeA, Range rangeB) {
        Mechanism mechanismA = findMechanism(subsystemA);
        Mechanism mechanismB = findMechanism(subsystemB);

        Restraint restraint = new Restraint(mechanismA, mechanismB, rangeA, rangeB);

        mechanismA.addRestraint(restraint);
        mechanismB.addRestraint(restraint);
    }

    /**
     * Sets the goal position of a mechanism
     * 
     * @param subsystem    the subsystem to set a goal for
     * @param goalPosition the goal position for the mechanism
     */
    public void setMechanismGoal(PIDSubsystemBase subsystem, double goalPosition) throws Exception {
        findMechanism(subsystem).setGoalPosition(goalPosition);
    }

    private Mechanism findMechanism(SubsystemBase subsystem) {
        if (mechanisms.containsKey(subsystem)) {
            return mechanisms.get(subsystem);
        } else
            System.out.println("Mechanism \"" + subsystem.getName() + "\" not found!");
        return null;
    }

    private class Mechanism {
        private final PIDSubsystemBase subsystem;
        private final ArrayList<Restraint> restraints = new ArrayList<Restraint>();

        private double goalPosition;

        private Mechanism(PIDSubsystemBase subsystem) {
            this.subsystem = subsystem;
            goalPosition = getCurrentPosition();
        }

        private double getCurrentPosition() {
            return subsystem.getCurrentRotation();
        }

        private void setGoalPosition(double goalPosition) {
            this.goalPosition = goalPosition;
        }

        private void addRestraint(Restraint restraint) {
            restraints.add(restraint);
        }

        private void periodic() {
            double clampedSetpoint = goalPosition;

            // Apply all restraints
            for (Restraint restraint : restraints) {
                clampedSetpoint = restraint.evaluate(this, clampedSetpoint);
            }

            subsystem.setTargetRotation(clampedSetpoint);
        }
    }

    private class Restraint {
        private final Mechanism mechanismA;
        private final Mechanism mechanismB;
        private final Range rangeA;
        private final Range rangeB;

        /**
         * Represents a position range where two mechanism cannot overlap
         * 
         * @param mechanismA the first mechanism
         * @param mechanismB the second mechanism
         * @param rangeA     the first mechanism's range
         * @param rangeB     the second mechanism's range
         */
        private Restraint(Mechanism mechanismA, Mechanism mechanismB, Range rangeA, Range rangeB) {
            this.mechanismA = mechanismA;
            this.mechanismB = mechanismB;
            this.rangeA = rangeA;
            this.rangeB = rangeB;
        }

        /**
         * Take the closest position to a mechanism's goal position that is outside the
         * overlap range
         * 
         * @param targetMechanism the mechanism that is trying to move
         * @param goal            it's goal position
         * 
         * @return the goal clamped to a "safe" value
         */
        private double evaluate(Mechanism targetMechanism, double goal) {
            boolean isA = targetMechanism == mechanismA;
            Mechanism otherMechanism = isA ? mechanismB : mechanismA;
            Range targetRange = isA ? rangeA : rangeB;
            Range otherRange = isA ? rangeB : rangeA;

            // Check if the other mechanism is in it's "dangerous" range
            if (otherRange.contains(otherMechanism.getCurrentPosition()))
                return goal;

            // Return the goal, clamped to be outside of the "dangerous" range

            // Moving in the positive direction
            if (targetMechanism.getCurrentPosition() < goal)
                return Math.min(goal, targetRange.min());

            // Moving in the negative direction
            else
                return Math.max(goal, targetRange.max());
        }
    }
}
