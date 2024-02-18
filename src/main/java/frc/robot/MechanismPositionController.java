package frc.robot;

import java.util.ArrayList;
import java.util.Hashtable;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.pigmice.frc.lib.utils.Range;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismPositionController extends SubsystemBase {
    private final Hashtable<PIDSubsystemBase, Mechanism> mechanisms = new Hashtable<PIDSubsystemBase, Mechanism>();

    /**
     * The purpose of this class is to control the position of different
     * mechanisms, avoiding collisions between them.
     */
    public MechanismPositionController(PIDSubsystemBase... subsystems) {
        for (var subsystem : subsystems) {
            Mechanism mechanism = new Mechanism(subsystem);
            mechanisms.put(subsystem, mechanism);
        }
    }

    @Override
    public void periodic() {
        // Update all mechanisms
        for (var mechanism : mechanisms.values()) {
            mechanism.periodic();
        }
    }

    /**
     * Add a restraint between two mechanisms.
     * 
     * @param subsystemA the first subsystem
     * @param subsystemB the second subsystem
     * @param rangeA     the overlapping position range of the first subsystem
     * @param rangeB     the overlapping position range of the second subsystem
     */
    public void addRestraint(PIDSubsystemBase subsystemA,
            PIDSubsystemBase subsystemB, Range rangeA, Range rangeB) {
        Mechanism mechanismA = findMechanism(subsystemA);
        Mechanism mechanismB = findMechanism(subsystemB);
        Restraint restraint = new Restraint(mechanismA, mechanismB, rangeA, rangeB);

        mechanismA.addRestraint(restraint);
        mechanismB.addRestraint(restraint);
    }

    /**
     * Sets the goal position of a mechanism.
     * 
     * @param subsystem    the subsystem to set a goal for
     * @param goalPosition the goal position for the mechanism
     */
    public void setMechanismGoal(PIDSubsystemBase subsystem,
            double goalPosition) throws Exception {
        findMechanism(subsystem).setGoalPosition(goalPosition);
    }

    private Mechanism findMechanism(SubsystemBase subsystem) {
        Mechanism output = mechanisms.get(subsystem);
        if (output == null)
            System.out.println(
                    "Mechanism \"" + subsystem.getName() + "\" not found!");
        return output;
    }

    private class Mechanism {
        private final PIDSubsystemBase subsystem;
        private final ArrayList<Restraint> restraints = new ArrayList<Restraint>();
        private double goalPosition, clampedSetpoint;

        private Mechanism(PIDSubsystemBase subsystem) {
            this.subsystem = subsystem;
            goalPosition = clampedSetpoint = getCurrentPosition();
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
            clampedSetpoint = goalPosition;

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
        private Restraint(Mechanism mechanismA, Mechanism mechanismB,
                Range rangeA, Range rangeB) {
            this.mechanismA = mechanismA;
            this.mechanismB = mechanismB;
            this.rangeA = rangeA;
            this.rangeB = rangeB;
        }

        /**
         * Take the closest position to a mechanism's goal position that is
         * outside the overlap range.
         * 
         * @param targetMechanism the mechanism that is trying to move
         * @param goal            its goal position
         * 
         * @return the goal clamped to a "safe" value
         */
        private double evaluate(Mechanism targetMechanism, double goal) {
            if (targetMechanism == mechanismA) {
                // Check if the other mechanism is in its "dangerous" range
                if (!rangeB.contains(mechanismB.getCurrentPosition()))
                    return goal;

                // Moving in the positive direction
                if (mechanismA.getCurrentPosition() < goal) {
                    return rangeA.min();
                } else {
                    // Moving in the negative direction
                    return rangeA.max();
                }
            } else if (targetMechanism == mechanismB) {
                // Check if the other mechanism is in its "dangerous" range
                if (!rangeA.contains(mechanismA.getCurrentPosition()))
                    return goal;

                // Moving in the positive direction
                if (mechanismB.getCurrentPosition() < goal) {
                    return rangeB.min();
                } else {
                    // Moving in the negative direction
                    return rangeB.max();
                }
            } else {
                return targetMechanism.getCurrentPosition();
            }
        }
    }
}
