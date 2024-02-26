package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunClimber extends Command {
    private final Climber climber;
    private final Supplier<Double> climbingSpeed;

    /** Runs the climbing based on an input from zero to one */
    public RunClimber(Climber climber, Supplier<Double> climbingSpeed) {
        this.climber = climber;
        this.climbingSpeed = climbingSpeed;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.outputToMotors(climbingSpeed.get());
    }
}
