package frc.robot.commands;

import com.pigmice.frc.lib.finite_state_machine.FiniteStateMachine;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRobotStateMachine extends CommandBase {
    private final FiniteStateMachine<RobotState, RobotData> stateMachine;

    public RunRobotStateMachine() {
        stateMachine = new FiniteStateMachine<RobotState, RobotData>(RobotState.IDLE);
        addRequirements();
    }

    @Override
    public void execute() {
        if (!stateMachine.execute(new RobotData())) {
            System.out.println("Turret state machine encountered an error.");
        }
    }

    public enum RobotState {
        IDLE,
        MOVING_ARM,
        SCORING,
        INTAKING
    }

    public class RobotData {

    }
}
