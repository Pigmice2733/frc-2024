package frc.robot.state_machine_transitions;

import com.pigmice.frc.lib.finite_state_machine.Transition;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class ScoreAmpPressed extends Transition<RobotState, RobotData> {

    public ScoreAmpPressed() {
        super(RobotState.SCORING);
    }

    @Override
    public boolean shouldExecute(RobotData robotData) {
        // TODO: is the score amp button pressed?
        throw new UnsupportedOperationException("Unimplemented method 'shouldExecute'");
    }

    @Override
    public RobotState execute(RobotData robotData) {
        // TODO: set wrist, arm, and intake target positions
        return to;
    }
}
