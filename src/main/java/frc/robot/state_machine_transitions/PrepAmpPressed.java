package frc.robot.state_machine_transitions;

import com.pigmice.frc.lib.finite_state_machine.Transition;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class PrepAmpPressed extends Transition<RobotState, RobotData> {

    public PrepAmpPressed() {
        super(RobotState.MOVING_ARM);
    }

    @Override
    public boolean shouldExecute(RobotData robotData) {
        // TODO: is the prepare for amp scoring button pressed?
        throw new UnsupportedOperationException("Unimplemented method 'shouldExecute'");
    }

    @Override
    public RobotState execute(RobotData robotData) {
        // TODO: set wrist, arm, and intake target positions
        return to;
    }

}
