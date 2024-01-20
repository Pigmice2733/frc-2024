package frc.robot.state_machine_transitions.running_loops;

import com.pigmice.frc.lib.finite_state_machine.RunningLoop;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class RunMovingArm extends RunningLoop<RobotState, RobotData> {
    public RunMovingArm() {
        super(RobotState.MOVING_ARM);
    }

    @Override
    protected void run(RobotData robotData) {
        // TODO: move the arm and wrist
        throw new UnsupportedOperationException();
    }

}
