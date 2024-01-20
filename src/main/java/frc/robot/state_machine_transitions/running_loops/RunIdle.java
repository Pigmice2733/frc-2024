package frc.robot.state_machine_transitions.running_loops;

import com.pigmice.frc.lib.finite_state_machine.RunningLoop;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class RunIdle extends RunningLoop<RobotState, RobotData> {
    public RunIdle() {
        super(RobotState.IDLE);
    }

    @Override
    protected void run(RobotData robotData) {
    }

}
