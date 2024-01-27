package frc.robot.state_machine_transitions.running_loops;

import com.pigmice.frc.lib.finite_state_machine.RunningLoop;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class RunScoring extends RunningLoop<RobotState, RobotData> {
    public RunScoring() {
        super(RobotState.SCORING);
    }

    @Override
    protected void run(RobotData robotData) {
        // TODO: run and feed the flywheels
        throw new UnsupportedOperationException();
    }
}
