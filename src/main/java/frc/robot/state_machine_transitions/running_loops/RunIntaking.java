package frc.robot.state_machine_transitions.running_loops;

import com.pigmice.frc.lib.finite_state_machine.RunningLoop;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class RunIntaking extends RunningLoop<RobotState, RobotData> {
    public RunIntaking() {
        super(RobotState.INTAKING);
    }

    @Override
    protected void run(RobotData robotData) {
        // TODO: keep running the intake
        throw new UnsupportedOperationException();
    }
}
