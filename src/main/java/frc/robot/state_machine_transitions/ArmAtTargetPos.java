package frc.robot.state_machine_transitions;

import com.pigmice.frc.lib.finite_state_machine.Transition;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class ArmAtTargetPos extends Transition<RobotState, RobotData> {

    public ArmAtTargetPos(RobotState to) {
        super(to);
    }

    @Override
    public boolean shouldExecute(RobotData robotData) {
        // TODO: are both the arm and wrist PID controllers in tolerance?
        throw new UnsupportedOperationException("Unimplemented method 'shouldExecute'");
    }

    @Override
    public RobotState execute(RobotData robotData) {
        return to;
    }

}
