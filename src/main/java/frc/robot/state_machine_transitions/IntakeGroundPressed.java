package frc.robot.state_machine_transitions;

import com.pigmice.frc.lib.finite_state_machine.Transition;

import frc.robot.commands.RunRobotStateMachine.RobotData;
import frc.robot.commands.RunRobotStateMachine.RobotState;

public class IntakeGroundPressed extends Transition<RobotState, RobotData> {

    public IntakeGroundPressed() {
        super(RobotState.INTAKING);
    }

    @Override
    public boolean shouldExecute(RobotData robotData) {
        // TODO: is the intake from ground button pressed?
        throw new UnsupportedOperationException("Unimplemented method 'shouldExecute'");
    }

    @Override
    public RobotState execute(RobotData robotData) {
        // TODO: start intake flywheels
        return to;
    }
}
