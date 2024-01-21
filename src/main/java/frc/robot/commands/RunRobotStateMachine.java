package frc.robot.commands;

import com.pigmice.frc.lib.finite_state_machine.FiniteStateMachine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.Controls.ControlsState;

public class RunRobotStateMachine extends CommandBase {
    private final Supplier<ControlsState> controlsStateSupplier;

    private final FiniteStateMachine<RobotState, RobotData> stateMachine;

    public RunRobotStateMachine(Supplier<ControlsState> controlsStateSupplier) {

        this.controlsStateSupplier = controlsStateSupplier;

        stateMachine = new FiniteStateMachine<RobotState, RobotData>(RobotState.IDLE);

        addRequirements();
    }

    @Override
    public void execute() {
        if (!stateMachine.execute(new RobotData())) {
            System.out.println("Robot state machine encountered an error.");
        }
    }

    public enum RobotState {
        IDLE,
        MOVING_ARM,
        SCORING,
        INTAKING
    }

    public class RobotData {
        public ControlsState controls;

        public RobotData() {
            controls = controlsStateSupplier.get();
        }
    }
}
