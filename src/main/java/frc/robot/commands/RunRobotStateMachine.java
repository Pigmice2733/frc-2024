package frc.robot.commands;

import com.pigmice.frc.lib.finite_state_machine.FiniteStateMachine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.vision.Vision;

public class RunRobotStateMachine extends Command {

    private final FiniteStateMachine<RobotState, RobotData> stateMachine;
    private final RobotData robotData;

    public RunRobotStateMachine(Arm arm, Climber climber, Indexer indexer, Intake intake, NoteSensor noteSensor,
            Shooter shooter, Wrist wrist, Vision vision, XboxController driverController,
            XboxController operatorController) {

        stateMachine = new FiniteStateMachine<RobotState, RobotData>(RobotState.IDLE);

        robotData = new RobotData(arm, climber, indexer, intake, noteSensor, shooter, wrist, vision,
                driverController, operatorController);

        addRequirements();
    }

    @Override
    public void execute() {
        if (!stateMachine.execute(robotData)) {
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
        public final Arm arm;
        public final Climber climber;
        public final Indexer indexer;
        public final Intake intake;
        public final NoteSensor noteSensor;
        public final Shooter shooter;
        public final Wrist wrist;
        public final Vision vision;
        public final XboxController driverController;
        public final XboxController operatorController;

        public RobotData(Arm arm, Climber climber, Indexer indexer, Intake intake, NoteSensor noteSensor,
                Shooter shooter, Wrist wrist, Vision vision, XboxController driverController,
                XboxController operatorController) {

            this.arm = arm;
            this.climber = climber;
            this.indexer = indexer;
            this.intake = intake;
            this.noteSensor = noteSensor;
            this.shooter = shooter;
            this.wrist = wrist;
            this.vision = vision;
            this.driverController = driverController;
            this.operatorController = operatorController;
        }
    }
}
