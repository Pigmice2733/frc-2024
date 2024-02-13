package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class StowKobra extends SequentialCommandGroup {
    public StowKobra(Arm arm, Intake intake, Wrist wrist) {
        addCommands(intake.goToState(IntakeState.STOW), wrist.goToState(WristState.STOW), arm.goToState(ArmState.STOW));

        addRequirements(arm, wrist, intake);
    }
}
