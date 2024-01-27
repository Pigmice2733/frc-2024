package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Stow extends SequentialCommandGroup {
    /** Stows all non-climber subsystems. */
    public Stow(Intake intake, Arm arm, Wrist wrist) {
        addCommands(
                Commands.parallel(
                        arm.goToState(ArmState.STOW),
                        wrist.goToState(WristState.STOW)),
                intake.goToState(IntakeState.STOW));
        addRequirements(intake, arm, wrist);
    }
}
