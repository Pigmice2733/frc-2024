package frc.robot.commands.autonomous.subcommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreFromStartAuto extends SequentialCommandGroup {
    public ScoreFromStartAuto(Intake intake, Indexer indexer, Arm arm, Wrist wrist, Shooter shooter,
            boolean waitForStow, KobraState shootingState, NoteSensor noteSensor) {
        addCommands(
                Commands.either(
                        new MoveKobraToPosition(arm, wrist, intake, shootingState, noteSensor, false),
                        Commands.none(),
                        () -> MoveKobraToPosition.currentKobraState != shootingState),
                new FireShooter(indexer, shooter), Commands.waitSeconds(0.15),
                Commands.parallel(indexer.stopIndexer(), shooter.stopFlywheels()));

        if (waitForStow)
            addCommands(new MoveKobraToPosition(arm, wrist, intake, KobraState.STOW, noteSensor, false));
        else
            addCommands(Commands.parallel(arm.setTargetState(ArmState.STOW), wrist.setTargetState(WristState.STOW)));
        addRequirements(intake, indexer, arm, wrist, shooter);
    }
}
