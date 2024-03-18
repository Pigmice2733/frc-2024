package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class AutoAimShooter extends SequentialCommandGroup {

    public AutoAimShooter(Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter) {
        addCommands(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_CENTER, false),
                Commands.run(() -> {
                    double distanceToSpeaker = DriverStation.getAlliance().get() == Alliance.Blue
                            ? drivetrain.getDistanceFromPoint(AutoConfig.Locations.BLUE_SPEAKER)
                            : drivetrain.getDistanceFromPoint(AutoConfig.Locations.RED_SPEAKER);

                    double wristAngle = calculateWristAngle(distanceToSpeaker);
                    wrist.setTargetRotation(wristAngle);
                    System.out.println("Distance To Speaker: " + distanceToSpeaker);
                }));

        addRequirements(wrist);
    }

    public static double calculateWristAngle(double distanceToSpeaker) {
        return 0;
    }
}
