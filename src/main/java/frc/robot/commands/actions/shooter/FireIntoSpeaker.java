package frc.robot.commands.actions.shooter;

import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class FireIntoSpeaker extends FireShooter {

    public FireIntoSpeaker(Arm arm, Wrist wrist, Shooter shooter, Indexer indexer) {
        super(arm, wrist, shooter, indexer, ArmState.SPEAKER, WristState.SPEAKER);
    }

}
