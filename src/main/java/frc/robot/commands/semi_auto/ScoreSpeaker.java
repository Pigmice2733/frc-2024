// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.AutoConfig.Locations;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreSpeaker extends SequentialCommandGroup {
    /** Drives to the speaker then scores */
    public ScoreSpeaker(SwerveDrivetrain drivetrain, Pathfinder pathfinder, Arm arm, Wrist wrist, Shooter shooter,
            Indexer indexer, Intake intake, NoteSensor noteSensor) {

        addCommands(
                // Pathfind to a point right in front of the amp
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.AMP_SCORING),
                // Start spinning the flywheels
                shooter.spinFlywheelsForward(),
                // Wait until the wheels are spinning and the subsystems are in place
                Commands.parallel(
                        Commands.waitSeconds(AutoConfig.SHOOTER_SPINUP_TIME),
                        // Move the intake out of the way, then the arm and wrist to position
                        Commands.sequence(
                                intake.goToState(IntakeState.UP),
                                Commands.parallel(
                                        wrist.goToState(WristState.SPEAKER),
                                        arm.goToState(ArmState.SPEAKER)))),
                // Push the note forward to shoot
                indexer.indexForward(),
                // Wait until the note is out
                noteSensor.waitForNoNote(),
                // Stop spinning the indexer and shooter wheels
                Commands.parallel(indexer.stopIndexer(), shooter.stopFlywheels()));

        addRequirements(drivetrain, arm, wrist, shooter, indexer, intake, noteSensor);
    }
}
