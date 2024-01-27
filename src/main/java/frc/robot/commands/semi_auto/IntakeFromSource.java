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
import frc.robot.Constants.AutoConfig.SemiAutoLocations;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class IntakeFromSource extends SequentialCommandGroup {
    /** Drives to the human player and obtains a ring. */
    public IntakeFromSource(SwerveDrivetrain drivetrain, Pathfinder pathfinder, Intake intake, Arm arm,
            Wrist wrist, NoteSensor noteSensor, Shooter shooter) {
        addCommands(
                // Pathfind to a point right in front of the human player
                new PathfindToPointSwerve(drivetrain, pathfinder,
                        SemiAutoLocations.HUMAN_PLAYER_PICKUP),
                // Start spinning the flywheels
                shooter.spinFlywheelsBackward(),
                // Wait until the wheels are spinning and the subsystems are in place
                Commands.parallel(
                        Commands.waitSeconds(AutoConfig.SHOOTER_SPINUP_TIME),
                        // Move the intake out of the way, then the arm and wrist to position
                        Commands.sequence(
                                intake.goToState(IntakeState.UP),
                                Commands.parallel(
                                        wrist.goToState(WristState.SOURCE),
                                        arm.goToState(ArmState.SOURCE)))),
                // Wait until the note is in
                noteSensor.waitForNoteInIndexer(),
                // Stop spinning the flywheels
                shooter.stopFlywheels());

        addRequirements(drivetrain, intake, arm, wrist);
    }
}
