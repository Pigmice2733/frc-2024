// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

// import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
// import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class Climb extends SequentialCommandGroup {
    /**
     * Drives to the chain on the left of the stage and climbs. Needs implementation
     * for other chains.
     */
    public Climb(/* SwerveDrivetrain drivetrain, */ Pathfinder pathfinder, Arm arm, Wrist wrist, Climber climber,
            Intake intake, NoteSensor noteSensor) {
        addCommands(
                // Pathfind to the chain and stop in front of it
                // new PathfindToPointSwerve(drivetrain, pathfinder, Locations.CLIMBING_LEFT),
                // Stow all other subsystems
                new MoveKobraToPosition(arm, wrist, intake, KobraState.STOW, noteSensor, true)
        // Raise climber TODO
        // climber.goToState(ClimberState.UP),
        // Drive to under the chain
        // TODO: fix once the fixed drivetrain is merged in
        // Commands.runOnce(
        // () -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(0,
        // AutoConfig.CLIMB_DRIVE_SPEED, 0))),
        // Commands.waitSeconds(AutoConfig.CLIMB_DRIVE_TIME),
        // Commands.runOnce(
        // () -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0))),
        // Pull robot up using climber TODO
        // climber.goToState(ClimberState.DOWN)
        );
        addRequirements(/* TODO drivetrain, */ arm, wrist, climber, intake);
    }
}
