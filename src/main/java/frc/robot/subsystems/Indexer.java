// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IndexerConfig;

public class Indexer extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(CANConfig.LEFT_INDEXER, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(CANConfig.LEFT_INDEXER, MotorType.kBrushless);

    /**
     * Shoots notes out of one end into the speaker and dumps them out of the other
     * end into the amp.
     * Intakes from the source into the speaker-shooting end.
     */
    public Indexer() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(false);
        rightMotor.setSmartCurrentLimit(40);

        rightMotor.follow(leftMotor);

        ShuffleboardHelper.addOutput("Motor Output", Constants.INDEXER_TAB, () -> leftMotor.get());

        ShuffleboardHelper.addInput("Set Speed", Constants.SHOOTER_TAB,
                (output) -> outputToMotor((double) output), 0);
    }

    public void outputToMotor(double output) {
        leftMotor.set(output);
    }

    public Command indexForward() {
        return Commands.runOnce(() -> outputToMotor(IndexerConfig.DEFAULT_SPEED));
    }

    /** Intake into the shooter box, as long as no note is being carried. */
    public Command indexBackward() {
        return Commands.runOnce(() -> outputToMotor(IndexerConfig.BACKWARD_SPEED));
    }

    public Command stopIndexer() {
        return Commands.runOnce(() -> outputToMotor(0));
    }
}