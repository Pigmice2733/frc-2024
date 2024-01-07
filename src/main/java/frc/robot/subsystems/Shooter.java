// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkMax flywheelsMotor = new CANSparkMax(CANConfig.SHOOTER_MOTOR, MotorType.kBrushless);
    private final CANSparkMax feederMotor = new CANSparkMax(CANConfig.FEEDER_MOTOR, MotorType.kBrushless);

    public Shooter() {
        flywheelsMotor.restoreFactoryDefaults();
        flywheelsMotor.setInverted(false);

        ShuffleboardHelper.addOutput("Motor Output", Constants.SHOOTER_TAB, () -> flywheelsMotor.get());
    }

    private void outputToFlywheels(double output) {
        flywheelsMotor.set(output);
    }

    public Command spinFlywheelsForward() {
        return Commands.runOnce(() -> outputToFlywheels(ShooterConfig.DEFAULT_FLYWHEEL_SPEED));
    }

    public Command spinFlywheelsBackward() {
        return Commands.runOnce(() -> outputToFlywheels(-ShooterConfig.DEFAULT_FLYWHEEL_SPEED));
    }

    public Command stopFlywheels() {
        return Commands.runOnce(() -> outputToFlywheels(0));
    }

    private void outputToFeeder(double output) {
        feederMotor.set(output);
    }

    public Command spinFeederForward() {
        return Commands.runOnce(() -> outputToFeeder(ShooterConfig.DEFAULT_FLYWHEEL_SPEED));
    }

    public Command spinFeederBackward() {
        return Commands.runOnce(() -> outputToFeeder(-ShooterConfig.DEFAULT_FLYWHEEL_SPEED));
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> outputToFeeder(0));
    }
}
