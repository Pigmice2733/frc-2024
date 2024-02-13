// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ClimberConfig;

public class Climber extends SubsystemBase {
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    /** Moves the chain hooks into position to raise the robot. */
    public Climber() {
        leftMotor = new CANSparkMax(CANConfig.LEFT_CLIMB, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(true);

        rightMotor = new CANSparkMax(CANConfig.RIGHT_CLIMB, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        rightMotor.follow(leftMotor, false);
    }

    @Override
    public void periodic() {
    }

    private void outputToMotors(double percent) {
        leftMotor.set(percent);
    }

    public void extendClimber() {
        outputToMotors(ClimberConfig.extensionSpeed);
    }

    /** For stowing the climber at the start of a match */
    public void retractClimberSlow() {
        outputToMotors(-ClimberConfig.extensionSpeed);
    }

    /** For actually climbing */
    public void retractClimberFast() {
        outputToMotors(ClimberConfig.climbingSpeed);
    }
}
