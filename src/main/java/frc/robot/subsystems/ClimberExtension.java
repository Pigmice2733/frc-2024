// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ClimberConfig;
import frc.robot.Constants.ClimberConfig.ClimberState;

public class ClimberExtension extends PIDSubsystemBase {
    public ClimberExtension() {
        super(new CANSparkMax(CANConfig.CLIMBER_EXTENSION, MotorType.kBrushless), ClimberConfig.P, ClimberConfig.I,
                ClimberConfig.D, new Constraints(ClimberConfig.MAX_VELOCITY, ClimberConfig.MAX_ACCELERATION), false,
                ClimberConfig.MOTOR_POSITION_CONVERSION, 50, Constants.CLIMBER_TAB, true);
    }

    @Override
    public void periodic() {
    }

    /** Sets the height state of the climber */
    public Command setTargetState(ClimberState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }
}
