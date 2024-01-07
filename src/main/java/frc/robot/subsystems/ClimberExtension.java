// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ClimberConfig;

public class ClimberExtension extends PIDSubsystemBase {
    public ClimberExtension() {
        super(new CANSparkMax(CANConfig.CLIMBER_EXTENSION, MotorType.kBrushless), ClimberConfig.p, ClimberConfig.i,
                ClimberConfig.d, new Constraints(ClimberConfig.maxVelocity, ClimberConfig.maxAcceleration), false,
                ClimberConfig.motorPositionConversion, Constants.CLIMBER_TAB, true);
    }

    @Override
    public void periodic() {
    }
}
