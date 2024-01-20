// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.Constants.PortConfig;
import frc.robot.Constants.ArmConfig.ArmState;

public class Arm extends PIDSubsystemBase {
    /** Moves the shooter box up, down, and around the space above the robot. */
    public Arm() {
        super(new CANSparkMax(PortConfig.ARM_ROTATION, MotorType.kBrushless), ArmConfig.P, ArmConfig.i, ArmConfig.D,
                new Constraints(ArmConfig.MAX_VELOCITY, ArmConfig.MAX_ACCELERATION), false,
                ArmConfig.MOTOR_POSITION_CONVERSION, 50, Constants.ARM_TAB, true);

        CANSparkMax encoderController = new CANSparkMax(PortConfig.ARM_ENCODER, MotorType.kBrushed);
        RelativeEncoder encoder = encoderController.getEncoder();
        addCustomEncoder(() -> encoder.getPosition());
        encoderController.close();

        addLimitSwitch(0, PortConfig.ARM_LIMIT_SWITCH, false, LimitSwitchSide.NEGATIVE);
    }

    /** Sets the rotation state of the arm */
    public Command setTargetState(ArmState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }

    /** Sets the rotation state to 'STOW' */
    public Command stow() {
        return setTargetState(ArmState.STOW);
    }

    /** Sets the target rotation, then waits until it gets to that rotation */
    public Command goToState(ArmState state) {
        return Commands.parallel(setTargetState(state), Commands.waitUntil(
                () -> Math.abs(getCurrentRotation() - state.getPosition()) < ArmConfig.POSITION_TOLERANCE));
    }
}
