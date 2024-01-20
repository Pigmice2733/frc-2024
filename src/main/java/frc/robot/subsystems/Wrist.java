// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.PortConfig;
import frc.robot.Constants.WristConfig;
import frc.robot.Constants.WristConfig.WristState;

public class Wrist extends PIDSubsystemBase {
    public Wrist() {
        super(new CANSparkMax(PortConfig.WRIST_ROTATION, MotorType.kBrushless), WristConfig.P, WristConfig.I,
                WristConfig.D, new Constraints(WristConfig.MAX_VELOCITY, WristConfig.MAX_ACCELERATION), false,
                WristConfig.MOTOR_POSITION_CONVERSION, 50, Constants.WRIST_TAB, true);

        CANSparkMax encoderController = new CANSparkMax(PortConfig.WRIST_ENCODER, MotorType.kBrushed);
        RelativeEncoder encoder = encoderController.getEncoder();
        addCustomEncoder(() -> encoder.getPosition());
        encoderController.close();

        addLimitSwitch(0, PortConfig.WRIST_LIMIT_SWITCH, false, LimitSwitchSide.NEGATIVE);
    }

    /** Sets the height state of the climber */
    public Command setTargetState(WristState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }

    /** Sets the rotation state to 'STOW' */
    public Command stow() {
        return setTargetState(WristState.STOW);
    }

    /** Sets the target rotation, then waits until it gets to that rotation */
    public Command goToState(WristState state) {
        return Commands.parallel(setTargetState(state), Commands.waitUntil(
                () -> Math.abs(getCurrentRotation() - state.getPosition()) < WristConfig.POSITION_TOLERANCE));
    }
}
