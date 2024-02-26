// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.WristConfig;
import frc.robot.Constants.WristConfig.WristState;

public class Wrist extends PIDSubsystemBase {
    private static Wrist instance;

    public static double getRotation() {
        if (instance != null)
            return instance.getCurrentRotation();
        System.out.println("wrist is null");
        return 0;
    }

    public Wrist() {
        super(new CANSparkMax(CANConfig.WRIST_ROTATION, MotorType.kBrushless), WristConfig.P, WristConfig.I,
                WristConfig.D, new Constraints(WristConfig.MAX_VELOCITY, WristConfig.MAX_ACCELERATION), false,
                WristConfig.MOTOR_POSITION_CONVERSION, 40, Constants.WRIST_TAB, false, false);
        instance = this;

        ShuffleboardHelper.addOutput("Extension Distance", Constants.WRIST_TAB,
                () -> calculateExtensionDistance(getCurrentRotation(), Arm.getRotation()));

        ShuffleboardHelper.addOutput("Over E Limit", Constants.WRIST_TAB, () -> {
            return calculateExtensionDistance(getCurrentRotation(), Arm.getRotation()) > 12.0;
        });

        ShuffleboardHelper.addOutput("Min Wrist Angle", Constants.WRIST_TAB,
                () -> calculateMinWristAngle(getCurrentRotation()));

        // addSoftwareStop(0, 360);

        // TODO: test a dynamic software stop
        addSoftwareStop(
                () -> calculateMinWristAngle(Arm.getRotation()),
                () -> calculateMaxWristAngle(Arm.getRotation()));

        setMaxAllowedOutput(0.8);
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
                () -> atState(state)));
    }

    public boolean atState(WristState state) {
        return Math.abs(getCurrentRotation() - state.getPosition()) < WristConfig.POSITION_TOLERANCE;
    }

    /** Waits until the wrist is at a state, without commanding it to go there */
    public Command waitForState(WristState state) {
        return Commands.waitUntil(() -> atState(state));
    }

    @Override
    public double getCurrentRotation() {
        // Makes the wrists current rotation relative to the ground
        return getMotor().getEncoder().getPosition() - Arm.getRotation();
    }

    /** @return the extension distance of the arm outside the drivetrain frame */
    public static double calculateExtensionDistance(double armAngle, double wristAngle) {
        // x pos of wrist pivot relative to shoulder pivot
        double wristPivotX = -Math.cos(Units.degreesToRadians(armAngle)) * ArmConfig.LENGTH_INCHES;

        // The length of the wrist in the x dimension
        double wristLengthX = Math.cos(Units.degreesToRadians(wristAngle)) * WristConfig.LENGTH_INCHES;

        // Total extension distance relative to the shoulder pivot
        double relativeExtensionDistance = wristPivotX + wristLengthX;

        // Return the extension distance outside of the frame
        return relativeExtensionDistance - ArmConfig.PIVOT_TO_FRAME_INCHES;
    }

    public static double calculateMinWristAngle(double armAngle) {
        /*
         * // x pos of wrist pivot relative to frame
         * double wristPivotX = -Math.cos(Units.degreesToRadians(armAngle)) *
         * ArmConfig.LENGTH_INCHES
         * - ArmConfig.PIVOT_TO_FRAME_INCHES;
         * 
         * // The max wrist length in x dimension
         * double maxWristLengthX = 12.0 - wristPivotX;
         * 
         * if (maxWristLengthX > WristConfig.LENGTH_INCHES)
         * return 0;
         * 
         * // The min angle the wrist can be at without going over max extension limit
         * return Math.acos(maxWristLengthX / WristConfig.LENGTH_INCHES);
         */

        if (armAngle >= 80) {
            return 90;
        }
        return -armAngle;
    }

    public static double calculateMaxWristAngle(double armAngle) {
        if (armAngle < 40)
            return 90;

        return 270;
    }
}
