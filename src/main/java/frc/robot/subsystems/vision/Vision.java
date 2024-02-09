package frc.robot.subsystems.vision;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConfig;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.Results;

public class Vision extends SubsystemBase {
    private static String camName = VisionConfig.CAM_NAME;

    private Results targetingResults;
    private LimelightTarget_Fiducial bestTarget;

    private boolean hasTarget;

    private Field2d fieldWidget;

    /** Finds and uses AprilTags and other vision targets. */
    public Vision() {
        fieldWidget = new Field2d();

        Constants.VISION_TAB.add(fieldWidget).withPosition(2, 0).withSize(7, 4);

        ShuffleboardHelper.addOutput("Target X", Constants.VISION_TAB,
                () -> hasTarget() ? bestTarget.tx : 0).withPosition(0, 0);
        ShuffleboardHelper.addOutput("Target Y", Constants.VISION_TAB,
                () -> hasTarget() ? bestTarget.ty : 0).withPosition(1, 0);

        ShuffleboardHelper.addOutput("Bot Pose X", Constants.VISION_TAB,
                () -> hasTarget() ? getEstimatedRobotPose().getX() : 0).withPosition(0, 1);
        ShuffleboardHelper.addOutput("Bot Pose Y", Constants.VISION_TAB,
                () -> hasTarget() ? getEstimatedRobotPose().getY() : 0).withPosition(1, 1);

        ShuffleboardHelper.addOutput("Pose to tag X", Constants.VISION_TAB,
                () -> hasTarget() ? getTranslationToBestTarget().getX() : 0).withPosition(0, 2);
        ShuffleboardHelper.addOutput("Pose to tag Y", Constants.VISION_TAB,
                () -> hasTarget() ? getTranslationToBestTarget().getY() : 0).withPosition(1, 2);
    }

    @Override
    public void periodic() {
        targetingResults = LimelightHelpers.getLatestResults(camName).targetingResults;

        var allTargets = targetingResults.targets_Fiducials;

        if (allTargets.length == 0) {
            bestTarget = null;
            hasTarget = false;
            return;
        }

        hasTarget = true;

        bestTarget = allTargets[0];

        // Update the field widget
        var estimatedPose = getEstimatedRobotPose();

        if (estimatedPose != null)
            fieldWidget.setRobotPose(estimatedPose);
    }

    /** @return true when there is a current target */
    public boolean hasTarget() {
        return hasTarget;
    }

    /** @return the best target's id or -1 if no target is seen */
    public int getBestTargetID() {
        return (int) (!hasTarget() ? -1 : bestTarget.fiducialID);
    }

    /** @return the robot's estimated 2d pose or null if no target is seen */
    public Pose2d getEstimatedRobotPose() {
        if (!hasTarget)
            return null;

        var estimatedPose = targetingResults.getBotPose2d_wpiBlue();

        // Rotate 180
        // estimatedPose = new Pose2d(estimatedPose.getX(), estimatedPose.getY(),
        // Rotation2d.fromDegrees(estimatedPose.getRotation().getDegrees() - 180));

        return estimatedPose;
    }

    /** @returns the estimated 2d translation to the best target */
    public Pose2d getTranslationToBestTarget() {
        return !hasTarget() ? null : bestTarget.getRobotPose_TargetSpace2D();
    }

    /** @returns true if the ring is visible */
    public boolean ringVisible() {
        // TODO: implementation
        return false;
    }

    /** @returns a command that ends when a ring is visible */
    public Command waitForRing() {
        return Commands.waitUntil(() -> ringVisible());
    }
}
