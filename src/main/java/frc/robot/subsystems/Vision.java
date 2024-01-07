package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConfig;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.Results;

public class Vision extends SubsystemBase {
    private static String camName = VisionConfig.CAM_NAME;

    private Results targetingResults;
    private LimelightTarget_Fiducial bestTarget;
    private boolean hasTarget;

    public Vision() {
        ShuffleboardHelper.addOutput("Target X", Constants.VISION_TAB,
                () -> bestTarget == null ? 0 : bestTarget.tx);
        ShuffleboardHelper.addOutput("Target Y", Constants.VISION_TAB,
                () -> bestTarget == null ? 0 : bestTarget.ty);

        ShuffleboardHelper.addOutput("Bot Pose X", Constants.VISION_TAB,
                () -> targetingResults == null ? 0 : getEstimatedRobotPose().getX());
        ShuffleboardHelper.addOutput("Bot Pose Y", Constants.VISION_TAB,
                () -> targetingResults == null ? 0 : getEstimatedRobotPose().getY());

        ShuffleboardHelper.addOutput("Pose to tag X", Constants.VISION_TAB,
                () -> targetingResults == null ? 0 : getTranslationToBestTarget().getX());
        ShuffleboardHelper.addOutput("Pose to tag Y", Constants.VISION_TAB,
                () -> targetingResults == null ? 0 : getTranslationToBestTarget().getY());
    }

    @Override
    public void periodic() {
        LimelightResults results = LimelightHelpers.getLatestResults(camName);
        hasTarget = results != null;

        if (hasTarget) {
            bestTarget = null;
            return;
        }

        targetingResults = results.targetingResults;

        var allTargets = results.targetingResults.targets_Fiducials;
        bestTarget = allTargets[0];
    }

    /** Returns the best target's id or -1 if no target is seen */
    public int getBestTargetID() {
        return (int) (!hasTarget ? -1 : bestTarget.fiducialID);
    }

    /** Returns the robot's estimated 2d pose or null if no target is seen */
    public Pose2d getEstimatedRobotPose() {
        return !hasTarget ? null : targetingResults.getBotPose2d();
    }

    /** Returns the estimated 2d translation to the best target */
    public Pose2d getTranslationToBestTarget() {
        return !hasTarget ? null : bestTarget.getRobotPose_TargetSpace2D();
    }
}
