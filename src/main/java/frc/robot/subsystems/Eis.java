package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.AprilTags;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Eis extends SubsystemBase {
    private PhotonCamera mainCamera;
    private PhotonPipelineResult recentResult = new PhotonPipelineResult();

    public Eis(String CameraName) {
        mainCamera = new PhotonCamera(CameraName);
    }

    public PhotonTrackedTarget mainTarget() {
        if (recentResult == null || !recentResult.hasTargets())
            return null;
        return recentResult.getBestTarget();
    }

    public double getTargetYaw() {
        PhotonTrackedTarget target = mainTarget();
        if (target == null) {
            return 0.0;
        }
        return target.getYaw();
    }

    public boolean noYawErr() {
        PhotonTrackedTarget target = mainTarget();
        if (target == null) {
            return false;
        }
        double yaw = target.getYaw();
        return Math.abs(yaw) < 1.0;
    }

    /**
     * Checks if the current target is a scorer tag.
     * @apiNote For all scorers inclusive, including offseted + both alliances, this might be become deprecated later.
     * @return true if the current target is a scorer tag, false otherwise.
     */
    public boolean isScorer() {
        if (recentResult == null || !recentResult.hasTargets())
            return false;

        int tagId = recentResult.getBestTarget().getFiducialId();
        return tagId == AprilTags.Scorers.RedAlliance.kScorerFrontOffset
                || tagId == AprilTags.Scorers.RedAlliance.kScorerFront
                || tagId == AprilTags.Scorers.BlueAlliance.kScorerFrontOffset
                || tagId == AprilTags.Scorers.BlueAlliance.kScorerFront;
    }

    @Override
    public void periodic() {
        recentResult = mainCamera.getLatestResult();
        super.periodic();
    }
}
