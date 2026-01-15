package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Eis extends SubsystemBase {
    private PhotonCamera mainCamera;
    private PhotonPipelineResult recentResult;

    Eis(String CameraName) {
        mainCamera = new PhotonCamera(CameraName);
    }

    public PhotonTrackedTarget mainTarget() {
        return recentResult.getBestTarget();
    }

    public double returnYaw() {
        return mainTarget().yaw;
    }

    @Override
    public void periodic() {
        recentResult = mainCamera.getLatestResult();
        super.periodic();
    }
}
