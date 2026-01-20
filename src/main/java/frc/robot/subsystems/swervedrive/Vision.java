package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors.Swerve;
import frc.robot.subsystems.Eis;

public class Vision extends SubsystemBase {
    private final Eis eis;

    private boolean hasScorerTarget = false;
    private double scorerYawDeg = 0.0;

    public Vision() {
        eis = new Eis("Blinky");
    }

    @Override
    public void periodic() {
        hasScorerTarget = eis.isScorer();
        scorerYawDeg = hasScorerTarget ? eis.getTargetYaw() : 0.0;
    }

    public boolean hasScorerTarget() {
        return hasScorerTarget;
    }

    /** Photon yaw is degrees. */
    public double getScorerYawDeg() {
        return scorerYawDeg;
    }

    public boolean scorerYawInTol(double tolDeg) {
        return hasScorerTarget && Math.abs(scorerYawDeg) <= tolDeg;
    }
}
