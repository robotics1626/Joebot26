package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;


import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;

  // Tune these
  private static final double MAX_SPEED_MPS = Units.feetToMeters(16.0); // pick real number later
  private static final double MAX_ANGULAR_RAD_PER_SEC = Math.PI * 2.0;  // ~1 rotation/sec as a start

  public Swerve() {
    try {
      File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve");
      swerve = new SwerveParser(swerveDir).createSwerveDrive(MAX_SPEED_MPS);
      zeroGyro();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load swerve config from deploy/swerve", e);
    }
  }

  /** Drive with normalized inputs (-1..1). */
  public void drive(
      double xSpeedNorm,
      double ySpeedNorm,
      double omegaNorm,
      boolean fieldRelative,
      boolean openLoop
  ) {
    double vx = xSpeedNorm * MAX_SPEED_MPS;
    double vy = ySpeedNorm * MAX_SPEED_MPS;
    double omega = omegaNorm * MAX_ANGULAR_RAD_PER_SEC;

    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading())
        : new ChassisSpeeds(vx, vy, omega);

    // YAGSL has a few drive entry points depending on version.
    // This one is common: drive(ChassisSpeeds, openLoop)
    swerve.drive(speeds, new Translation2d(0d, 0d));
  }

  /** Lock the wheels in an "X" to resist being pushed. */
  public void lockWheels() {
    swerve.lockPose();
  }

  /** Zero gyro so current heading becomes 0° */
  public void zeroGyro() {
    swerve.zeroGyro();
  }

  /** Current robot heading (gyro yaw) */
  public Rotation2d getHeading() {
    return swerve.getYaw();
  }

  /** Robot pose from odometry */
  public Pose2d getPose() {
    return swerve.getPose();
  }

  /** Reset pose/odometry (useful at start of auto) */
  public void resetPose(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  @Override
  public void periodic() {
    swerve.updateOdometry();
  }
}
