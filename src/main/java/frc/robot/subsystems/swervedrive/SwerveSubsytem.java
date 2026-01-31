package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Controller;
import frc.robot.Constants.Motors.Swerve;

import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class SwerveSubsytem extends SubsystemBase {
  private final SwerveDrive sd;
  public final double maxmps;
  private final boolean fieldrelative;

  // Aim PID: use radians for measurement, radians/sec for output.
  private final PIDController aimPID =
      new PIDController(Swerve.PIDValues.kAimP, Swerve.PIDValues.kAimI, Swerve.PIDValues.kAimD);

  /**
   * Applies deadband to joystick inputs.
   *
   * @param v Input value
   * @return Value after deadband
   */
  private static double deadband(double v) {
    return Math.abs(v) < Controller.kJoystickDeadband ? 0.0 : v;
  }

  /**
   * Creates a new SwerveSubsytem using deploy/swerve config.
   *
   * @param maxmps        Max meters per second
   * @param fieldrelative Whether to use field-relative control
   */
  public SwerveSubsytem(double maxmps, boolean fieldrelative) {
    this(maxmps, new File(Filesystem.getDeployDirectory(), "swerve"), fieldrelative);
  }

  /**
   * Creates a new SwerveSubsytem using a provided config directory.
   *
   * @param maxmps          Max meters per second
   * @param configDirectory Directory containing the "swerve" folder OR the folder that contains it
   * @param fieldrelative   Whether to use field-relative control
   */
  public SwerveSubsytem(double maxmps, File configDirectory, boolean fieldrelative) {
    try {
      //
      File swerveDir = configDirectory.getName().equalsIgnoreCase("swerve")
          ? configDirectory
          : new File(configDirectory, "swerve");

      this.sd = new SwerveParser(swerveDir).createSwerveDrive(maxmps);
    } catch (IOException e) {
      throw new RuntimeException("Failed to create swervedrive: " + e.getMessage(), e);
    }

    this.maxmps = maxmps;
    this.fieldrelative = fieldrelative;
    this.sd.setCosineCompensator(false);
    this.sd.setHeadingCorrection(false);
    this.sd.setAngularVelocityCompensation(true, true, 1.0);

    aimPID.enableContinuousInput(-Math.PI, Math.PI);
    aimPID.setTolerance(Math.toRadians(1.0)); // 1 degree tolerance
  }

  @Override
  public void periodic() {
    // TODO: update odometry
  }

  /** Zero the gyro heading. */
  public void zeroGyro() {
    sd.zeroGyro();
  }

  /** Simple command to drive forward at 1 m/s (robot-relative). */
  public Command driveForward() {
    return run(() -> sd.drive(new Translation2d(1, 0), 0.0, false, false))
        .finallyDo(() -> sd.drive(new Translation2d(0, 0), 0.0, false, false));
  }

  /**
   * Standard teleop drive command.
   *
   * @param x   X axis input (-1..1)
   * @param y   Y axis input (-1..1)
   * @param rot Rotation input (-1..1)
   */
  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return run(() -> {
      double xRaw = deadband(x.getAsDouble());
      double yRaw = deadband(y.getAsDouble());
      double rRaw = deadband(rot.getAsDouble());

      // shaping
      double xIn = Math.copySign(xRaw * xRaw, xRaw);
      double yIn = Math.copySign(yRaw * yRaw, yRaw);
      double rIn = Math.pow(rRaw, 3);

      double vx = xIn * sd.getMaximumChassisVelocity();
      double vy = yIn * sd.getMaximumChassisVelocity();
      double omega = rIn * sd.getMaximumChassisAngularVelocity();

      sd.drive(new Translation2d(vx, vy), omega, fieldrelative, false);
    });
  }

  /**
   * Drive + auto-turn toward scorer AprilTags when visible.
   shows=-:
   * - Translation comes from driver (x,y)
   * - Rotation is vision PID if target exists, else fallback rotation stick
   *
   * @param x           driver x (-1..1)
   * @param y           driver y (-1..1)
   * @param rotFallback driver rotation stick (-1..1) used when no target
   * @param vision      Vision subsystem (must expose hasScorerTarget + yaw degrees)
   * @param slowMode    multiplier supplier (1.0 normal, e.g. 0.35 slow)
   */
  public Command driveAimScorer(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rotFallback,
      Vision vision,
      DoubleSupplier slowMode
  ) {
    return run(() -> {
      // same deadband + shaping as teleop
      double xRaw = deadband(x.getAsDouble());
      double yRaw = deadband(y.getAsDouble());
      double rRaw = deadband(rotFallback.getAsDouble());

      double xIn = Math.copySign(xRaw * xRaw, xRaw);
      double yIn = Math.copySign(yRaw * yRaw, yRaw);
      double rIn = Math.pow(rRaw, 3);

      double vx = xIn * sd.getMaximumChassisVelocity();
      double vy = yIn * sd.getMaximumChassisVelocity();

      Translation2d translation = SwerveMath.scaleTranslation(
          new Translation2d(vx, vy),
          0.8 * slowMode.getAsDouble()
      );

      double omega;
      if (vision.hasScorerTarget()) {
        double yawRad = Math.toRadians(vision.getScorerYawDeg());
        omega = aimPID.calculate(yawRad, 0.0); // rad -> rad/s
      } else {
        omega = rIn * sd.getMaximumChassisAngularVelocity();
      }

      omega = MathUtil.clamp(
          omega,
          -sd.getMaximumChassisAngularVelocity(),
           sd.getMaximumChassisAngularVelocity()
      );

      sd.drive(translation, omega, fieldrelative, false);
    });
  }
}
