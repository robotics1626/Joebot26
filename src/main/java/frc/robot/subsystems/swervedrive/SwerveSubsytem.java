package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.numbers.N1;s

public class SwerveSubsytem extends SubsystemBase {
  private SwerveDrive sd;
  public double maxmps;

  /**
   * Creates a new SwerveSubsytem.
   * 
   * @param maxmps          Max meters per second
   * @param configDirectory Optional configuration directory ( if there is a
   *                        seperate configuration to use )
   */
  public SwerveSubsytem(double maxmps, Optional<File> configDirectory) {
    try {
      this.sd = new SwerveParser(configDirectory.orElse(new File(Filesystem.getDeployDirectory(), "swerve")))
          .createSwerveDrive(maxmps);
    } catch (IOException e) {
      e.printStackTrace();
    }
    this.maxmps = maxmps;

    this.sd.setCosineCompensator(false);
    this.sd.setHeadingCorrection(false);
    this.sd.setAngularVelocityCompensation(true, true, 1.d);
  }

  @Override
  public void periodic() {
    // Update the swerve drive subsystem
  }

  public Command driveForward() {
    return run(() -> {
      this.sd.drive(new Translation2d(1, 0), 0.0, false, false);
    }).finallyDo(() -> {
      this.sd.drive(new Translation2d(0, 0), 0, false, false);
    });
  }

  public void zeroGyro() {
    this.sd.zeroGyro();
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier anglex) {
    return run(() -> {

      this.sd.drive(SwerveMath.scaleTranslation(new Translation2d(
          x.getAsDouble() * this.sd.getMaximumChassisVelocity(),
          y.getAsDouble() * this.sd.getMaximumChassisVelocity()), .8d),
          Math.pow(anglex.getAsDouble(), 3) * this.sd.getMaximumChassisAngularVelocity(), true, false);
    });
  }
}