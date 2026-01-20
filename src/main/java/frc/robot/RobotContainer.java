// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.Constants.AprilTags;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsytem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsytem m_swerve = new SwerveSubsytem(Units.feetToMeters(14d), false);
  private final SwerveSubsytem m_oldSwerve = new SwerveSubsytem(Units.feetToMeters(14d), new File(Filesystem.getDeployDirectory(), "oldswerve"), false);
  private final Vision m_vision = new Vision();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Controller.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (DriverStation.isTest()) {
      m_driverController.b().onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));
      
      m_swerve.driveCommand(
          () -> m_driverController.getLeftX(),
          () -> m_driverController.getLeftY(),
          () -> m_driverController.getRightX());

      if (m_driverController.leftTrigger().getAsBoolean()) {
        m_swerve.driveAimScorer(
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getRightX(),
            m_vision,
            m_driverController.rightTrigger().getAsBoolean() ? () -> 1d : () -> .35d);
      }

      m_driverController.leftBumper().onTrue(m_swerve.driveForward());
    }

    m_driverController.b().onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));

    if (m_driverController.leftTrigger().getAsBoolean()) {
      m_swerve.driveAimScorer(
          () -> m_driverController.getLeftX(),
          () -> m_driverController.getLeftY(),
          () -> m_driverController.getRightX(),
          m_vision,
          m_driverController.rightTrigger().getAsBoolean() ? () -> 1d : () -> .35d);
    }
    m_swerve.driveCommand(
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
