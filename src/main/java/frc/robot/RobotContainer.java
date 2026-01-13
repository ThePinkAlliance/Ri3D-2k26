// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimberJoystickControl;
import frc.robot.commands.ShooterTuner;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private EditableDouble positionEditable = new EditableDouble("Shooter Speed", 80);
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.05)
          .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Shooter shooterSubsystem = new Shooter();
  private final Collector collectorSubsystem = new Collector();
  private final Climbers climberSubsystem = new Climbers(41, 44, 24);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandXboxController joystickBase = new CommandXboxController(0);
  private final CommandXboxController joystickTower = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        joystickBase.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        joystickBase.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        joystickBase.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    climberSubsystem.setDefaultCommand(
        new ClimberJoystickControl(
            () -> joystickTower.getLeftY(), () -> joystickTower.getRightY(), climberSubsystem));

    joystickBase
        .povUp()
        .whileTrue(new ShooterTuner(positionEditable, shooterSubsystem))
        .onFalse(shooterSubsystem.disengageCommand());

    joystickBase
        .x()
        .whileTrue(shooterSubsystem.setMotorSpeedCommand(1))
        .onFalse(shooterSubsystem.disengageCommand());

    joystickBase
        .rightBumper()
        .whileTrue(collectorSubsystem.setCollectorSpeedCommand(0.25))
        .onFalse(collectorSubsystem.setCollectorSpeedCommand(0));

    joystickBase.a().whileTrue(collectorSubsystem.setCollectorPitchCommand(8.35));
    joystickBase.b().whileTrue(collectorSubsystem.setCollectorPitchCommand(0));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystickBase
        .back()
        .and(joystickBase.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystickBase
        .back()
        .and(joystickBase.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystickBase
        .start()
        .and(joystickBase.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystickBase
        .start()
        .and(joystickBase.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystickBase.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));
  }
}
