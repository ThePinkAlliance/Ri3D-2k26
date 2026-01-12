// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberJoystickControl extends Command {
  private Supplier<Double> leftYAxisSupplier, rightYAxisSupplier;
  private Climbers climberSubsystem;

  /** Creates a new ClimberJoystickControl. */
  public ClimberJoystickControl(
      Supplier<Double> leftYAxisSupplier,
      Supplier<Double> rightYAxisSupplier,
      Climbers climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.leftYAxisSupplier = leftYAxisSupplier;
    this.rightYAxisSupplier = rightYAxisSupplier;
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = MathUtil.clamp(leftYAxisSupplier.get().doubleValue(), -0.25, 0.25);
    double rightSpeed = MathUtil.clamp(rightYAxisSupplier.get().doubleValue(), -0.25, 0.25);

    climberSubsystem.setRachetingClimberPower(leftSpeed);
    climberSubsystem.setExtendingClimberPower(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setExtendingClimberPower(0);
    climberSubsystem.setRachetingClimberPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
