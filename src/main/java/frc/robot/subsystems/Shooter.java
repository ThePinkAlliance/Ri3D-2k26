// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    this.shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_ID, CANBus.roboRIO());

    configureTalonFX();
  }

  private void configureTalonFX() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(60);
    configuration.Slot0.kP = 0.4;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    this.shooterMotor.getConfigurator().apply(configuration);
  }

  public void setMotorSpeed(double speed) {
    this.shooterMotor.set(speed);
  }

  public Command setMotorSpeedCommand(double speed) {
    return Commands.runOnce(() -> setMotorSpeed(speed), this);
  }

  /**
   * Sets the shooter velocity with a REV Lib api call.
   *
   * @param speed velocity in rotations per second.
   */
  public void setShooterVelocity(double speed) {
    this.shooterMotor.setControl(new VelocityVoltage(speed).withFeedForward(Volt.of(7)));
  }

  public void disengage() {
    this.shooterMotor.stopMotor();
  }

  public Command disengageCommand() {
    return Commands.runOnce(() -> disengage(), this);
  }

  /**
   * The command wrapper for setShooterVelocity. NOTE: No interrupt or termination handling.
   *
   * @param speed
   * @return Command
   */
  public Command setShooterVelocityCommand(double speed) {
    return Commands.runOnce(() -> setShooterVelocity(speed), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput(
        "Shooter/Flywheel Velocity", this.shooterMotor.getVelocity().getValueAsDouble());
  }
}
