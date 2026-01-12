// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberPair extends SubsystemBase {
  private SparkMax rachetingClimber;
  private TalonFX extendingClimber;

  /** Creates a new Climbers. */
  public ClimberPair(int rachetingClimberId, int extendingClimberId) {
    this.rachetingClimber = new SparkMax(rachetingClimberId, MotorType.kBrushless);
    this.extendingClimber = new TalonFX(extendingClimberId, CANBus.roboRIO());
  }

  public void configureRachetingClimber() {
    SparkMaxConfig config = new SparkMaxConfig();

    this.rachetingClimber.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configureExtendingClimber() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.5;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 2;

    this.extendingClimber.getConfigurator().apply(config);
  }

  public void setRachetingClimberSetpoint(double position) {
    this.rachetingClimber.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
  }

  public void setExtendingClimberSetpoint(double position) {
    this.extendingClimber.setControl(new PositionVoltage(position));
  }

  public Command setRachetingClimberSetpointCommand(double position) {
    return Commands.runOnce(() -> setRachetingClimberSetpoint(position), this);
  }

  public Command setExtendingClimberSetpointCommand(double position) {
    return Commands.runOnce(() -> setExtendingClimberSetpoint(position), this);
  }

  public double getExtendingClimberPosition() {
    return this.extendingClimber.getPosition().getValueAsDouble();
  }

  public double getRachetingClimberPosition() {
    return this.rachetingClimber.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput(
        "Climbers/racheting-" + rachetingClimber.getDeviceId() + "/position",
        rachetingClimber.getEncoder().getPosition());
    Logger.recordOutput(
        "Climbers/extending-" + rachetingClimber.getDeviceId() + "/position",
        extendingClimber.getPosition().getValueAsDouble());
  }
}
