// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
  private SparkMax collectorPitchMotor;
  private SparkFlex collectorMotor;

  /** Creates a new Collector. */
  public Collector() {
    this.collectorMotor = new SparkFlex(Constants.COLLECTOR_MOTOR_ID, MotorType.kBrushless);
    this.collectorPitchMotor =
        new SparkMax(Constants.COLLECTOR_PITCH_MOTOR_ID, MotorType.kBrushless);

    configureMotor();
    configureCollectorPitch();
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    this.collectorMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureCollectorPitch() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.p(0.12);
    config.closedLoop.maxOutput(0.25);
    config.closedLoop.minOutput(-0.25);

    // Reset collector pitch motor postiion.
    this.collectorPitchMotor.getEncoder().setPosition(0);

    this.collectorPitchMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCollectorSpeed(double speed) {
    collectorMotor.set(speed);
  }

  public void setCollectorPitch(double angle) {
    this.collectorPitchMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
  }

  public Command setCollectorPitchCommand(double pitch) {
    return Commands.runOnce(() -> this.setCollectorPitch(pitch), this);
  }

  public Command setCollectorSpeedCommand(double speed) {
    return Commands.runOnce(() -> this.setCollectorSpeed(speed), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Collector/Angle", this.collectorPitchMotor.getEncoder().getPosition());
    Logger.recordOutput("Collector/Velocity", this.collectorMotor.getEncoder().getVelocity());
  }
}
