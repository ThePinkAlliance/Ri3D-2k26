// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    this.shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_ID);

    configureTalonFX();
  }

  private void configureTalonFX() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(60);
    configuration.ClosedLoopGeneral = new ClosedLoopGeneralConfigs();

    this.shooterMotor.getConfigurator().apply(configuration);
  }

  public void setShooterVelocity(double speed) {
    this.shooterMotor.setControl(new VelocityVoltage(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Shooter/Flywheel Velocity", this.shooterMotor.getVelocity().getValueAsDouble());
  }
}
