/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private TalonFX m_motor;
  private TalonFXConfiguration m_config;
  private DoubleSolenoid m_unjammer;

  /**
   * we can count to five
   * which differs from a certain
   * holy hand grenade
   */
  public Hopper() {
    m_motor = new TalonFX(Constants.HOPPER);
    m_motor.setInverted(InvertType.InvertMotorOutput);
    m_unjammer = new DoubleSolenoid(Constants.HOPPER_UNJAMMER_PCM, Constants.HOPPER_UNJAMMER_DEPLOY, Constants.HOPPER_UNJAMMER_RETRACT);
  }

  public void setPercentOutput(double percentOutput) {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void deployUnjammer() {
    m_unjammer.set(Value.kForward);
  }

  public void retractUnjammer(){
    m_unjammer.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // Motor data
    SmartDashboard.putNumber("Hopper velocity", m_motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Hopper stator current", m_motor.getStatorCurrent());
    SmartDashboard.putNumber("Hopper supply current", m_motor.getSupplyCurrent());
  }




}
