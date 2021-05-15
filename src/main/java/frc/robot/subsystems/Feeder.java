/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Feeder extends SubsystemBase {
  private TalonSRX m_feeder = new TalonSRX(Constants.FEEDER_MOTOR);
  
  private DoublePreferenceConstant feeder_kP;
  private DoublePreferenceConstant feeder_kD;

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    m_feeder.configFactoryDefault();
    m_feeder.enableVoltageCompensation(true);
    m_feeder.setInverted(true);
    m_feeder.setSensorPhase(false);
    m_feeder.setNeutralMode(NeutralMode.Brake);
    m_feeder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 0.001;
    currentLimit.triggerThresholdCurrent = 55;
    currentLimit.currentLimit = 55;
    m_feeder.configSupplyCurrentLimit(currentLimit);

    feeder_kP = new DoublePreferenceConstant("Feeder kP", 0);
    feeder_kP.addChangeHandler((Double kP) -> m_feeder.config_kP(0, kP));
    m_feeder.config_kP(0, feeder_kP.getValue());

    feeder_kD = new DoublePreferenceConstant("Feeder kD", 0);
    feeder_kD.addChangeHandler((Double kD) -> m_feeder.config_kD(0, kD));
    m_feeder.config_kD(0, feeder_kD.getValue());
  }

  public void setFeeder(double percentOutput) {
    m_feeder.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setFeederPosition(double position) {
    m_feeder.set(ControlMode.Position, position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder Position", m_feeder.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
}
