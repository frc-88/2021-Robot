/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Feeder extends SubsystemBase {
  private TalonFX m_feeder = new TalonFX(Constants.FEEDER_MOTOR);
  
  private DoublePreferenceConstant feeder_kP;
  private DoublePreferenceConstant feeder_kD;

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    m_feeder.configFactoryDefault();
    //TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    //allConfigs.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
    //m_feeder.configAllSettings(allConfigs);
    
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

  public void setZeroOnBallSensed(boolean enabled) {
    m_feeder.configClearPositionOnLimitR(enabled, 0);
  }

  public void setSensorPosition(double position) {
    m_feeder.setSelectedSensorPosition(position);
  }

  public void configLimitSwitches(boolean state) {
    if(state) {
      m_feeder.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }
    else {
      m_feeder.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyClosed);
    }
  }

  public boolean checkForwardLimitSwitch() {
    return m_feeder.isFwdLimitSwitchClosed() == 1;
  }

  public boolean checkReverseLimitSwitch() {
    return m_feeder.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder Position", m_feeder.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
}
