/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

public class FeederIndex extends CommandBase {
  private Feeder m_feeder;
  private Sensors m_Sensors;
  private Arm m_arm;
  private Hopper m_hopper;
  /**
   * states when arm is up include:
   *  0: No actions
   *  1: Ball at mouth
   *  2: Ball at mouth, feeder running
   *  3: no ball at mouth, feeder running
   *  4: no ball at mouth, feeder stopped
   *  
   * states when arm is down include: 
   *  0: No actions
   *  1: Ball at mouth
   *  2: Ball at mouth, feeder running
   *  3: no ball at mouth, feeder running
   *  4: no ball at mouth, feeder stopped
   */
  private int m_state;

  /**
   * Sets feeder to desired percent output
   */
  public FeederIndex(Feeder feeder, Sensors sensors, Arm arm, Hopper hopper) {
    m_feeder = feeder;
    m_Sensors = sensors;
    m_arm = arm;
    m_hopper = hopper;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
    m_feeder.setZeroOnBallSensed(true);
    m_feeder.resetOnBallSensed(true);
    m_feeder.setSensorPosition(-Integer.MIN_VALUE + 1);
    m_feeder.configLimitSwitches(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.getCurrentArmPosition() <= 10) {
      switch (m_state) {
        case 0:
          m_hopper.setPercentOutput(0.25);
          if (m_Sensors.hasBallAtMouth() == true) {
            m_state = 1;
          }
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 1:
          m_feeder.setFeeder(0.5);
          m_hopper.setPercentOutput(0);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (m_Sensors.hasBallAtMouth() == true) {
            m_feeder.setFeeder(.5);
            m_state = 1;
          }
          if (m_Sensors.hasBallAtMouth() == false) {
            m_state = 2;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 2:
          m_hopper.setPercentOutput(0.1);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (m_Sensors.hasBallAtMouth() == true) {
            m_state = 1;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 3:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeeder(0);
          m_hopper.setPercentOutput(0.1);
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          if (m_Sensors.hasBallAtMouth()) {
            m_state = 5;
          }
          break;
        case 4:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeeder(0);
          if (m_Sensors.hasBallAtMouth() == true){
            m_hopper.setPercentOutput(0);
          }
          else{
            m_hopper.setPercentOutput(.1);
          }
          
          break;
        
        case 5:
          m_hopper.setPercentOutput(0);
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;

      }

    }
    if(m_arm.getCurrentArmPosition() > 10) {
      switch (m_state) {
        case 0:
          if (m_Sensors.hasBallAtMouth() == true) {
            m_state = 1;
          }
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 1:
          m_feeder.setFeeder(1);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (!m_Sensors.hasBallAtMouth() == true) {
            m_state = 2;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 2:
          m_hopper.setPercentOutput(0.1);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 3;
          }
          if (m_Sensors.hasBallAtMouth() == true) {
            m_state = 1;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 3:
          m_feeder.setSensorPosition(0);
          //m_feeder.setFeederPosition(0);
          m_feeder.setFeeder(0);
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          if(m_Sensors.hasBallAtMouth() == true) {
            m_state = 1;
          }
          break;
        case 4:
          m_feeder.setSensorPosition(0);
          //m_feeder.setFeederPosition(0);
          m_feeder.setFeeder(0);
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setSensorPosition(0);
    //m_feeder.setFeederPosition(0);
    m_feeder.setFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
