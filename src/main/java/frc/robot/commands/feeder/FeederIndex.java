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
import frc.robot.subsystems.Sensors;

public class FeederIndex extends CommandBase {
  private Feeder m_feeder;
  private Sensors m_Sensors;
  private Arm m_Arm;
  /**
   * states when arm is up include:
   *  0: No actions
   *  1: Ball at mouth
   *  2: Ball at mouth, feeder running
   *  3: no ball at mouth, feeder running
   *  4: no ball at mouth, feeder stopped
   *  5: ball at 1st switch, feeder running
   *  6: ball at 1st switch, feeder off (waiting for new ball to show up)
   *  8: ball at top switch
   * states when arm is down include: 
   *  0: No actions
   *  1: Ball at mouth
   *  2: Ball at mouth, feeder running
   *  3: no ball at mouth, feeder running
   *  4: no ball at mouth, feeder stopped
   *  5: ball at 1st switch (ball would be pushed out of feeder if it still ran)
   *  6: ball at top switch (shouldn't be possible in this mode)
   */
  private int m_state;

  /**
   * Sets feeder to desired percent output
   */
  public FeederIndex(Feeder feeder, Sensors sensors, Arm arm) {
    m_feeder = feeder;
    m_Sensors = sensors;
    m_Arm = arm;
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

    if (m_Arm.getCurrentArmPosition() < 10) {
      switch (m_state) {
        case 0:
          if (m_Sensors.hasBallAtMouth()) {
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
          m_feeder.setFeederPosition(0);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 5;
          }
          if (!m_Sensors.hasBallAtMouth()) {
            m_state = 2;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 2:
          m_feeder.setSensorPosition(0);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 5;
          }
          if (m_Sensors.hasBallAtMouth()) {
            m_state = 1;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 3:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeederPosition(0);
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 4:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeederPosition(0);
          break;

      }

    }
    if(m_Arm.getCurrentArmPosition() >= 10) {
      switch (m_state) {
        case 0:
          if (m_Sensors.hasBallAtMouth()) {
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
          m_feeder.setFeederPosition(0);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 5;
          }
          if (!m_Sensors.hasBallAtMouth()) {
            m_state = 2;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 2:
          m_feeder.setSensorPosition(0);
          if (m_feeder.checkForwardLimitSwitch() == true) {
            m_state = 5;
          }
          if (m_Sensors.hasBallAtMouth()) {
            m_state = 1;
          }
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          break;
        case 3:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeederPosition(0);
          if (m_feeder.checkReverseLimitSwitch() == true) {
            m_state = 4;
          }
          if(m_Sensors.hasBallAtMouth()) {
            m_state = 1;
          }
          break;
        case 4:
          m_feeder.setSensorPosition(0);
          m_feeder.setFeederPosition(0);
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
