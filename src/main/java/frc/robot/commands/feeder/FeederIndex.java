/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Sensors;

public class FeederIndex extends CommandBase {
  private Feeder m_feeder;
  private double m_percentOutput;
  private Sensors m_Sensors;
  private long m_timeBallFirstSeen;
  private long m_timeBallLastSeen;
  private static final long FIRST_SEEN_DELAY_US = (long)(0.5 * 1_000_000);
  private static final long LAST_SEEN_DELAY_US = (long)(0.075 * 1_000_000);
  /**
   * states include: 
   * 0 - mouth sensor not triggered, feeder not running
   * 1 - mouth sensor triggered, waiting to run feeder.
   * 2 - mouth sensor triggered, running feeder.
   * 3 - mouth sensor not triggered, feeder still running.
   * 4 - shooter sensor triggered, feeder shut down no matter what.
*/
  private int m_state;
  
  
  /**
   * Sets feeder to desired percent output
   */
  public FeederIndex(Feeder feeder,Sensors sensors, double percentOutput) {
    m_feeder = feeder;
    m_percentOutput = percentOutput;
    m_Sensors = sensors;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeBallFirstSeen = 0;
    m_timeBallLastSeen = 0;
    m_state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_state) {
      case 0:
      m_feeder.setFeeder(0);

      if (m_Sensors.hasBallAtMouth()) {
        m_state = 1;
        m_timeBallFirstSeen = RobotController.getFPGATime();
      }
      if (m_Sensors.hasBallInShooter()) {
        m_state = 4;
      }
      break;

      case 1:
      m_feeder.setFeeder(0);
      if (RobotController.getFPGATime() >= m_timeBallFirstSeen + FIRST_SEEN_DELAY_US) {
        m_state = 2;
      }
      if (!m_Sensors.hasBallAtMouth()) {
        m_state = 0;
      }
      if (m_Sensors.hasBallInShooter()) {
        m_state = 4;
      }
      break;

      case 2:
      m_feeder.setFeeder(m_percentOutput);
      if (!m_Sensors.hasBallAtMouth()) {
        m_timeBallLastSeen = RobotController.getFPGATime();
        m_state = 3;
      }
      if (m_Sensors.hasBallInShooter()) {
        m_state = 4;
      }
      break;

      case 3:
      m_feeder.setFeeder(m_percentOutput);
      if (RobotController.getFPGATime() >= m_timeBallLastSeen + LAST_SEEN_DELAY_US) {
        m_state = 0;
      }
      if (m_Sensors.hasBallAtMouth()) {
        m_state = 2;
      }
      if (m_Sensors.hasBallInShooter()) {
        m_state = 4;
      }
      break;

      case 4:
      m_feeder.setFeeder(0);
      if (!m_Sensors.hasBallInShooter()) {
        m_state = 0;
      }
      break;

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
