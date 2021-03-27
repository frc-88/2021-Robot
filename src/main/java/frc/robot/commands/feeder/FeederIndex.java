/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feeder;

import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Hopper;

public class FeederIndex extends CommandBase {
  private Feeder m_feeder;
  private Hopper m_hopper;
  private double m_percentOutput;
  private FeederIndex m_initializeFrom;
  private Sensors m_Sensors;
  protected long m_timeBallFirstSeen;
  protected long m_timeBallLastSeen;
  private static final long FIRST_SEEN_DELAY_US = (long)(0.5 * 1_000_000);
  private static final long LAST_SEEN_DELAY_US = (long)(0.075 * 1_000_000);
  /**
   * states include: 
   * 0 - mouth sensor not triggered, feeder not running
   * 1 - mouth sensor triggered, waiting to run feeder.
   * 2 - mouth sensor triggered, running feeder.
   * 3 - mouth sensor not triggered, feeder still running.
   * 4 - shooter sensor triggered, feeder shut down no matter what.
   * 5 - run feeder until ball seen.
*/
  public int m_state;
  
  
  /**
   * Sets feeder to desired percent output
   */
  public FeederIndex(Feeder feeder,Hopper hopper,Sensors sensors, double percentOutput, FeederIndex initializeFrom) {
    m_feeder = feeder;
    m_percentOutput = percentOutput;
    m_Sensors = sensors;
    m_hopper = hopper;
    m_initializeFrom = initializeFrom;
    addRequirements(m_feeder);
  }

  public FeederIndex(Feeder feeder,Hopper hopper,Sensors sensors, double percentOutput) {
    this(feeder, hopper, sensors, percentOutput, null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Objects.nonNull(m_initializeFrom)) {
      m_timeBallFirstSeen = m_initializeFrom.m_timeBallFirstSeen;
      m_timeBallLastSeen = m_initializeFrom.m_timeBallLastSeen;
      m_state = m_initializeFrom.m_state;
    } else {
      m_timeBallFirstSeen = 0;
      m_timeBallLastSeen = 0;
      m_state = 5;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_state) {
      case 0:
      m_feeder.setFeeder(0);
      m_hopper.setPercentOutput(0.3);

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
      m_hopper.setPercentOutput(0.3);
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
      m_hopper.setPercentOutput(0);
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
      m_hopper.setPercentOutput(0.15);
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
      m_hopper.setPercentOutput(0);
      if (!m_Sensors.hasBallInShooter()) {
        m_state = 0;
      }
      break;

      case 5:
      m_feeder.setFeeder(m_percentOutput);
      m_hopper.setPercentOutput(0.3);

      if (m_Sensors.hasBallAtMouth()) {
        m_state = 2;
      }
      if (m_Sensors.hasBallInShooter()) {
        m_state = 4;
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
