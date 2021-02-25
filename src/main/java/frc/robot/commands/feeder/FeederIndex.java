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
  private static final long DELAY_US = (long)(0.5 * 1_000_000);

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Sensors.hasBallAtMouth() && !m_Sensors.hasBallInShooter()) {
      if(m_timeBallFirstSeen == 0) {
        m_timeBallFirstSeen = RobotController.getFPGATime();
      }
      if (RobotController.getFPGATime() >= m_timeBallFirstSeen + DELAY_US) {
        m_feeder.setFeeder(m_percentOutput);
      }
      else {
        m_feeder.setFeeder(0);
      }
    }
    else {
      m_feeder.setFeeder(0);
      m_timeBallFirstSeen = 0;
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