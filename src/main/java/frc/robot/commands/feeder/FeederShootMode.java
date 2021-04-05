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
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

public class FeederShootMode extends CommandBase {
  private Feeder m_feeder;
  private double m_percentOutput;
  private Sensors m_Sensors;
  private long m_timeBallLastSeen;
  private long m_timeBallShot;
  private Hopper m_hopper;
  private static final long TIMER_0_US = (long)(0.03 * 1_000_000);
  private static final long TIMER_1_US = (long)(1.5 * 1_000_000);
  /**
   * states include: 
   * 0 - Ball is seen, feeder is running.
   * 1 - Ball is not longer seen, feeder is running, Timer 0 runs.
   * 2 - Timer 0 is done, feeder stopped, Timer 1 runs.
   * 3 - Timer 1 is done, feeder is running (no ball seen).
*/
  private int m_state;
  
  
  /**
   * Sets feeder to desired percent output
   */
  public FeederShootMode(Feeder feeder,Sensors sensors,Hopper hopper, double percentOutput) {
    m_feeder = feeder;
    m_percentOutput = percentOutput;
    m_Sensors = sensors;
    addRequirements(m_feeder);
    m_hopper = hopper;
    addRequirements(m_hopper);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeBallLastSeen = 0;
    m_timeBallShot = 0;
    m_state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_state) {
        case 0:
        m_feeder.setFeeder(m_percentOutput);
        m_hopper.setPercentOutput(Constants.HOPPER_SHOOT_PERCENT_OUTPUT);
        if (!m_Sensors.hasBallInShooter()) {
            m_state = 1;
            m_timeBallLastSeen = RobotController.getFPGATime();
        }
        break;
        
        case 1:
        m_feeder.setFeeder(m_percentOutput);
        m_hopper.setPercentOutput(Constants.HOPPER_SHOOT_PERCENT_OUTPUT);
        if (RobotController.getFPGATime() >= m_timeBallLastSeen + TIMER_0_US) {
            m_state = 2;
            m_timeBallShot = RobotController.getFPGATime();
        }
        break;

        case 2:
        m_feeder.setFeeder(0);
        m_hopper.setPercentOutput(0);
        if (RobotController.getFPGATime() >= m_timeBallShot + TIMER_1_US) {
            m_state = 3;
        }
        break;

        case 3:
        m_feeder.setFeeder(m_percentOutput);
        m_hopper.setPercentOutput(Constants.HOPPER_SHOOT_PERCENT_OUTPUT);
        if (m_Sensors.hasBallInShooter()) {
            m_state = 0;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setFeeder(m_percentOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
