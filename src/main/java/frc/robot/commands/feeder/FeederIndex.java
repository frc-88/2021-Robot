/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Sensors;

public class FeederIndex extends CommandBase {
  private Feeder m_feeder;
  private Sensors m_Sensors;
  /**
   * states include:
   * 0: 
  */
  private int m_state;
  
  
  /**
   * Sets feeder to desired percent output
   */
  public FeederIndex(Feeder feeder,Sensors sensors, double percentOutput) {
    m_feeder = feeder;
    m_Sensors = sensors;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
    m_feeder.setZeroOnBallSensed(true);
    m_feeder.setSensorPosition(-Integer.MIN_VALUE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_state) {
      case 0:
        m_feeder.setFeederPosition(0);
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
