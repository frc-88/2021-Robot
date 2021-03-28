/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

/**
 * Run both sides at the same speed forward to feed power cells into the
 * shooter.
 */
public class HopperShootMode extends CommandBase {
  private Hopper m_hopper;
  private Sensors m_sensors;
  private double m_percentOutput;

  public HopperShootMode(Hopper hopper, Sensors sensors) {
    this(hopper,sensors, Constants.HOPPER_SHOOT_PERCENT_OUTPUT);
  }

  public HopperShootMode(Hopper hopper, Sensors sensors, double percentOutput) {
    m_hopper = hopper;
    m_sensors = sensors;
    m_percentOutput = percentOutput;
    addRequirements(m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.setPercentOutput(m_sensors.hasBallAtMouth() ? 0:  m_percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
