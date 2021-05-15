/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class Unjam extends CommandBase {
  private Hopper hopper;
  /**
   * Creates a new Unjam.
   */
  public Unjam(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper=hopper;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.deployUnjammer();
  }
  @Override
  public void execute() {
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.retractUnjammer();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}