/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Sensors;

public class ZeroYaw extends InstantCommand {
  private Sensors sensors;

  public ZeroYaw(Sensors sensors) {
    this.sensors=sensors;
    addRequirements(sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.navx.zeroYaw();
  }
}
