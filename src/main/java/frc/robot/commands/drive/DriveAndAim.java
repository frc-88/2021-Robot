/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;
import frc.robot.util.SyncPIDController;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class DriveAndAim extends CommandBase {

  private final Drive m_drive;
  private final Sensors m_sensors;
  private final DoubleSupplier m_setpoint;
  private final DoubleSupplier m_maxSpeed;

  private final SyncPIDController m_drivePID;
  private final SyncPIDController m_aimPID;

  private boolean forwards;
  private double currentTarget;

  private static final double DISTANCE_INCREMENT = 1.;

  public DriveAndAim(final Drive drive, final Sensors sensors, DoubleSupplier setpoint, final DoubleSupplier maxSpeed, final PIDPreferenceConstants drivePIDConstants, final PIDPreferenceConstants aimPIDConstants) {
    m_drive = drive;
    m_sensors = sensors;
    m_setpoint = setpoint;
    m_maxSpeed = maxSpeed;

    m_drivePID = new SyncPIDController(drivePIDConstants);
    m_aimPID = new SyncPIDController(aimPIDConstants);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivePID.reset();
    m_drive.setOnLimelightTarget(false);
    m_drive.shiftToLow();

    currentTarget = (m_drive.getLeftPosition() + m_drive.getRightPosition()) / 2;
    forwards = m_setpoint.getAsDouble() > currentTarget;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (forwards) {
        currentTarget = Math.min(currentTarget + DISTANCE_INCREMENT, m_setpoint.getAsDouble());
    } else {
        currentTarget = Math.max(currentTarget - DISTANCE_INCREMENT, m_setpoint.getAsDouble());
    }

    double averagePosition = (m_drive.getLeftPosition() + m_drive.getRightPosition()) / 2;
    double speed = m_drivePID.calculateOutput(averagePosition, currentTarget);
    speed = Math.max(-m_maxSpeed.getAsDouble(), Math.min(m_maxSpeed.getAsDouble(), speed));
    double aim = 0;
    
    if (m_sensors.doesLimelightHaveTarget()) {
      aim = m_aimPID.calculateOutput(m_sensors.getShooterAngle(), 0);
    }

    m_drive.basicDriveLimited(speed + aim, speed - aim);

    m_drive.shiftToLow();

    if (Math.abs(averagePosition - m_setpoint.getAsDouble()) < .2 && Math.abs(m_sensors.getShooterAngle()) < 0.3) {
      m_drive.setOnLimelightTarget(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drive.setOnLimelightTarget(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
