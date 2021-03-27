/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;
import frc.robot.util.SyncPIDController;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.TrapezoidalProfileController;

public class DriveAndAim extends CommandBase {

  private final Drive m_drive;
  private final Sensors m_sensors;
  private final BooleanSupplier m_limelightAim;
  private final DoubleSupplier m_setpoint;
  private final DoubleSupplier m_maxSpeed;
  private final DoubleSupplier m_maxAcceleration;
  private final DoubleSupplier m_gyroSetpoint;

  private final TrapezoidalProfileController m_drivePID;
  private final SyncPIDController m_aimPID;

  public DriveAndAim(final Drive drive, final Sensors sensors, final BooleanSupplier limelightAim, final DoubleSupplier setpoint, final DoubleSupplier maxSpeed, final DoubleSupplier maxAcceleration, final DoubleSupplier gyroSetpoint, final PIDPreferenceConstants drivePIDConstants, final PIDPreferenceConstants aimPIDConstants) {
    m_drive = drive;
    m_sensors = sensors;
    m_limelightAim = limelightAim;
    m_setpoint = setpoint;
    m_maxSpeed = maxSpeed;
    m_maxAcceleration = maxAcceleration;
    m_gyroSetpoint = gyroSetpoint;

    m_drivePID = new TrapezoidalProfileController(maxSpeed.getAsDouble(), maxAcceleration.getAsDouble(), new SyncPIDController(drivePIDConstants));
    m_aimPID = new SyncPIDController(aimPIDConstants);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = (m_drive.getLeftPosition() + m_drive.getRightPosition()) / 2;
    m_drivePID.setMaxSpeed(m_maxSpeed.getAsDouble());
    m_drivePID.setMaxAcceleration(m_maxAcceleration.getAsDouble());
    m_drivePID.reset(currentPosition);
    m_drive.setOnLimelightTarget(false);
    m_drivePID.setTargetVelocity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get position to PID on
    double averagePosition = (m_drive.getLeftPosition() + m_drive.getRightPosition()) / 2;

    // Calculate the driving speed
    m_drivePID.setTargetPosition(m_setpoint.getAsDouble());
    double speed = m_drivePID.calculateCommandVelocity(averagePosition, (m_drive.getLeftSpeed() + m_drive.getRightSpeed()) / 2);

    // Determine gear
    m_drive.setMaxSpeed(m_maxSpeed.getAsDouble());
    if (m_drive.autoshift(speed)) {
      m_drive.shiftToHigh();
    } else {
      m_drive.shiftToLow();
    }

    // Calculate the aim PID
    double aim = 0;
    if (m_limelightAim.getAsBoolean()) {
      if (m_sensors.doesLimelightHaveTarget()) {
        aim = m_aimPID.calculateOutput(m_sensors.getShooterAngle(), 1);
      }
    } else {
      aim = m_aimPID.calculateOutput(m_sensors.getYaw(), m_gyroSetpoint.getAsDouble());
    }

    // Command the drive
    m_drive.basicDriveLimited(speed + aim, speed - aim);

    // Determine if we are in shooting position
    if (Math.abs(averagePosition - m_setpoint.getAsDouble()) < .2 && m_sensors.doesLimelightHaveTarget() && Math.abs(m_sensors.getShooterAngle() - 1) < 0.3) {
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
