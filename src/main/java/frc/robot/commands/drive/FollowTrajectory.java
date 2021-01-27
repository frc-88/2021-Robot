// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;

public class FollowTrajectory extends CommandBase {
  private Drive m_drive;
  private Sensors m_sensors;
  private Trajectory m_trajectory;
  private double m_duration;

  private int m_state;
  private Timer m_timer = new Timer();
  // TODO maybe allow for tuning of RamseteController parameters
  private RamseteController m_controller = new RamseteController();

  public FollowTrajectory(final Drive drive, final Sensors sensors, Trajectory trajectory) {
    m_drive = drive;
    m_sensors = sensors;
    m_trajectory = trajectory;
    m_duration = m_trajectory.getTotalTimeSeconds();

    addRequirements(m_drive);
    addRequirements(m_sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case 0: // Zero drive
        m_drive.zeroDrive();
        m_state++;
        break;
      case 1: // Check to make sure things are near zero
        if ((Math.abs(m_drive.getLeftPosition()) < 0.2) && (Math.abs(m_drive.getRightPosition()) < 0.2)
            && (Math.abs(m_sensors.getYaw()) < 2.0)) {
          m_state++;
        }
        break;
      case 2: // Reset the odometry to the starting pose of the Trajectory
        m_drive.resetOdometry(m_trajectory.getInitialPose(), Rotation2d.fromDegrees(m_sensors.getYaw()));
        m_state++;
        break;
      case 3: // reset the timer and go!
        m_timer.reset();
        m_timer.start();
        m_state++;
        // fall through right away to case 3
      case 4: // follow the trajectory, our final state
        // calculate what we need to do to be where we need to be 20ms from now.
        // TODO is that offset needed? Should I just calculate based on where I should
        // be now?
        m_drive.updateOdometry();
        double now = m_timer.get();
        Trajectory.State goal = m_trajectory.sample(now + 0.020);
        ChassisSpeeds targetSpeeds = m_controller.calculate(m_drive.getCurrentPose(), goal);

        DifferentialDriveWheelSpeeds wheelSpeeds = m_drive.wheelSpeedsFromChassisSpeeds(targetSpeeds);
        double leftSpeed = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightSpeed = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);

        SmartDashboard.putNumber("Trajectory left speed", leftSpeed);
        SmartDashboard.putNumber("Trajectory right speed", rightSpeed);

        m_drive.basicDriveLimited(leftSpeed, rightSpeed);

        if (m_drive.autoshift((leftSpeed + rightSpeed) / 2)) {
          m_drive.shiftToHigh();
        } else {
          m_drive.shiftToLow();
        }

        break;
      default:
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.basicDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_duration * 2.0;
  }
}
