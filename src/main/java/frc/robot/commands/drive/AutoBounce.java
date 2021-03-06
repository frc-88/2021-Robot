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

public class AutoBounce extends CommandBase {
  private Drive m_drive;
  private Sensors m_sensors;
  private Trajectory m_trajectory[];
  private RamseteController m_controller = new RamseteController();
  private Timer m_timer = new Timer();

  private int m_state;
  private int m_currentTraj;
  private double m_duration;

  public AutoBounce(final Drive drive, final Sensors sensors) {
    m_drive = drive;
    m_sensors = sensors;

    m_trajectory = new Trajectory[4];
    m_trajectory[0] = m_drive.trajectories.bounce1;
    m_trajectory[1] = m_drive.trajectories.bounce2;
    m_trajectory[2] = m_drive.trajectories.bounce3;
    m_trajectory[3] = m_drive.trajectories.bounce4;

    addRequirements(m_drive);
    addRequirements(m_sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
    m_currentTraj = 0;

    m_drive.setBrakeMode();
    m_drive.shiftToHigh();
    m_drive.zeroDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (m_state) {
      case 0: // Zero drive
        m_timer.reset();
        m_duration = m_trajectory[m_currentTraj].getTotalTimeSeconds();
        m_drive.resetEncoderPositions();
        m_state++;
        break;

      case 1: // Check to make sure things are near zero
        if ((Math.abs(m_drive.getLeftPosition()) < 0.2) && (Math.abs(m_drive.getRightPosition()) < 0.2)) {
          m_state++;
        }
        break;

      case 2: // Reset the odometry to the starting pose of the Trajectory
        m_drive.resetOdometry(m_trajectory[m_currentTraj].getInitialPose(), Rotation2d.fromDegrees(m_sensors.getYaw()));
        m_state++;
        break;

      case 3: // reset the timer and go!
        m_timer.start();
        m_state++;
        // fall through right away to case 4

      case 4: // follow the trajectory
        m_drive.updateOdometry();
        double now = m_timer.get();

        if (m_timer.get() < m_duration) {
          Trajectory.State goal = m_trajectory[m_currentTraj].sample(now);
          ChassisSpeeds targetSpeeds = m_controller.calculate(m_drive.getCurrentPose(), goal);

          DifferentialDriveWheelSpeeds wheelSpeeds = m_drive.wheelSpeedsFromChassisSpeeds(targetSpeeds);
          leftSpeed = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
          rightSpeed = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        } else {
          m_state++;
        }

        break;

      case 5: // go to the next trajectory
        if (m_currentTraj++ < m_trajectory.length) {
          m_state = 0;
        } else {
          m_state++;
        }
        break;

      default:
        break;
    }

    SmartDashboard.putNumber("AFT State", m_state);
    SmartDashboard.putNumber("AFT Left Speed", leftSpeed);
    SmartDashboard.putNumber("AFT Right Speed", rightSpeed);

    m_drive.basicDriveLimited(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.basicDriveLimited(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state > 5;
  }
}
