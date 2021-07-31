// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class GameChangerTrajectories
{
  public Trajectory barrelRun;
  public Trajectory barrelRun2;
  public Trajectory slalom;
  public Trajectory bounce1;
  public Trajectory bounce2;
  public Trajectory bounce3;
  public Trajectory bounce4;
  public Trajectory test;
  public Trajectory auto3ball1;
  public Trajectory autotrench1;
  public Trajectory autotrench2;
  public Trajectory autotrench3;
  public Trajectory autostealyoballs1;
  public Trajectory autostealyoballs2;
  private static final double TRAJ_CONFIG_MAX_VEL = 16.0D;
  private static final double TRAJ_CONFIG_MAX_ACCEL = 8.0D;
  private static final double TRAJ_CONFIG_MAX_CENTRIP_ACCEL = 4.5D;
  private final TrajectoryConfig m_config;
  public GameChangerTrajectories() {
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.feetToMeters(2.109375D));
    this.m_config = new TrajectoryConfig(Units.feetToMeters(16.0D), Units.feetToMeters(8.0D));
    this.m_config.setKinematics(kinematics);
    this.m_config.setStartVelocity(0.0D);
    this.m_config.setEndVelocity(0.0D);
    this.m_config.addConstraint((TrajectoryConstraint)new DifferentialDriveKinematicsConstraint(kinematics, Units.feetToMeters(16.0D)));
    this.m_config.addConstraint((TrajectoryConstraint)new CentripetalAccelerationConstraint(4.5D));
    this.barrelRun = generateBarrelRunTrajectory();
    this.barrelRun2 = generateBarrelRun2Trajectory();
    this.slalom = generateSlalomTrajectory();
    this.bounce1 = generateBounce1Trajectory();
    this.bounce2 = generateBounce2Trajectory();
    this.bounce3 = generateBounce3Trajectory();
    this.bounce4 = generateBounce4Trajectory();
    this.auto3ball1 = generateauto3ball1Trajectory();
    this.autotrench1 = generateautotrench1Trajectory();
    this.autotrench2 = generateautotrench2Trajectory();
    this.autotrench3 = generateautotrench3Trajectory();
    this.autostealyoballs1 = generateautostealyoballs1Trajectory();
    this.autostealyoballs2 = generateautostealyoballs2Trajectory();
    this.test = generateTestTrajectory();
  }


  private Trajectory generateBarrelRunTrajectory() {
    Pose2d start = new Pose2d(Units.feetToMeters(5.0D), Units.feetToMeters(7.5D), new Rotation2d());

    ArrayList<Translation2d> waypoints = new ArrayList<>();
    waypoints.add(new Translation2d(Units.feetToMeters(13.5D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(16.1D), Units.feetToMeters(4.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(12.0D), Units.feetToMeters(3.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(12.5D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(21.0D), Units.feetToMeters(7.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(12.5D)));
    waypoints.add(new Translation2d(Units.feetToMeters(17.8D), Units.feetToMeters(12.8D)));
    waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(29.1D), Units.feetToMeters(5.0D)));
    waypoints.add(new Translation2d(Units.feetToMeters(26.0D), Units.feetToMeters(7.8D)));
    waypoints.add(new Translation2d(Units.feetToMeters(15.0D), Units.feetToMeters(8.1D)));

    Pose2d end = new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(8.1D), Rotation2d.fromDegrees(180.0D));

    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, this.m_config);
  }


  private Trajectory generateBarrelRun2Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), Units.feetToMeters(7.5D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(15.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.5D), Units.feetToMeters(2.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(9.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(23.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(20.0D), Units.feetToMeters(13.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(17.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(28.0D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(8.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(0.5D), Units.feetToMeters(7.5D), Rotation2d.fromDegrees(180.0D)));

    return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
  }


  private Trajectory generateSlalomTrajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), 
          Units.feetToMeters(1.0546875D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(7.7D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(75.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(8.0D), Rotation2d.fromDegrees(0.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-80.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(1.0546875D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(28.9453125D), Units.feetToMeters(5.0D), 
          Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(27.5D), Units.feetToMeters(7.0D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-100.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(1.0546875D), 
          Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(4.0D), Rotation2d.fromDegrees(100.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(0.5D), Units.feetToMeters(7.7D), Rotation2d.fromDegrees(180.0D)));

    return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
  }


  private Trajectory generateBounce1Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), Units.feetToMeters(7.5D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(7.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));

    return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
  }


  private Trajectory generateBounce2Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(7.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(7.5D), Rotation2d.fromDegrees(120.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(12.5D), Units.feetToMeters(2.5D), Rotation2d.fromDegrees(180.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(7.0D), Rotation2d.fromDegrees(-95.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }


  private Trajectory generateBounce3Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(18.5D), Units.feetToMeters(2.5D), Rotation2d.fromDegrees(0.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));

    return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
  }


  private Trajectory generateBounce4Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(27.5D), Units.feetToMeters(7.5D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }


  private Trajectory generateTestTrajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(0.0D), new Rotation2d()));

    return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
  }

  private Trajectory generateauto3ball1Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(-2.0D), Units.feetToMeters(0.0D), new Rotation2d()));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }

  private Trajectory generateautotrench1Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
    waypoints.add(new Pose2d(Units.feetToMeters(-7.0D), Units.feetToMeters(0.0D), new Rotation2d()));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }

  private Trajectory generateautotrench2Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(-7.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-10.0D), Units.feetToMeters(0.875D), new Rotation2d(6.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-14.0D), Units.feetToMeters(1.0D), new Rotation2d(8.0D)));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }

  private Trajectory generateautotrench3Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(-14.0D), Units.feetToMeters(0.0D), new Rotation2d(-6.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-5.0D), Units.feetToMeters(-3.0D), new Rotation2d(0.0D)));


    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);

    return trajectory;
  }

  private Trajectory generateautostealyoballs1Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-8.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));

    this.m_config.setReversed(true);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
    this.m_config.setReversed(false);
    return trajectory;
  }

  private Trajectory generateautostealyoballs2Trajectory() {
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(Units.feetToMeters(-8.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-2.0D), Units.feetToMeters(-3.0D), new Rotation2d(-90.0D)));
    waypoints.add(new Pose2d(Units.feetToMeters(-3.0D), Units.feetToMeters(-10.0D), new Rotation2d(-5.0D)));

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);

    return trajectory;
  }
}