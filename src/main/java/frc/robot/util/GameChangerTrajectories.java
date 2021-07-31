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
/*     */ {
/*     */   public Trajectory barrelRun;
/*     */   public Trajectory barrelRun2;
/*     */   public Trajectory slalom;
/*     */   public Trajectory bounce1;
/*     */   public Trajectory bounce2;
/*     */   public Trajectory bounce3;
/*     */   public Trajectory bounce4;
/*     */   public Trajectory test;
/*     */   public Trajectory auto3ball1;
/*     */   public Trajectory autotrench1;
/*     */   public Trajectory autotrench2;
/*     */   public Trajectory autotrench3;
/*     */   public Trajectory autostealyoballs1;
/*     */   public Trajectory autostealyoballs2;
/*     */   private static final double TRAJ_CONFIG_MAX_VEL = 16.0D;
/*     */   private static final double TRAJ_CONFIG_MAX_ACCEL = 8.0D;
/*     */   private static final double TRAJ_CONFIG_MAX_CENTRIP_ACCEL = 4.5D;
/*     */   private final TrajectoryConfig m_config;
/*     */   
/*     */   public GameChangerTrajectories() {
/*  40 */     DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.feetToMeters(2.109375D));
/*     */     
/*  42 */     this.m_config = new TrajectoryConfig(Units.feetToMeters(16.0D), Units.feetToMeters(8.0D));
/*  43 */     this.m_config.setKinematics(kinematics);
/*  44 */     this.m_config.setStartVelocity(0.0D);
/*  45 */     this.m_config.setEndVelocity(0.0D);
/*     */     
/*  47 */     this.m_config.addConstraint((TrajectoryConstraint)new DifferentialDriveKinematicsConstraint(kinematics, Units.feetToMeters(16.0D)));
/*  48 */     this.m_config.addConstraint((TrajectoryConstraint)new CentripetalAccelerationConstraint(4.5D));
/*     */     
/*  50 */     this.barrelRun = generateBarrelRunTrajectory();
/*  51 */     this.barrelRun2 = generateBarrelRun2Trajectory();
/*  52 */     this.slalom = generateSlalomTrajectory();
/*  53 */     this.bounce1 = generateBounce1Trajectory();
/*  54 */     this.bounce2 = generateBounce2Trajectory();
/*  55 */     this.bounce3 = generateBounce3Trajectory();
/*  56 */     this.bounce4 = generateBounce4Trajectory();
/*  57 */     this.auto3ball1 = generateauto3ball1Trajectory();
/*  58 */     this.autotrench1 = generateautotrench1Trajectory();
/*  59 */     this.autotrench2 = generateautotrench2Trajectory();
/*  60 */     this.autotrench3 = generateautotrench3Trajectory();
/*  61 */     this.autostealyoballs1 = generateautostealyoballs1Trajectory();
/*  62 */     this.autostealyoballs2 = generateautostealyoballs2Trajectory();
/*     */     
/*  64 */     this.test = generateTestTrajectory();
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBarrelRunTrajectory() {
/*  70 */     Pose2d start = new Pose2d(Units.feetToMeters(5.0D), Units.feetToMeters(7.5D), new Rotation2d());
/*     */ 
/*     */     
/*  73 */     ArrayList<Translation2d> waypoints = new ArrayList<>();
/*     */     
/*  75 */     waypoints.add(new Translation2d(Units.feetToMeters(13.5D), Units.feetToMeters(7.5D)));
/*  76 */     waypoints.add(new Translation2d(Units.feetToMeters(16.1D), Units.feetToMeters(4.0D)));
/*  77 */     waypoints.add(new Translation2d(Units.feetToMeters(12.0D), Units.feetToMeters(3.0D)));
/*  78 */     waypoints.add(new Translation2d(Units.feetToMeters(12.5D), Units.feetToMeters(7.5D)));
/*     */     
/*  80 */     waypoints.add(new Translation2d(Units.feetToMeters(21.0D), Units.feetToMeters(7.5D)));
/*  81 */     waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(12.5D)));
/*  82 */     waypoints.add(new Translation2d(Units.feetToMeters(17.8D), Units.feetToMeters(12.8D)));
/*     */     
/*  84 */     waypoints.add(new Translation2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D)));
/*  85 */     waypoints.add(new Translation2d(Units.feetToMeters(29.1D), Units.feetToMeters(5.0D)));
/*  86 */     waypoints.add(new Translation2d(Units.feetToMeters(26.0D), Units.feetToMeters(7.8D)));
/*     */     
/*  88 */     waypoints.add(new Translation2d(Units.feetToMeters(15.0D), Units.feetToMeters(8.1D)));
/*     */ 
/*     */     
/*  91 */     Pose2d end = new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(8.1D), Rotation2d.fromDegrees(180.0D));
/*     */ 
/*     */     
/*  94 */     return TrajectoryGenerator.generateTrajectory(start, waypoints, end, this.m_config);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBarrelRun2Trajectory() {
/* 100 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 102 */     waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), Units.feetToMeters(7.5D), new Rotation2d()));
/*     */     
/* 104 */     waypoints.add(new Pose2d(Units.feetToMeters(15.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-90.0D)));
/* 105 */     waypoints.add(new Pose2d(Units.feetToMeters(12.5D), Units.feetToMeters(2.0D), Rotation2d.fromDegrees(180.0D)));
/* 106 */     waypoints.add(new Pose2d(Units.feetToMeters(9.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
/*     */     
/* 108 */     waypoints.add(new Pose2d(Units.feetToMeters(23.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(90.0D)));
/* 109 */     waypoints.add(new Pose2d(Units.feetToMeters(20.0D), Units.feetToMeters(13.0D), Rotation2d.fromDegrees(180.0D)));
/* 110 */     waypoints.add(new Pose2d(Units.feetToMeters(17.0D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(-90.0D)));
/*     */     
/* 112 */     waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(2.0D), new Rotation2d()));
/* 113 */     waypoints.add(new Pose2d(Units.feetToMeters(28.0D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(90.0D)));
/* 114 */     waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(8.0D), Rotation2d.fromDegrees(180.0D)));
/*     */     
/* 116 */     waypoints.add(new Pose2d(Units.feetToMeters(0.5D), Units.feetToMeters(7.5D), Rotation2d.fromDegrees(180.0D)));
/*     */ 
/*     */     
/* 119 */     return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateSlalomTrajectory() {
/* 125 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 127 */     waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), 
/* 128 */           Units.feetToMeters(1.0546875D), new Rotation2d()));
/*     */     
/* 130 */     waypoints.add(new Pose2d(Units.feetToMeters(7.7D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(75.0D)));
/*     */     
/* 132 */     waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(8.0D), Rotation2d.fromDegrees(0.0D)));
/*     */     
/* 134 */     waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-80.0D)));
/*     */     
/* 136 */     waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(1.0546875D), new Rotation2d()));
/*     */     
/* 138 */     waypoints.add(new Pose2d(Units.feetToMeters(28.9453125D), Units.feetToMeters(5.0D), 
/* 139 */           Rotation2d.fromDegrees(90.0D)));
/* 140 */     waypoints.add(new Pose2d(Units.feetToMeters(27.5D), Units.feetToMeters(7.0D), Rotation2d.fromDegrees(180.0D)));
/*     */     
/* 142 */     waypoints.add(new Pose2d(Units.feetToMeters(25.0D), Units.feetToMeters(5.0D), Rotation2d.fromDegrees(-100.0D)));
/*     */     
/* 144 */     waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(1.0546875D), 
/* 145 */           Rotation2d.fromDegrees(180.0D)));
/*     */     
/* 147 */     waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(4.0D), Rotation2d.fromDegrees(100.0D)));
/*     */     
/* 149 */     waypoints.add(new Pose2d(Units.feetToMeters(0.5D), Units.feetToMeters(7.7D), Rotation2d.fromDegrees(180.0D)));
/*     */ 
/*     */     
/* 152 */     return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBounce1Trajectory() {
/* 158 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 160 */     waypoints.add(new Pose2d(Units.feetToMeters(3.9453125D), Units.feetToMeters(7.5D), new Rotation2d()));
/*     */     
/* 162 */     waypoints.add(new Pose2d(Units.feetToMeters(7.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));
/*     */ 
/*     */     
/* 165 */     return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBounce2Trajectory() {
/* 171 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 173 */     waypoints.add(new Pose2d(Units.feetToMeters(7.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));
/*     */     
/* 175 */     waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(7.5D), Rotation2d.fromDegrees(120.0D)));
/* 176 */     waypoints.add(new Pose2d(Units.feetToMeters(12.5D), Units.feetToMeters(2.5D), Rotation2d.fromDegrees(180.0D)));
/*     */     
/* 178 */     waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(7.0D), Rotation2d.fromDegrees(-95.0D)));
/* 179 */     waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));
/*     */ 
/*     */     
/* 182 */     this.m_config.setReversed(true);
/* 183 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 184 */     this.m_config.setReversed(false);
/*     */     
/* 186 */     return trajectory;
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBounce3Trajectory() {
/* 192 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 194 */     waypoints.add(new Pose2d(Units.feetToMeters(15.0D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));
/*     */     
/* 196 */     waypoints.add(new Pose2d(Units.feetToMeters(18.5D), Units.feetToMeters(2.5D), Rotation2d.fromDegrees(0.0D)));
/*     */     
/* 198 */     waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(90.0D)));
/*     */ 
/*     */     
/* 201 */     return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateBounce4Trajectory() {
/* 207 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 209 */     waypoints.add(new Pose2d(Units.feetToMeters(27.5D), Units.feetToMeters(7.5D), new Rotation2d()));
/* 210 */     waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(10.0D), Rotation2d.fromDegrees(-90.0D)));
/*     */     
/* 212 */     waypoints.add(new Pose2d(Units.feetToMeters(22.5D), Units.feetToMeters(12.5D), Rotation2d.fromDegrees(-90.0D)));
/*     */ 
/*     */     
/* 215 */     this.m_config.setReversed(true);
/* 216 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 217 */     this.m_config.setReversed(false);
/*     */     
/* 219 */     return trajectory;
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   private Trajectory generateTestTrajectory() {
/* 225 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 227 */     waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */     
/* 229 */     waypoints.add(new Pose2d(Units.feetToMeters(10.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */ 
/*     */     
/* 232 */     return TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateauto3ball1Trajectory() {
/* 237 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 239 */     waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */     
/* 241 */     waypoints.add(new Pose2d(Units.feetToMeters(-2.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */ 
/*     */     
/* 244 */     this.m_config.setReversed(true);
/* 245 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 246 */     this.m_config.setReversed(false);
/*     */     
/* 248 */     return trajectory;
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateautotrench1Trajectory() {
/* 253 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 255 */     waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */     
/* 257 */     waypoints.add(new Pose2d(Units.feetToMeters(-7.0D), Units.feetToMeters(0.0D), new Rotation2d()));
/*     */ 
/*     */     
/* 260 */     this.m_config.setReversed(true);
/* 261 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 262 */     this.m_config.setReversed(false);
/*     */     
/* 264 */     return trajectory;
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateautotrench2Trajectory() {
/* 269 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 271 */     waypoints.add(new Pose2d(Units.feetToMeters(-7.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
/* 272 */     waypoints.add(new Pose2d(Units.feetToMeters(-10.0D), Units.feetToMeters(0.875D), new Rotation2d(6.0D)));
/*     */     
/* 274 */     waypoints.add(new Pose2d(Units.feetToMeters(-14.0D), Units.feetToMeters(1.0D), new Rotation2d(8.0D)));
/*     */ 
/*     */     
/* 277 */     this.m_config.setReversed(true);
/* 278 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 279 */     this.m_config.setReversed(false);
/*     */     
/* 281 */     return trajectory;
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateautotrench3Trajectory() {
/* 286 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 288 */     waypoints.add(new Pose2d(Units.feetToMeters(-14.0D), Units.feetToMeters(0.0D), new Rotation2d(-6.0D)));
/*     */     
/* 290 */     waypoints.add(new Pose2d(Units.feetToMeters(-5.0D), Units.feetToMeters(-3.0D), new Rotation2d(0.0D)));
/*     */ 
/*     */ 
/*     */     
/* 294 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */ 
/*     */     
/* 297 */     return trajectory;
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateautostealyoballs1Trajectory() {
/* 302 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 304 */     waypoints.add(new Pose2d(Units.feetToMeters(0.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
/*     */     
/* 306 */     waypoints.add(new Pose2d(Units.feetToMeters(-8.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
/*     */ 
/*     */     
/* 309 */     this.m_config.setReversed(true);
/* 310 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/* 311 */     this.m_config.setReversed(false);
/*     */     
/* 313 */     return trajectory;
/*     */   }
/*     */ 
/*     */   
/*     */   private Trajectory generateautostealyoballs2Trajectory() {
/* 318 */     ArrayList<Pose2d> waypoints = new ArrayList<>();
/*     */     
/* 320 */     waypoints.add(new Pose2d(Units.feetToMeters(-8.0D), Units.feetToMeters(0.0D), new Rotation2d(0.0D)));
/*     */     
/* 322 */     waypoints.add(new Pose2d(Units.feetToMeters(-2.0D), Units.feetToMeters(-3.0D), new Rotation2d(-90.0D)));
/* 323 */     waypoints.add(new Pose2d(Units.feetToMeters(-3.0D), Units.feetToMeters(-10.0D), new Rotation2d(-5.0D)));
/*     */ 
/*     */     
/* 326 */     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, this.m_config);
/*     */ 
/*     */     
/* 329 */     return trajectory;
/*     */   }
/*     */ }