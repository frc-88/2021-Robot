// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class GameChangerTrajectories {
       
    public Trajectory testTrajectory;

    private TrajectoryConfig m_config;

    public GameChangerTrajectories(TrajectoryConfig config) {
        m_config = config;

        testTrajectory = generateTestTrajectory();
    }

    private Trajectory generateTestTrajectory() {
        // begining and ending poses
        Pose2d start = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
        Pose2d end = new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(0.0),  new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(3.0)));
        
        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
    }
}

