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
       
    public Trajectory barrelRun;
    public Trajectory slalom;
    public Trajectory bounce;
    public Trajectory test1;
    public Trajectory test2;
    public Trajectory testLoop;

    private TrajectoryConfig m_config;

    public GameChangerTrajectories(TrajectoryConfig config) {
        m_config = config;

        barrelRun = generateBarrelRunTrajectory();
        slalom = generateSlalomTrajectory();
        bounce = generateBounceTrajectory();

        test1 = generateTestTrajectory();
        test2 = generateTest2Trajectory();
        testLoop = generateTestLoopTrajectory();
    }

    private Trajectory generateBounceTrajectory() {
        // TODO
        return null;
    }

    private Trajectory generateSlalomTrajectory() {
        // begining pose, at the end of the start zone, next to the right side
        Pose2d start = new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(0.0), new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        // drive by D4
        waypoints.add(new Translation2d(Units.feetToMeters(9.5), Units.feetToMeters(5.0)));
        // mid point on left side, near D6
        waypoints.add(new Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(6.5)));
        // drive by D8
        waypoints.add(new Translation2d(Units.feetToMeters(20.5), Units.feetToMeters(5.0)));
        // loop around D10
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(0.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(29.5), Units.feetToMeters(5.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(10.0)));
        // drive by D8 again
        waypoints.add(new Translation2d(Units.feetToMeters(20.5), Units.feetToMeters(5.0)));
        // mid point or right side, near D6
        waypoints.add(new Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(3.5)));
        // drive by D4
        waypoints.add(new Translation2d(Units.feetToMeters(9.5), Units.feetToMeters(5.0)));
        // race to the finish! Near B2
        waypoints.add(new Translation2d(Units.feetToMeters(5.0), Units.feetToMeters(9.5)));

        // ending pose, well past finish line, all the way into the finish zone
        Pose2d end = new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(9.0),  Rotation2d.fromDegrees(180));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
    }

    private Trajectory generateBarrelRunTrajectory() {
        // TODO
        return null;
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

    private Trajectory generateTest2Trajectory() {
        // begining and ending poses
        Pose2d start = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
        Pose2d end = new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(0.0),  new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(1.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(13.0), Units.feetToMeters(-1.0)));
        
        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
    }

    private Trajectory generateTestLoopTrajectory() {
        // begining and ending poses
        Pose2d start = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
        Pose2d end = new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(0.0),  new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-3.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(11.0), Units.feetToMeters(0.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(3.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(4.0), Units.feetToMeters(0.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-3.5)));
        
        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
    }


}

