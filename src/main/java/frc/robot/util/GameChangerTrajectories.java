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
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class GameChangerTrajectories {
    public Trajectory barrelRun;
    public Trajectory barrelRun2;
    public Trajectory slalom;
    public Trajectory bounce1, bounce2, bounce3, bounce4;
    public Trajectory test;

    private static final double TRAJ_CONFIG_MAX_VEL = 16.0;
    private static final double TRAJ_CONFIG_MAX_ACCEL = 8.0;
    private static final double TRAJ_CONFIG_MAX_CENTRIP_ACCEL = 4.5;
    private final TrajectoryConfig m_config;

    public GameChangerTrajectories() {
        // define constraints for trajectory generation
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
                Units.feetToMeters(Constants.WHEEL_BASE_WIDTH));

        m_config = new TrajectoryConfig(Units.feetToMeters(TRAJ_CONFIG_MAX_VEL), Units.feetToMeters(TRAJ_CONFIG_MAX_ACCEL));
        m_config.setKinematics(kinematics);
        m_config.setStartVelocity(0.0);
        m_config.setEndVelocity(0.0);

        m_config.addConstraint(new DifferentialDriveKinematicsConstraint(kinematics, Units.feetToMeters(TRAJ_CONFIG_MAX_VEL)));
        m_config.addConstraint(new CentripetalAccelerationConstraint(TRAJ_CONFIG_MAX_CENTRIP_ACCEL));

        barrelRun = generateBarrelRunTrajectory();
        barrelRun2 = generateBarrelRun2Trajectory();
        slalom = generateSlalomTrajectory();
        bounce1 = generateBounce1Trajectory();
        bounce2 = generateBounce2Trajectory();
        bounce3 = generateBounce3Trajectory();
        bounce4 = generateBounce4Trajectory();

        test = generateTestTrajectory();
    }


    private Trajectory generateBarrelRunTrajectory() {
        // begining pose, on the center line, up against the start line
        Pose2d start = new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(7.5), new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        // around the first barrel
        waypoints.add(new Translation2d(Units.feetToMeters(13.5), Units.feetToMeters(7.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(16.1), Units.feetToMeters(4.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(12.0), Units.feetToMeters(3.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(7.5)));
        // around the second barrel
        waypoints.add(new Translation2d(Units.feetToMeters(21.0), Units.feetToMeters(7.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(12.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(17.8), Units.feetToMeters(12.8)));
        // around the third barrel
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(2.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(29.1), Units.feetToMeters(5.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(26.0), Units.feetToMeters(7.8)));
        // race to the finish!
        waypoints.add(new Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(8.1)));

        // ending pose, well past finish line, all the way into the finish zone
        Pose2d end = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(8.1), Rotation2d.fromDegrees(180));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
    }


    private Trajectory generateBarrelRun2Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, on the center line, up against the start line
        waypoints.add(new Pose2d(Units.feetToMeters(5.0-(Constants.WHEEL_BASE_WIDTH/2)), Units.feetToMeters(7.5), new Rotation2d()));
        // around the first barrel
        waypoints.add(new Pose2d(Units.feetToMeters(15.5), Units.feetToMeters(5.0), Rotation2d.fromDegrees(-90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(12.5), Units.feetToMeters(2.0), Rotation2d.fromDegrees(180.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(9.5), Units.feetToMeters(5.0), Rotation2d.fromDegrees(90.0)));
        // around the second barrel
        waypoints.add(new Pose2d(Units.feetToMeters(23.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(20.0), Units.feetToMeters(13.0), Rotation2d.fromDegrees(180.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(17.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(-90.0)));
        // around the third barrel
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(2.0), new Rotation2d()));
        waypoints.add(new Pose2d(Units.feetToMeters(28.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(8.0), Rotation2d.fromDegrees(180.0)));
        // race to the finish!
        waypoints.add(new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180.0)));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
    }


    private Trajectory generateSlalomTrajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, at the end of the start zone, next to the right side
        waypoints.add(new Pose2d(Units.feetToMeters(5.0 - Constants.WHEEL_BASE_WIDTH / 2.0),
                Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0), new Rotation2d()));
        // first slalom
        waypoints.add(new Pose2d(Units.feetToMeters(7.7), Units.feetToMeters(5.0), Rotation2d.fromDegrees(75.0)));
        // midway left
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(8.0), Rotation2d.fromDegrees(0.0)));
        // second slalom
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(5.0), Rotation2d.fromDegrees(-80.0)));
        // loop
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0),
                new Rotation2d()));
        waypoints.add(new Pose2d(Units.feetToMeters(30.0 - Constants.WHEEL_BASE_WIDTH / 2.0), Units.feetToMeters(5.0),
                Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(7.0), Rotation2d.fromDegrees(180.0)));
        // second slalom
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(-100.0)));
        // midway right
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0),
                Rotation2d.fromDegrees(180.0)));
        // first slalom
        waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(4.0), Rotation2d.fromDegrees(100.0)));
        // race to the finish!
        waypoints.add(new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(7.7), Rotation2d.fromDegrees(180.0)));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
    }


    private Trajectory generateBounce1Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, on the center line, up against the start line
        waypoints.add(new Pose2d(Units.feetToMeters(5.0 - (Constants.WHEEL_BASE_WIDTH /2)), Units.feetToMeters(7.5), new Rotation2d()));
        // first star
        waypoints.add(new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
    }


    private Trajectory generateBounce2Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // start at first star
        waypoints.add(new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));
        // bounce
        waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(120.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(12.5), Units.feetToMeters(2.5), Rotation2d.fromDegrees(180.0)));
        // end at second star
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(7.0), Rotation2d.fromDegrees(-95.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90.0)));

        // generate trajectory
        m_config.setReversed(true);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        m_config.setReversed(false);

        return trajectory;
    }


    private Trajectory generateBounce3Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // start at second star
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90.0)));
        // bounce
        waypoints.add(new Pose2d(Units.feetToMeters(18.5), Units.feetToMeters(2.5), Rotation2d.fromDegrees(0.0)));
        // end at third star
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
    }


    private Trajectory generateBounce4Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // to the finish line
        waypoints.add(new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(7.5), new Rotation2d()));
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(10), Rotation2d.fromDegrees(-90.0)));
        // start at first star
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90.0)));

        // generate trajectory
        m_config.setReversed(true);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        m_config.setReversed(false);

        return trajectory;
    }


    private Trajectory generateTestTrajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, on the origin
        waypoints.add(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()));
        // ending pose, 10 feet forward
        waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(0.0), new Rotation2d()));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
    }
}
