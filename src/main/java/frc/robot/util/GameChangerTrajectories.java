// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class GameChangerTrajectories {
       
    public Trajectory barrelRun;
    public Trajectory barrelRun2;
    public Trajectory slalom;
    public Trajectory bounce1, bounce2, bounce3, bounce4;
    public Trajectory test1;
    public Trajectory test2;
    public Trajectory testLoop;

    private TrajectoryConfig m_config;

    public GameChangerTrajectories(TrajectoryConfig config) {
        m_config = config;

        barrelRun = generateBarrelRunTrajectory();
        barrelRun2 = generateBarrelRun2Trajectory();
        slalom = generateSlalomTrajectory();
        bounce1 = generateBounce1Trajectory();
        bounce2 = generateBounce2Trajectory();
        bounce3 = generateBounce3Trajectory();
        bounce4 = generateBounce4Trajectory();

        test1 = generateTestTrajectory();
        test2 = generateTest2Trajectory();
        testLoop = generateTestLoopTrajectory();
    }

    private Trajectory generateBounce1Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, on the center line, up against the start line
        waypoints.add(new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(7.5), new Rotation2d()));
        // first star
        waypoints.add(new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));

        // generate trajectory and time how long it takes
        Timer timer = new Timer();
        timer.reset();
        timer.start();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        SmartDashboard.putNumber("TrajGen:Bounce1", timer.get());
        
        return trajectory;
    }

    private Trajectory generateBounce2Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // start at first star
        waypoints.add(new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));
        // bounce
        waypoints.add(new Pose2d(Units.feetToMeters(8.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(135.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(12.5), Units.feetToMeters(2.5), Rotation2d.fromDegrees(180.0)));
        // end at second star
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90.0)));
        
        // generate trajectory
        m_config.setReversed(true);
        Trajectory result = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        m_config.setReversed(false);
        
        return result;
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
        // start at first star
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(12.5), Rotation2d.fromDegrees(90.0)));
        // to the finish line
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(10), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180.0)));
        
        // generate trajectory
        m_config.setReversed(true);
        Trajectory result = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        m_config.setReversed(false);
        
        return result;
    }

    private Trajectory generateSlalomTrajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, at the end of the start zone, next to the right side
        waypoints.add(new Pose2d(Units.feetToMeters(5.0 - Constants.WHEEL_BASE_WIDTH / 2.0), Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0), new Rotation2d()));
        // first slalom
        waypoints.add(new Pose2d(Units.feetToMeters(7.7), Units.feetToMeters(5.0), Rotation2d.fromDegrees(75.0)));
        // midway left
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(8.0), Rotation2d.fromDegrees(0.0)));
        // second slalom
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(5.0), Rotation2d.fromDegrees(-80.0)));
        // loop
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0), new Rotation2d()));
        waypoints.add(new Pose2d(Units.feetToMeters(30.0 - Constants.WHEEL_BASE_WIDTH / 2.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(7.0), Rotation2d.fromDegrees(180.0)));
        // second slalom
        waypoints.add(new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(-100.0)));
        // midway right
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(Constants.WHEEL_BASE_WIDTH / 2.0), Rotation2d.fromDegrees(180.0)));
        // first slalom
        waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(4.0), Rotation2d.fromDegrees(100.0)));
        // race to the finish!
        waypoints.add(new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(7.7),  Rotation2d.fromDegrees(180.0)));

        // generate trajectory and time how long it takes
        Timer timer = new Timer();
        timer.reset();
        timer.start();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, m_config);
        SmartDashboard.putNumber("TrajGen:Slalom", timer.get());
        
        return trajectory;
    }

    private Trajectory generateBarrelRunTrajectory() {
        // begining pose, on the center line, up against the start line
        Pose2d start = new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(7.5), new Rotation2d());

        // set up waypoints for path
        var waypoints = new ArrayList<Translation2d>();
        // around the first barrel
        waypoints.add(new Translation2d(Units.feetToMeters(13.5), Units.feetToMeters(7.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(16.0), Units.feetToMeters(4.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(12.0), Units.feetToMeters(3.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(7.5)));
        // around the second barrel
        waypoints.add(new Translation2d(Units.feetToMeters(21.0), Units.feetToMeters(7.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(10.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(18.5), Units.feetToMeters(12.8)));
        //waypoints.add(new Translation2d(Units.feetToMeters(18.5), Units.feetToMeters(10.0)));
        // around the third barrel
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(2.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(29.0), Units.feetToMeters(5.0)));
        waypoints.add(new Translation2d(Units.feetToMeters(25.0), Units.feetToMeters(7.5)));
        // race to the finish!
        waypoints.add(new Translation2d(Units.feetToMeters(15.0), Units.feetToMeters(7.5)));
        waypoints.add(new Translation2d(Units.feetToMeters(5.0), Units.feetToMeters(7.8)));

        // ending pose, well past finish line, all the way into the finish zone
        Pose2d end = new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(7.8),  Rotation2d.fromDegrees(179));

        // generate trajectory and time how long it takes
        Timer timer = new Timer();
        timer.reset();
        timer.start();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, m_config);
        SmartDashboard.putNumber("TrajGen:BarrelRun", timer.get());
        
        return trajectory;
    }

    private Trajectory generateBarrelRun2Trajectory() {
        // set up waypoints for path
        var waypoints = new ArrayList<Pose2d>();
        // begining pose, on the center line, up against the start line
        waypoints.add(new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(7.5), new Rotation2d()));
        // around the first barrel
        waypoints.add(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(3.5), Rotation2d.fromDegrees(90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(6.5), Rotation2d.fromDegrees(-90.0)));
        // around the second barrel
        waypoints.add(new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(11.5), Rotation2d.fromDegrees(-90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(17.5), Units.feetToMeters(8.5), Rotation2d.fromDegrees(90.0)));
        // around the third barrel
        waypoints.add(new Pose2d(Units.feetToMeters(26.5), Units.feetToMeters(2.5), new Rotation2d()));
        waypoints.add(new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(6.5), Rotation2d.fromDegrees(-90.0)));
        waypoints.add(new Pose2d(Units.feetToMeters(20.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180.0)));
        // race to the finish!
        waypoints.add(new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(7.5),  Rotation2d.fromDegrees(180.0)));

        // generate trajectory
        return TrajectoryGenerator.generateTrajectory(waypoints, m_config);
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

