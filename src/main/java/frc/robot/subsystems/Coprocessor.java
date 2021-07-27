package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coprocessor extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private Drive m_drive;

    private NetworkTable table;
    private NetworkTable hostTable;
    private NetworkTable clientTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;
    private NetworkTable odomTable;

    private final double clientConnectedTimeout = 1.0;  // seconds
    private final String rootTableName = "coprocessor";

    public Coprocessor(Drive drive)
    {
        m_drive = drive;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);
        clientTable = table.getSubTable("client");  // Data and commands from the Jetson
        hostTable = table.getSubTable("host");  // Data and commands from the RoboRIO
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = hostTable.getEntry("timestamp");

        driverStationTable = hostTable.getSubTable("DriverStation");
        odomTable = hostTable.getSubTable("odom");

        hostTimestamp.setNumber(getTime());
    }

    private long getTime()
    {
        return RobotController.getFPGATime();
    }

    public boolean isConnected()
    {
        return clientTimestamp.exists() && hostTimestamp.exists() && (getTime() - clientTimestamp.getLastChange() < clientConnectedTimeout);
    }

    @Override
    public void periodic()
    {
        hostTimestamp.setNumber(getTime());
        if (!isConnected()) {
            return;
        }
        // Driver Station
        driverStationTable.getEntry("isFMSAttached").setBoolean(DriverStation.getInstance().isFMSAttached());
        driverStationTable.getEntry("getMatchTime").setDouble(DriverStation.getInstance().getMatchTime());

        // Robot odometry pose
        Pose2d pose = m_drive.getCurrentPose();
        odomTable.getEntry("xPosition").setDouble(Units.metersToFeet(pose.getX()));  // feet
        odomTable.getEntry("yPosition").setDouble(Units.metersToFeet(pose.getY()));  // feet
        odomTable.getEntry("theta").setDouble(pose.getRotation().getDegrees());  // degrees

        odomTable.getEntry("xVelocity").setDouble(m_drive.getStraightSpeed());  // feet per second
        odomTable.getEntry("thetaVelocity").setDouble(m_drive.getTurnSpeed());  // degrees per second
        
    }
}
