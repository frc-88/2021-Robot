package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.listeners.CommandListener;
import frc.robot.subsystems.listeners.PingListener;
import frc.robot.subsystems.listeners.SetOdomListener;

public class Coprocessor extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private Drive m_drive;

    private NetworkTable table;
    private NetworkTable clientTable;
    private NetworkTable commandsTable;
    private NetworkTable setOdomTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;
    private NetworkTable odomTable;

    private PingListener ntPingListener;
    private CommandListener ntCommandListener;
    private SetOdomListener setOdomListener;
    
    private final long clientConnectedTimeout = 1_000_000;  // micro seconds
    private final String rootTableName = "coprocessor";

    public Coprocessor(Drive drive)
    {
        m_drive = drive;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);
        clientTable = table.getSubTable("ROS");  // Data from the Jetson
        commandsTable = table.getSubTable("commands");  // Commands from the Jetson
        setOdomTable = clientTable.getSubTable("setOdom");
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = table.getEntry("timestamp");

        driverStationTable = table.getSubTable("DriverStation");
        odomTable = table.getSubTable("odometryState");

        hostTimestamp.setNumber(getTime());

        ntPingListener = new PingListener();
        ntPingListener.setTable(table);
        clientTable.addEntryListener("ping", ntPingListener, EntryListenerFlags.kUpdate);

        ntCommandListener = new CommandListener();
        ntCommandListener.setTable(commandsTable);
        commandsTable.addEntryListener("timestamp", ntCommandListener, EntryListenerFlags.kUpdate);

        setOdomListener = new SetOdomListener(drive);
        setOdomListener.setTable(setOdomTable);
        setOdomTable.addEntryListener("timestamp", setOdomListener, EntryListenerFlags.kUpdate);
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
        // if (!isConnected()) {
        //     return;
        // }
        // Driver Station
        driverStationTable.getEntry("isFMSAttached").setBoolean(DriverStation.getInstance().isFMSAttached());
        driverStationTable.getEntry("isAutonomous").setBoolean(DriverStation.getInstance().isAutonomous());
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
