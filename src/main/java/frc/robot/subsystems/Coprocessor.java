package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.listeners.SetOdomListener;
import frc.robot.subsystems.tunnel.TunnelDataRelayThread;
import frc.robot.subsystems.tunnel.TunnelServer;

public class Coprocessor extends SubsystemBase {
    /**
     * A class to encapsulate shared data interactions and action notifications with the NVidia Jetson.
     */
    
    private Drive m_drive;

    private NetworkTable table;
    private NetworkTable clientTable;
    private NetworkTable setOdomTable;
    private NetworkTableEntry clientTimestamp;
    private NetworkTableEntry hostTimestamp;

    private NetworkTable driverStationTable;

    private SetOdomListener setOdomListener;
    
    private final long clientConnectedTimeout = 1_000_000;  // micro seconds
    private final String rootTableName = "coprocessor";

    private TunnelServer tunnel;
    private TunnelDataRelayThread data_relay_thread;

    private boolean isCommandSet = false;

    public Coprocessor(Drive drive)
    {
        m_drive = drive;

        table = NetworkTableInstance.getDefault().getTable(rootTableName);
        clientTable = table.getSubTable("ROS");  // Data from the Jetson
        setOdomTable = clientTable.getSubTable("setOdom");
        clientTimestamp = clientTable.getEntry("timestamp");
        hostTimestamp = table.getEntry("timestamp");

        driverStationTable = table.getSubTable("DriverStation");

        hostTimestamp.setNumber(getTime());

        setOdomListener = new SetOdomListener(drive);
        setOdomListener.setTable(setOdomTable);
        setOdomTable.addEntryListener("timestamp", setOdomListener, EntryListenerFlags.kUpdate);

        tunnel = new TunnelServer(m_drive, 3000);
        tunnel.start();

        data_relay_thread = new TunnelDataRelayThread(tunnel);
        data_relay_thread.start();
    }

    private long getTime() {
        return RobotController.getFPGATime();
    }

    public boolean isConnected() {
        return clientTimestamp.exists() && hostTimestamp.exists() && (getTime() - clientTimestamp.getLastChange() < clientConnectedTimeout);
    }

    public boolean getIsCommandSet() {
        return isCommandSet;
    }

    @Override
    public void periodic()
    {
        hostTimestamp.setNumber(getTime());

        // Driver Station
        driverStationTable.getEntry("isFMSAttached").setBoolean(DriverStation.getInstance().isFMSAttached());
        driverStationTable.getEntry("isAutonomous").setBoolean(DriverStation.getInstance().isAutonomous());
        driverStationTable.getEntry("getMatchTime").setDouble(DriverStation.getInstance().getMatchTime());

        isCommandSet = tunnel.setCommandIfActive();
    }
}
