package frc.robot.subsystems.listeners;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.RobotController;

/** Handles callbacks for when the command subtable is updated */
public class CommandListener implements TableEntryListener {
  private NetworkTable m_table;
  private long lastActiveTime = 0;
  private long activeTimeThreshold = 5_000_000;  // microseconds
  private double speedCommand = 0.0;
  private double turnCommand = 0.0;

  public CommandListener() {
    
  }

  public boolean isActive()
  {
    return (getTime() - lastActiveTime) < activeTimeThreshold;
  }
  
  private long getTime()
  {
    return RobotController.getFPGATime();
  }
  
  public double getSpeedCommand() {
    return speedCommand;
  }
  public double getTurnCommand() {
    return turnCommand;
  }

  /**
   * Take a NetworkTable instance from parent and setup keys to listen to
   *
   * @param table A table from DataManager
   */
  public void setTable(NetworkTable table) {
    m_table = table;
    m_table.getEntry("timestamp").setDouble(0.0);
    m_table.getEntry("translationDirection").setDouble(0.0);
    m_table.getEntry("translationSpeed").setDouble(0.0);
    m_table.getEntry("rotationVelocity").setDouble(0.0);
    m_table.getEntry("isFieldCentric").setBoolean(false);
  }

  /** Callback for when data is pushed to the designated subtable */
  @Override
  public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    lastActiveTime = getTime();
    // double translationDirection = m_table.getEntry("translationDirection").getDouble(0.0);
    speedCommand = m_table.getEntry("translationSpeed").getDouble(0.0);
    turnCommand = m_table.getEntry("rotationVelocity").getDouble(0.0);
    // boolean isFieldCentric = m_table.getEntry("isFieldCentric").getBoolean(false);
  }
}
