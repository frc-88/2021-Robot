package frc.robot.subsystems.listeners;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.Drive;

/** Handles callbacks for when the command subtable is updated */
public class SetOdomListener implements TableEntryListener {
  private NetworkTable m_table;
  private Drive m_drive;

  public SetOdomListener(Drive drive) {
    m_drive = drive;
  }

  /**
   * Take a NetworkTable instance from parent and setup keys to listen to
   *
   * @param table A table from DataManager
   */
  public void setTable(NetworkTable table) {
    m_table = table;
    m_table.getEntry("xPosition").setDouble(0.0);
    m_table.getEntry("yPosition").setDouble(0.0);
    m_table.getEntry("theta").setDouble(0.0);
  }

  /** Callback for when data is pushed to the designated subtable */
  @Override
  public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    double xPosition = m_table.getEntry("xPosition").getDouble(0.0);
    double yPosition = m_table.getEntry("yPosition").getDouble(0.0);
    double theta = m_table.getEntry("theta").getDouble(0.0);

    Rotation2d rotation = Rotation2d.fromDegrees(theta);
    m_drive.resetOdometry(new Pose2d(xPosition, yPosition, rotation), rotation);
  }
}
