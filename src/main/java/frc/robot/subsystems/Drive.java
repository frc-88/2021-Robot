/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driveutil.DriveConfiguration;
import frc.robot.driveutil.TJDriveModule;
import frc.robot.util.GameChangerTrajectories;
import frc.robot.util.SyncPIDController;
import frc.robot.util.WrappingPIDController;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.transmission.CTREMagEncoder;
import frc.robot.util.transmission.Falcon500;
import frc.robot.util.transmission.ShiftingTransmission;

public class Drive extends SubsystemBase {

  private final Sensors m_sensors;

  private final TJDriveModule m_leftDrive, m_rightDrive;
  private final CANCoder m_leftEncoder, m_rightEncoder;
  private ShiftingTransmission m_leftTransmission, m_rightTransmission;
  private SyncPIDController m_leftVelPID, m_rightVelPID;
  private WrappingPIDController m_headingPID;
  private DriveConfiguration m_driveConfiguration;
  private DoubleSolenoid m_leftShifter, m_rightShifter;

  private double m_currentLimit = Constants.DRIVE_CURRENT_LIMIT;
  private double m_leftCommandedSpeed = 0;
  private double m_rightCommandedSpeed = 0;
  private double m_maxSpeed = Constants.MAX_SPEED_HIGH;

  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
  public GameChangerTrajectories trajectories;

  private PIDPreferenceConstants velPIDConstants;
  private PIDPreferenceConstants headingPIDConstants;
  private DoublePreferenceConstant downshiftSpeed;
  private DoublePreferenceConstant upshiftSpeed;
  private DoublePreferenceConstant commandDownshiftSpeed;
  private DoublePreferenceConstant commandDownshiftCommandValue;
  private DoublePreferenceConstant highGearRatioValue;
  private DoublePreferenceConstant lowGearRatioValue;

  private boolean isOnLimelightTarget = false;

  // Constants for negative inertia
  private static final double LARGE_TURN_RATE_THRESHOLD = 0.65;
  private static final double INCREASE_TURN_SCALAR = 2;
  private static final double SMALL_DECREASE_TURN_SCALAR = 2.0;
  private static final double LARGE_DECREASE_TURN_SCALAR = 2.5;
  private double m_prevTurn = 0; // The last turn value
  private double m_negInertialAccumulator = 0; // Accumulates our current inertia value

  public Drive(Sensors sensors) {
    m_sensors = sensors;

    m_driveConfiguration = new DriveConfiguration();

    velPIDConstants = new PIDPreferenceConstants("Drive Vel", 1.0, 0.02, 0, 0, 2, 2, 0);
    headingPIDConstants = new PIDPreferenceConstants("Heading", .01, .0005, 0, 0, 3, 1, 0.25);
    downshiftSpeed = new DoublePreferenceConstant("Downshift Speed", 4.5);
    upshiftSpeed = new DoublePreferenceConstant("UpshiftSpeed", 6);
    commandDownshiftSpeed = new DoublePreferenceConstant("Command Downshift Speed", 5);
    commandDownshiftCommandValue = new DoublePreferenceConstant("Command Downshift Command Value", 0.1);
    
    highGearRatioValue = new DoublePreferenceConstant("High Gear Ratio", Constants.HIGH_DRIVE_RATIO);
    lowGearRatioValue = new DoublePreferenceConstant("Low Gear Ratio", Constants.LOW_DRIVE_RATIO);

    m_leftTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_LEFT_LOW_EFFICIENCY, Constants.DRIVE_LEFT_HIGH_EFFICIENCY);
    m_rightTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_RIGHT_LOW_EFFICIENCY, Constants.DRIVE_RIGHT_HIGH_EFFICIENCY);
    
    m_leftVelPID = new SyncPIDController(velPIDConstants);
    m_rightVelPID = new SyncPIDController(velPIDConstants);
    m_leftTransmission.setVelocityPID(m_leftVelPID);
    m_rightTransmission.setVelocityPID(m_rightVelPID);

    m_leftEncoder = new CANCoder(Constants.LEFT_DRIVE_ENCODER_ID);
    m_rightEncoder = new CANCoder(Constants.RIGHT_DRIVE_ENCODER_ID);

    m_leftEncoder.configFactoryDefault();
    m_rightEncoder.configFactoryDefault();

    m_leftEncoder.configSensorDirection(true);
    m_rightEncoder.configSensorDirection(true);

    m_leftDrive = new TJDriveModule(m_driveConfiguration.left, m_leftTransmission);
    m_rightDrive = new TJDriveModule(m_driveConfiguration.right, m_rightTransmission);

    m_leftDrive.configRemoteFeedbackFilter(m_leftEncoder, 0);
    m_rightDrive.configRemoteFeedbackFilter(m_rightEncoder, 0);

    m_leftShifter = new DoubleSolenoid(Constants.SHIFTER_LEFT_PCM, Constants.SHIFTER_LEFT_OUT,
        Constants.SHIFTER_LEFT_IN);
    m_rightShifter = new DoubleSolenoid(Constants.SHIFTER_RIGHT_PCM, Constants.SHIFTER_RIGHT_OUT,
        Constants.SHIFTER_RIGHT_IN);

    m_headingPID = new WrappingPIDController(180, -180, headingPIDConstants);

    shiftToLow();

    SmartDashboard.putBoolean("Zero Drive", false);

    // Creating my kinematics object
    m_kinematics = new DifferentialDriveKinematics(Units.feetToMeters(Constants.WHEEL_BASE_WIDTH));

    // generate trajectories
    trajectories = new GameChangerTrajectories();

    // Creating my odometry object
    // our starting pose is 1 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    m_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_sensors.getYaw()), m_pose);
  }

  public void basicDrive(double leftSpeed, double rightSpeed) {
    m_leftDrive.set(ControlMode.PercentOutput, leftSpeed);
    m_rightDrive.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Commands the drivetrain to the given velocities (in fps) while proactively
   * limiting current draw.
   */
  public void basicDriveLimited(double leftVelocity, double rightVelocity) {
    double leftExpectedCurrent = m_leftDrive.getExpectedCurrentDraw(leftVelocity);
    double rightExpectedCurrent = m_rightDrive.getExpectedCurrentDraw(rightVelocity);
    double totalExpectedCurrent = leftExpectedCurrent + rightExpectedCurrent;
    double leftCurrentLimit;
    double rightCurrentLimit;
    if (totalExpectedCurrent == 0) {
      leftCurrentLimit =  m_currentLimit / 2.;
      rightCurrentLimit = m_currentLimit / 2.;
    } else {
      leftCurrentLimit = m_currentLimit * leftExpectedCurrent / totalExpectedCurrent;
      rightCurrentLimit = m_currentLimit * rightExpectedCurrent / totalExpectedCurrent;
    }

    m_leftDrive.setVelocityCurrentLimited(leftVelocity, leftCurrentLimit);
    m_rightDrive.setVelocityCurrentLimited(rightVelocity, rightCurrentLimit);
    
    m_leftCommandedSpeed = leftVelocity;
    m_rightCommandedSpeed = rightVelocity;
  }

  /**
   * Arcade drive function for teleop control.
   * 
   * Parameters:
   * 
   * @param speed    The forwards/backwards speed on a scale from -1 to 1
   * @param turnRate The rate to turn at on a scale from -1 (counterclockwise) to
   *                 1 (clockwise)
   */
  public void arcadeDrive(double speed, double turn) {
    // Apply negative intertia
    turn = negativeInertia(speed, turn);

    // Convert to feet per second
    speed *= m_maxSpeed;
    turn *= m_maxSpeed;
    
    // Calculate left and right speed
    double leftSpeed = (speed + turn);
    double rightSpeed = (speed - turn);

    // Apply values
    basicDriveLimited(leftSpeed, rightSpeed);
  }

  public void turnToHeading(double heading) {
    double turnRate = m_headingPID.calculateOutput(m_sensors.getYaw(), heading);
    basicDrive(turnRate, -turnRate);
  }

  public void resetHeadingPID() {
    m_headingPID.reset();
  }

  public boolean autoshift(double commandedValue) {
    double currentSpeed = getStraightSpeed();
    if (isInHighGear() && Math.abs(currentSpeed) <= downshiftSpeed.getValue()) {
      return false;
    } else if (!isInHighGear() && Math.abs(currentSpeed) >= upshiftSpeed.getValue()) {
      return true;
    } else if (isInHighGear() && Math.abs(currentSpeed) <= commandDownshiftSpeed.getValue()
        && (Math.signum(commandedValue) != Math.signum(currentSpeed)
        || Math.abs(commandedValue) <= commandDownshiftCommandValue.getValue())) {
      return false;
    } else {
      return isInHighGear();
    }
  }

  public void shiftToLow() {
    m_leftShifter.set(Value.kForward);
    m_rightShifter.set(Value.kForward);

    m_leftTransmission.shiftToLow();
    m_rightTransmission.shiftToLow();
  }

  public void shiftToHigh() {
    m_leftShifter.set(Value.kReverse);
    m_rightShifter.set(Value.kReverse);

    m_leftTransmission.shiftToHigh();
    m_rightTransmission.shiftToHigh();
  }

  public boolean isInHighGear() {
    return m_leftTransmission.isInHighGear();
  }

  public void resetEncoderPositions() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftPosition() {
    return m_leftDrive.getScaledSensorPosition();
  }

  public double getRightPosition() {
    return m_rightDrive.getScaledSensorPosition();
  }

  public double getLeftSpeed() {
    return m_leftDrive.getScaledSensorVelocity();
  }

  public double getRightSpeed() {
    return m_rightDrive.getScaledSensorVelocity();
  }

  public double getStraightSpeed() {
    // Return forwards (X) velocity in feet per second
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  public double getTurnSpeed() {
    // Return rotational velocity in degrees per second
    return Units.radiansToDegrees((getRightSpeed() - getLeftSpeed()) / Constants.WHEEL_BASE_WIDTH);
  }

  public void setBrakeMode() {
    m_leftDrive.brakeAll();
    m_rightDrive.brakeAll();
  }

  public void setCoastMode() {
    m_leftDrive.coastAll();
    m_rightDrive.coastAll();
  }

  public void setMaxSpeed(double maxSpeed) {
    m_maxSpeed = maxSpeed;
  }

  public void setOnLimelightTarget(boolean onLimelightTarget) {
    this.isOnLimelightTarget = onLimelightTarget;
  }

  public boolean isOnLimelightTarget() {
    return this.isOnLimelightTarget;
  }

  public Pose2d getCurrentPose() {
    return m_pose;
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Units.feetToMeters(getLeftSpeed()), Units.feetToMeters(getRightSpeed())));
  }

  public DifferentialDriveWheelSpeeds wheelSpeedsFromChassisSpeeds(ChassisSpeeds speeds) {
    return m_kinematics.toWheelSpeeds(speeds);
  }

  // Negative inertia! The idea is that the robot has some inertia
  // which theoretically is based on previously commanded values. Returns an
  // updated turn value
  private double negativeInertia(double throttle, double turn) {

      // How much we are currently trying to change the turn value
      double turnDiff = turn - m_prevTurn;
      m_prevTurn = turn;

      // Determine which scaling constant to use based on how we are moving
      double negInertiaScalar;
      if (turn * turnDiff > 0) {
          // We are trying to increase our turning rate
          negInertiaScalar = INCREASE_TURN_SCALAR;
      } else {
          if (Math.abs(turn) < LARGE_TURN_RATE_THRESHOLD) {
              // We are trying to reduce our turning rate to something
              // relatively close to 0
              negInertiaScalar = SMALL_DECREASE_TURN_SCALAR;
          } else {
              // We are trying to reduce our turning rate, but still want to
              // be turning fairly fast
              negInertiaScalar = LARGE_DECREASE_TURN_SCALAR;
          }
      }

      // Apply the scalar, and add it to the accumulator
      double negInertiaPower = turnDiff * negInertiaScalar;
      m_negInertialAccumulator += negInertiaPower;

      // Add the current negative inertia value to the turn
      double updatedTurn = turn + m_negInertialAccumulator;

      // Reduce our current inertia
      if (m_negInertialAccumulator > 1) {
          m_negInertialAccumulator -= 1;
      } else if (m_negInertialAccumulator < -1) {
          m_negInertialAccumulator += 1;
      } else {
          m_negInertialAccumulator = 0;
      }

      return updatedTurn;
  }

  public void zeroDrive() {
    m_sensors.zeroYaw();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()), new Rotation2d());
  }

  public void resetOdometry(Pose2d startPose, Rotation2d startGyro) {
    m_odometry.resetPosition(startPose, startGyro);
  }

  public void updateOdometry() {
    m_pose = m_odometry.update(Rotation2d.fromDegrees(-m_sensors.getYaw()), Units.feetToMeters(getLeftPosition()), Units.feetToMeters(getRightPosition()));
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("Zero Drive", false)) {
      zeroDrive();
      resetOdometry();
      SmartDashboard.putBoolean("Zero Drive", false);
    }
    
    updateOdometry();

    SmartDashboard.putNumber("L Drive Current", m_leftDrive.getTotalCurrent());
    SmartDashboard.putNumber("R Drive Current", m_rightDrive.getTotalCurrent());
    SmartDashboard.putNumber("L Drive Speed", m_leftDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("R Drive Speed", m_rightDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("L Drive Position", m_leftDrive.getScaledSensorPosition());
    SmartDashboard.putNumber("R Drive Position", m_rightDrive.getScaledSensorPosition());
    SmartDashboard.putNumber("L Drive Command Speed", m_leftCommandedSpeed);
    SmartDashboard.putNumber("R Drive Command Speed", m_rightCommandedSpeed);
    SmartDashboard.putNumber("L Drive Voltage", m_leftDrive.getMotorOutputVoltage());
    SmartDashboard.putNumber("R Drive Voltage", m_rightDrive.getMotorOutputVoltage());
    SmartDashboard.putBoolean("In High Gear?", isInHighGear());
    SmartDashboard.putNumber("Max Drive Speed", m_maxSpeed);
    SmartDashboard.putBoolean("LimelightHeadingOnTarget", isOnLimelightTarget);

    SmartDashboard.putNumber("Pose X", Units.metersToFeet(m_pose.getX()));
    SmartDashboard.putNumber("Pose Y", Units.metersToFeet(m_pose.getY()));
    SmartDashboard.putNumber("Pose Rotation", m_pose.getRotation().getDegrees());
    ChassisSpeeds speeds = getCurrentChassisSpeeds();
    SmartDashboard.putNumber("Linear Velocity X", Units.metersToFeet(speeds.vxMetersPerSecond));
    SmartDashboard.putNumber("Linear Velocity Y", Units.metersToFeet(speeds.vyMetersPerSecond));
    SmartDashboard.putNumber("Angular Velocity", Units.metersToFeet(speeds.omegaRadiansPerSecond));

    if (DriverStation.getInstance().isEnabled()) {
      this.setBrakeMode();
    } else {
      this.setCoastMode();
    }
  }
}
