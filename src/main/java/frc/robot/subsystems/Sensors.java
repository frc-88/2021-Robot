/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.NavX;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.Constants;
import frc.robot.util.Limelight;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {

  private final NavX m_navx;
  private final Limelight limelight;
  private BooleanSupplier ledOverride;

  private boolean m_powerCellDetected = false;
  private final DoublePreferenceConstant m_limelightHeight = new DoublePreferenceConstant("Limelight Height", 19.5);
  private final DoublePreferenceConstant m_limelightAngle = new DoublePreferenceConstant("Limelight Angle", 20.0);
  private final DoublePreferenceConstant m_limelightOffset = new DoublePreferenceConstant("Limelight Offset", 8.0);
  private double m_yawOffset = 0.0;

  private DigitalInput shooterBallSensor;
  private DigitalInput feederMouthSensor;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors(BooleanSupplier ledOverride) {
    this.ledOverride = ledOverride;

    m_navx = new NavX();

    limelight = new Limelight();
    limelight.camVision();
    limelight.ledOff();
    SmartDashboard.putNumber("Limelight Test Distance", 120.0);

    shooterBallSensor = new DigitalInput(Constants.SHOOTER_BALL_SENSOR_ID);
    feederMouthSensor = new DigitalInput(Constants.FEEDER_MOUTH_SENSOR_ID);

    startPowerCellDetector();
  }

  public void startPowerCellDetector() {
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(Constants.PCD_IMAGE_WIDTH, Constants.PCD_IMAGE_HEIGHT);
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo(Constants.PCD_STREAM_NAME, Constants.PCD_FRAME_WIDTH,
          Constants.PCD_FRAME_HEIGHT);

      Mat source = new Mat();
      Mat output = new Mat();
      Mat hierarchy = new Mat();
      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }

        contours.clear();

        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        output = new Mat(output, new Rect(Constants.PCD_FRAME_X, Constants.PCD_FRAME_Y, Constants.PCD_FRAME_WIDTH,
            Constants.PCD_FRAME_HEIGHT));
        Imgproc.blur(output, output, new Size(Constants.PCD_BLUR, Constants.PCD_BLUR));
        Core.inRange(output, new Scalar(Constants.PCD_HUE_LO, Constants.PCD_SAT_LO, Constants.PCD_VAL_LO),
            new Scalar(Constants.PCD_HUE_HI, Constants.PCD_SAT_HI, Constants.PCD_VAL_HI), output);
        Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        m_powerCellDetected = contours.size() > 0;

        outputStream.putFrame(output);
      }
    }).start();
  }

  public void zeroYaw() {
    m_yawOffset = m_navx.getYaw();
  }

  public double getYaw() {
    return m_navx.getYaw() - m_yawOffset;
  }

  public double getYawRate() {
    return m_navx.getYawRate();
  }

  public void ledOn() {
    limelight.ledOn();
  }

  public void ledOff() {
    if (!(DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean())) {
      limelight.ledOff();
    }
  }

  public boolean powerCellDetected() {
    return m_powerCellDetected;
  }

  public double getDistanceToTarget() {
    double distance = 0;

    if (limelight.isConnected() && limelight.hasTarget()) {
      //
      // After further analysis of the spreadsheet data,
      // a polynomial curve matches the empirical data best
      // Sorry, Bill, for the magic numbers. They came from
      // a spreadsheet, I swear! :D
      // distance = 190 - 11.7 * ty + 0.32 * ty * ty;

      double ty = limelight.getTargetVerticalOffsetAngle();

      distance = (Constants.FIELD_PORT_TARGET_HEIGHT - m_limelightHeight.getValue())
          / Math.tan(Math.toRadians(m_limelightAngle.getValue() + ty));

    }

    return distance;
  }

  public double getAngleToTarget() {
    return -limelight.getTargetHorizontalOffsetAngle();
  }

  public double getShooterAngle() {
    double distance = getDistanceToTarget();
    double tx = -limelight.getTargetHorizontalOffsetAngle();

    return Math.toDegrees(Math.atan(((distance * Math.sin(Math.toRadians(tx))) + m_limelightOffset.getValue())
        / (distance * Math.cos(Math.toRadians(tx)))));
  }

  public double calcLimelightAngle() {
    double distance = SmartDashboard.getNumber("Limelight Test Distance", 120.0);
    double ty = limelight.getTargetVerticalOffsetAngle();

    return Math.toDegrees(Math.atan((Constants.FIELD_PORT_TARGET_HEIGHT - m_limelightHeight.getValue()) / distance))
        - ty;
  }

  public boolean doesLimelightHaveTarget() {
    return limelight.hasTarget();
  }

  public boolean hasBallInShooter() {
    return !shooterBallSensor.get();
  }

  public boolean hasBallAtMouth() {
    return !feederMouthSensor.get();
  }

  @Override
  public void periodic() {
    // NavX data
    SmartDashboard.putNumber("NavX Yaw", getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", getYawRate());
    SmartDashboard.putNumber("NavX Pitch", m_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", limelight.hasTarget());
    SmartDashboard.putNumber("Limelight Distance", getDistanceToTarget());
    SmartDashboard.putNumber("Limelight H-Angle", getAngleToTarget());
    SmartDashboard.putNumber("Limelight V-Angle", limelight.getTargetVerticalOffsetAngle());
    SmartDashboard.putNumber("Limelight Shooter Angle", getShooterAngle());
    SmartDashboard.putNumber("Limelight Calc Angle", calcLimelightAngle());

    // Beam breaks
    SmartDashboard.putBoolean("Shooter Ball Sensor", shooterBallSensor.get());
    SmartDashboard.putBoolean("Feeder Mouth Ball Sensor", feederMouthSensor.get());

    // Check LED override, only when disabled
    if (DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean()) {
      limelight.ledOn();
    }

    // PCD data
    SmartDashboard.putBoolean("Powercell Detected?", m_powerCellDetected);
  }
}
