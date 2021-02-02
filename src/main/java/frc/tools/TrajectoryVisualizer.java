package frc.tools;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.Stroke;
import java.util.ArrayList;
import java.util.List;
import javax.swing.*;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.util.GameChangerTrajectories;

@SuppressWarnings("serial")
public class TrajectoryVisualizer extends JPanel {
   private static final int PREF_W = 800;
   private static final int PREF_H = 650;
   private static final int BORDER_GAP = 30;
   private static final Color GRAPH_COLOR = Color.green;
   private static final Color GRAPH_POINT_COLOR = new Color(150, 50, 50, 180);
   private static final Stroke GRAPH_STROKE = new BasicStroke(2f);
   private static final int GRAPH_POINT_WIDTH = 12;
   private static final int Y_HATCH_CNT = 16;
   private static final int X_HATCH_CNT = 31;
   private static final double WHEEL_BASE_WIDTH = Units.feetToMeters((25. + 5. / 16.) / 12.);
   private static final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(WHEEL_BASE_WIDTH);

   private List<State> states;
   private double duration;

   public TrajectoryVisualizer(List<State> states, double duration) {
      this.states = states;
      this.duration = duration;
   }

   @Override
   protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2 = (Graphics2D) g;
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      double maxVelocity = 0;
      double maxAcceleration = 0;
      double xScale = ((double) getWidth() - 2 * BORDER_GAP) / 30;
      double yScale = ((double) getHeight() - 2 * BORDER_GAP) / 15;

      List<Point> track = new ArrayList<Point>();
      List<Point> trackLeft = new ArrayList<Point>();
      List<Point> trackRight = new ArrayList<Point>();

      for (int i = 0; i < states.size(); i++) {
         State state = states.get(i);
         Pose2d pose = state.poseMeters;
         maxVelocity = Math.max(maxVelocity, state.velocityMetersPerSecond);
         maxAcceleration = Math.max(maxAcceleration, state.accelerationMetersPerSecondSq);
         
         int x1 = (int) (Units.metersToFeet(pose.getX()) * xScale + BORDER_GAP);
         int y1 = (int) ((15 - Units.metersToFeet(pose.getY())) * yScale + BORDER_GAP);

         double rotation = pose.getRotation().getRadians();
         int xL = (int) (x1 - (WHEEL_BASE_WIDTH / 2 * Math.sin(rotation)) * xScale);
         int yL = (int) (y1 - (WHEEL_BASE_WIDTH / 2 * Math.cos(rotation)) * yScale);

         int xR = (int) (x1 + (WHEEL_BASE_WIDTH / 2 * Math.sin(rotation)) * xScale);
         int yR = (int) (y1 + (WHEEL_BASE_WIDTH / 2 * Math.cos(rotation)) * yScale);

         track.add(new Point(x1, y1));
         trackLeft.add(new Point(xL, yL));
         trackRight.add(new Point(xR, yR));
      }

      drawAxes(g2);

      g2.drawString("Duration: " + duration, 100, 100);
      g2.drawString("Max Velocity: " + Units.metersToFeet(maxVelocity), 100, 150);
      g2.drawString("Max Acceleration: " + Units.metersToFeet(maxAcceleration), 100, 200);

      g2.setColor(GRAPH_COLOR);
      drawTrack(g2, track, true);
      g2.setColor(Color.blue);
      drawTrack(g2, trackLeft, false);
      g2.setColor(Color.red);
      drawTrack(g2, trackRight, false);
   }

   private void drawTrack(Graphics2D g2, List<Point> track, boolean drawPoints) {
      Stroke oldStroke = g2.getStroke();
      g2.setStroke(GRAPH_STROKE);
      for (int i = 0; i < track.size() - 1; i++) {
         int x1 = track.get(i).x;
         int y1 = track.get(i).y;
         int x2 = track.get(i + 1).x;
         int y2 = track.get(i + 1).y;
         g2.drawLine(x1, y1, x2, y2);
      }
      g2.setStroke(oldStroke);

      if (drawPoints) {
         g2.setColor(GRAPH_POINT_COLOR);
         for (int i = 0; i < track.size(); i++) {
            int x = track.get(i).x - GRAPH_POINT_WIDTH / 2;
            int y = track.get(i).y - GRAPH_POINT_WIDTH / 2;
            ;
            int ovalW = GRAPH_POINT_WIDTH;
            int ovalH = GRAPH_POINT_WIDTH;
            g2.fillOval(x, y, ovalW, ovalH);
         }
      }
   }

   private void drawAxes(Graphics2D g2) {
      // create x and y axes
      g2.drawLine(BORDER_GAP, getHeight() - BORDER_GAP, BORDER_GAP, BORDER_GAP);
      g2.drawLine(BORDER_GAP, getHeight() - BORDER_GAP, getWidth() - BORDER_GAP, getHeight() - BORDER_GAP);

      // create hatch marks for y axis.
      for (int i = 0; i < Y_HATCH_CNT; i++) {
         int x0 = BORDER_GAP;
         int x1 = GRAPH_POINT_WIDTH + BORDER_GAP;
         int y0 = getHeight() - ((i + 1) * (getHeight() - BORDER_GAP * 2) / Y_HATCH_CNT + BORDER_GAP);
         int y1 = y0;
         g2.drawLine(x0, y0, x1, y1);
      }

      // and for x axis
      for (int i = 0; i < X_HATCH_CNT; i++) {
         int x0 = (i + 1) * (getWidth() - BORDER_GAP * 2) / X_HATCH_CNT + BORDER_GAP;
         int x1 = x0;
         int y0 = getHeight() - BORDER_GAP;
         int y1 = y0 - GRAPH_POINT_WIDTH;
         g2.drawLine(x0, y0, x1, y1);
      }
   }

   @Override
   public Dimension getPreferredSize() {
      return new Dimension(PREF_W, PREF_H);
   }

   private static void createAndShowGui() {
      // define constraints for trajectory generation
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(16.0), Units.feetToMeters(8.0));
      config.setKinematics(m_kinematics);
      config.setStartVelocity(0.0);
      config.setEndVelocity(0.0);

      config.addConstraint(new DifferentialDriveKinematicsConstraint(m_kinematics, Units.feetToMeters(16.0)));
      config.addConstraint(new CentripetalAccelerationConstraint(2.5));

      GameChangerTrajectories trajectories = new GameChangerTrajectories(config);
      Trajectory traj = trajectories.barrelRun;
      List<State> states = new ArrayList<State>();

      for (double t = 0; t < traj.getTotalTimeSeconds(); t += 0.1) {
         states.add(traj.sample(t));
      }

      TrajectoryVisualizer mainPanel = new TrajectoryVisualizer(states, traj.getTotalTimeSeconds());

      JFrame frame = new JFrame("DrawGraph");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.getContentPane().add(mainPanel);
      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
   }

   public static void main(String[] args) {
      SwingUtilities.invokeLater(new Runnable() {
         public void run() {
            createAndShowGui();
         }
      });
   }
}
