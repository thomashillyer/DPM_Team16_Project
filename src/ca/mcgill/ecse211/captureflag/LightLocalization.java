package ca.mcgill.ecse211.captureflag;

import java.util.Arrays;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class provides the light localization that can be used at any point to relocalize to the
 * nearest grid intersection
 */

public class LightLocalization {
  // object instances
  private Navigation nav;
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  // data
  private int filterCounter = 0;
  private float oldValue = 0;
  private int derivativeThreshold = -60;

  private int lineCounter = 0;
  private double xminus, xplus, yminus, yplus;
  private double thetax, thetay;
  private double x, y;
  private double deltaThetaY;
  private int x0;
  private int y0;
  private int xC;
  private int yC;
  private int corner;

  private double currX = 0.0;
  private double currY = 0.0;
  private double currTheta = 0.0;

  private boolean detectSingleLine = false;
  private boolean detectFourLines = false;
  private boolean cornerLocalization = false;

  private double[] lines = new double[4];
  private static final double centerDistanceD = 13.5;

  private EV3LargeRegulatedMotor[] syncList = new EV3LargeRegulatedMotor[1];

  /**
   * The constructor for LightLocalization class.
   * 
   * @param leftMotor An instance of the EV3LargeRegulatedMotor that controls the left motor.
   * @param rightMotor An instance of the EV3LargeRegulatedMotor that controls the right motor.
   * @param odometer An instance of the Odometer class.
   * @param nav An instance of the Navigation class.
   */
  public LightLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Odometer odometer, Navigation nav) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;
    this.nav = nav;
    Sound.setVolume(85);
  }

  /**
   * Method will localize the robot at any point along the board. The robot will end up on the point
   * and orient to 0 degrees after the localization
   * 
   * @param x_start The x position of the point the robot is supposed to be at
   * @param y_start The y position of the point the robot is supposed to be at
   */
  public void anyPointLocalization(int x_start, int y_start) {
    lineCounter = 0;
    lines = new double[4];
    leftMotor.setAcceleration(CaptureFlag.ACCELERATION);
    rightMotor.setAcceleration(CaptureFlag.ACCELERATION);
    leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
    rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
    detectFourLines = true;
    leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 378),
        true); // 375
    rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 378),
        false);
    detectFourLines = false;
    double xminus = 0, xplus = 0, yminus = 0, yplus = 0;

    for (int i = 0; i < lines.length - 1; i++) {
      for (int j = 0; j < lines.length - 1; j++) {
        if (lines[j] > lines[j + 1]) {
          double temp = lines[j];
          lines[j] = lines[j + 1];
          lines[j + 1] = temp;
        }
      }
    }

    if (lines[0] > (2 * Math.PI - lines[3])) {
      double line_angles_copy[] = new double[4];

      line_angles_copy[0] = lines[3];

      for (int i = 0; i < lines.length - 1; i++) {
        line_angles_copy[i + 1] = lines[i];
      }

      lines = line_angles_copy;
    }

    yminus = lines[0];
    xminus = lines[1];
    yplus = lines[2];
    xplus = lines[3];

    if (yminus < yplus) {
      yminus += 2 * Math.PI;
    }

    if (xplus < xminus) {
      xplus += 2 * Math.PI;
    }
    // from the light sensor values that we got
    // correct x , y , and theta by the values we got from the sensor

    double thetay = yminus - yplus;
    double thetax = xplus - xminus;

    double x = x_start * CaptureFlag.TILE_LENGTH - centerDistanceD * Math.cos(thetay / 2.0);
    double y = y_start * CaptureFlag.TILE_LENGTH - centerDistanceD * Math.cos(thetax / 2.0);
    double deltaTheta = (Math.PI / 2.0) - (yminus) + Math.PI + (thetay / 2.0);

    odometer.setX(x);
    odometer.setY(y);
    odometer.setTheta(odometer.getTheta() + deltaTheta);
  }

  /**
   * This method gets input from the LightPoller instance that is running. The method will check if
   * the difference between the passed in value and the previously passed value is less than the
   * threshold for detecting a black line. It then handles what happens if a black line is detected
   * depending on the state of the robot.
   * 
   * @param value A value that is passed in from the LightPoller class. The value is retrieved by
   *        sampling the color sensor.
   */
  protected void processData(int value) {
    long correctionStart, correctionEnd;
    correctionStart = System.currentTimeMillis();

    // computing the derivative at each point
    float diff = value - oldValue;

    // storing the current value, to be able to get the derivative on
    // the next iteration
    oldValue = value;
    if (diff < derivativeThreshold && filterCounter == 0) {
      filterCounter++;
      if (detectFourLines) {
        lineCounter++;
        Sound.beep();
        if (lineCounter - 1 < 4)
          lines[lineCounter - 1] = odometer.getTheta();

        if (lineCounter == 1) {
          xminus = odometer.getTheta();

        } else if (lineCounter == 2) {
          yplus = odometer.getTheta();

        } else if (lineCounter == 3) {
          xplus = odometer.getTheta();

        } else if (lineCounter == 4) {
          yminus = odometer.getTheta();
        }
      } else if (detectSingleLine) {
        detectSingleLine = false;
      }
    } else if (diff < derivativeThreshold && filterCounter > 0) {
      filterCounter++;
    } else if (diff > derivativeThreshold) {
      filterCounter = 0;
    }
  }

  /**
   * This method localizes specifically from the starting corner of the board. The following are the
   * steps it takes:
   * <ol>
   * <li>It turns the robot 45 degrees</li>
   * <li>The robot goes forward until a line is detected</li>
   * <li>The robot reverses by the length of the robot</li>
   * <li>The robot rotates 360 degrees to detect four black lines</li>
   * <li>The distance and angle offset are calculated</li>
   * <li>Corrects the robots orientation</li>
   * </ol>
   */
  protected void cornerLocalization(int corner) {
    detectSingleLine = true;
    cornerLocalization = true;
    // Adjust the robot position, before it starts to rotate to ensure that
    // the light sensor will cross 4 black lines
    adjustRobotStartingPosition();
    anyPointLocalization(0, 0);
    nav.travelTo(0, 0);
    nav.turn(-(odometer.getTheta() + Math.toRadians(8)));

    if (corner == 0) {
      odometer.setX(CaptureFlag.TILE_LENGTH);
      odometer.setY(CaptureFlag.TILE_LENGTH);
      odometer.setTheta(0);
    } else if (corner == 1) {
      odometer.setX(11 * CaptureFlag.TILE_LENGTH);
      odometer.setY(CaptureFlag.TILE_LENGTH);
      odometer.setTheta(3 * Math.PI / 2);
    } else if (corner == 2) {
      odometer.setX(11 * CaptureFlag.TILE_LENGTH);
      odometer.setY(11 * CaptureFlag.TILE_LENGTH);
      odometer.setTheta(Math.PI);
    } else if (corner == 3) {
      odometer.setX(CaptureFlag.TILE_LENGTH);
      odometer.setY(11 * CaptureFlag.TILE_LENGTH);
      odometer.setTheta(Math.PI / 2);
    }
    cornerLocalization = false;
  }

  /**
   * This method calculates the robot's actual x and y position using the angle values determined
   * earlier.Using trigonometry, the theta by which the odometer is off is calculated. Then, it
   * updates the x, y, and theta values of the odometer in order to fix them. Then, it uses the
   * navigation travelTo method to travel to the inputed destination.
   */
  private void correctOdometer(double incomeTheta) {
    Arrays.sort(lines);
    double theta = lines[0] * 180 / Math.PI;
    if (theta >= 0 && theta < 40) {
      thetay = lines[2] - lines[0];
      thetax = lines[3] - lines[1];
    } else {
      thetay = lines[3] - lines[1];
      thetax = lines[2] - lines[0];
    }

    this.x = -CaptureFlag.BOT_LENGTH * Math.cos(thetay / 2.0);
    this.y = -CaptureFlag.BOT_LENGTH * Math.cos(thetax / 2.0);

    if (theta >= 0 && theta < 40) {
      deltaThetaY = (Math.PI / 2.0) - lines[0] + Math.PI + (thetay / 2.0);
    } else {
      deltaThetaY = (Math.PI / 2.0) - lines[3] + Math.PI + (thetay / 2.0);
    }

    if ((incomeTheta * 180 / Math.PI >= 270 && incomeTheta * 180 / Math.PI < 360)
        || (incomeTheta * 180 / Math.PI >= 180 && incomeTheta * 180 / Math.PI < 270)
        || (incomeTheta * 180 / Math.PI >= 90 && incomeTheta * 180 / Math.PI < 180)) {
      odometer.setX(-this.x);
      odometer.setY(this.y);
    } else {
      odometer.setX(this.x);
      odometer.setY(this.y);
    }

    if (cornerLocalization) {
      if (odometer.getTheta() > 2 * Math.PI) {
        odometer.setTheta(odometer.getTheta() + deltaThetaY - 2 * Math.PI);
      } else {
        odometer.setTheta(odometer.getTheta() + deltaThetaY);
      }
    }
  }

  /**
   * This method adjusts the robot position before it starts rotating. It ensures that the light
   * sensor, mounted on the back of the robot, runs over 4 black lines. It order to so, the robot,
   * facing north after the ultrasonic localization, moves forward until it detects a line, and then
   * move backwards 1.5 times its center distance. This, ensures that the robot is close enough to
   * the horizontal black line facing the robot. Then, the method makes the robot turn 90 degrees
   * and repeat the same procedure, so it can be placed close enough to the vertical line. Now the
   * robot it close enough to the first intersection.
   */
  private void adjustRobotStartingPosition() {
    // Set the wheel's rotation speed to ROTATESPEED
    leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED + 70);
    rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED + 70);

    // Rotate the robot by 45 degrees
    leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 45),
        true);
    rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 45),
        false);
    rightMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, 20), true);
    leftMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, 20), false);

  }
}
