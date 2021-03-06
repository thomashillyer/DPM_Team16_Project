package ca.mcgill.ecse211.captureflag;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Navigation class implements the logic of moving from one point to another, using the shortest
 * path.
 */

public class Navigation extends Thread {

  private double currX = 0.0;
  private double currY = 0.0;
  private double currTheta = 0.0;

  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  /**
   * This is the constructor for the Navigation class.
   * 
   * @param odometer An instance of the Odometer class.
   * @param leftMotor An instance of the EV3LargeRegulatedMotor that controls the left motor.
   * @param rightMotor An instance of the EV3LargeRegulatedMotor that controls the right motor.
   */
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }



  /**
   * This method causes the robot to travel to the absolute field location (x, y) ,specified in tile
   * points.This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will pool the odometer for information.
   * 
   * @param x The new x destination of the robot
   * @param y The new y destination of the robot
   */

  public void travelTo(double x, double y) {

    currX = odometer.getX();
    currY = odometer.getY();
    currTheta = odometer.getTheta();

    // calculating the information needed (destination - current) for both y
    // and x, in order to calculate the minimum angle using arctan
    double deltaX = (x * CaptureFlag.TILE_LENGTH) - currX;
    double deltaY = (y * CaptureFlag.TILE_LENGTH) - currY;

    // rotate the robot towards its new way point
    turnTo(x, y);

    // calculate the distance to next point using the built in pythagorean
    // theorem
    double distToTravel = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
    distToTravel = Math.sqrt(distToTravel);

    // travel to the next point, and don't wait until the action is
    // complete. So the boolean in both rotate method should be true
    leftMotor.setSpeed(CaptureFlag.FORWARDSPEED);
    rightMotor.setSpeed(CaptureFlag.FORWARDSPEED);
    leftMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, distToTravel), true);
    rightMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, distToTravel), false);
  }

  /**
   * This method turns to the angle passed, it takes an angle theta as input. It starts by setting
   * the speed of both motors. The angle is between -360 and 360 degrees, but since this methods
   * should always make the robot turn with the minimum angle possible, theta should be updated. If
   * theta is greater than 180 degrees, instead of turning positively, the robot turns negatively by
   * an angle of theta - 360. And if theta is less than or equal to -180, instead of turning
   * negatively, the robot turns positevly by an angle of theta + 360.
   * 
   * @param theta The angle to which the cart should turn.
   */
  public void turnTo(double x, double y) {

    // get the current position and rotation of the robot, based on its
    // starting point

    currX = odometer.getX();
    currY = odometer.getY();
    currTheta = odometer.getTheta();

    // calculating the information needed (destination - current) for both y
    // and x, in order to calculate the minimum angle using arctan
    double deltaX = (x * CaptureFlag.TILE_LENGTH) - currX;
    double deltaY = (y * CaptureFlag.TILE_LENGTH) - currY;

    // calculating the minimum angle using Math.atan2 method
    double theta = Math.atan2(deltaX, deltaY) - currTheta;
    turn(theta);
  }

  /**
   * This method turns by the angle given.
   * 
   * @param theta The angle to turn by.
   */
  protected void turn(double theta) {
    leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
    rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);

    // adjusting the angle in order to have an optimal turn (a turn with the
    // minimum angle)
    if (theta <= -Math.PI) {
      theta += Math.PI * 2;
    } else if (theta > Math.PI) {
      theta -= Math.PI * 2;
    }

    theta = Math.toDegrees(theta);

    // turn to the left if angle is negative
    if (theta < 0) {
      leftMotor.rotate(
          -CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, -theta), true);// +2
      rightMotor.rotate(
          CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, -theta), false);
    }
    // turn to the right if angle is positive
    else {
      leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, theta),
          true);// -2
      rightMotor.rotate(
          -CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, theta), false);
    }
  }

  /**
   * Method will have the robot navigate across the middle of the bridge based on the given points
   * 
   * @param sh_ll_x lower left hand corner of horizontal shallow water zone x
   * @param sh_ll_y lower left hand corner of horizontal shallow water zone y
   * @param sh_ur_x upper right hand corner of horizontal shallow water zone x
   * @param sh_ur_y upper right hand corner of horizontal shallow water zone y
   * @param sv_ll_x lower left hand corner of vertical shallow water zone x
   * @param sv_ll_y lower left hand corner of vertical shallow water zone y
   */
  protected void crossBridge(int sh_ll_x, int sh_ll_y, int sh_ur_x, int sh_ur_y, int sv_ll_x,
      int sv_ll_y) {
    // based on orientation of bridge, traverse it in a certain manner
    if (sh_ll_x < sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sh_ll_x, sh_ll_y + 0.5);
      travelTo(sh_ur_x - 0.5, sh_ur_y - 0.5);
      travelTo(sv_ll_x + 0.5, sv_ll_y);
    } else if (sh_ll_x < sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sh_ll_x + 0.5, sh_ll_y);
      travelTo(sh_ur_x - 0.5, sh_ur_y + 0.5);
      travelTo(sv_ll_x, sv_ll_y - 0.5);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sh_ll_x - 0.5, sh_ll_y);
      travelTo(sh_ur_x + 0.5, sh_ur_y + 0.5);
      travelTo(sv_ll_x, sv_ll_y - 0.5);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sh_ll_x, sh_ll_y + 0.5);
      travelTo(sh_ur_x + 0.5, sh_ur_y - 0.5);
      travelTo(sv_ll_x - 0.5, sv_ll_y);
    }
  }

  /**
   * Method will perform light localization at 1 point away from the entrance of the bridge
   * 
   * @param sh_ll_x lower left hand corner of horizontal shallow water zone x
   * @param sh_ll_y lower left hand corner of horizontal shallow water zone y
   * @param sh_ur_x upper right hand corner of horizontal shallow water zone x
   * @param sh_ur_y upper right hand corner of horizontal shallow water zone y
   * @param lp light poller
   * @param li light localization object
   */
  protected void localizeBeforeBridge(int sh_ll_x, int sh_ll_y, int sh_ur_x, int sh_ur_y,
      LightPoller lp, LightLocalization li) {
    if (sh_ll_x < sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sh_ll_x - 1, sh_ll_y);
      lp.restartTask();
      li.anyPointLocalization(sh_ll_x - 1, sh_ll_y);
      lp.killTask();
      travelTo(sh_ll_x - 1, sh_ll_y);
    } else if (sh_ll_x < sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sh_ll_x, sh_ll_y + 1);
      lp.restartTask();
      li.anyPointLocalization(sh_ll_x, sh_ll_y + 1);
      lp.killTask();
      travelTo(sh_ll_x, sh_ll_y + 1);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sh_ll_x, sh_ll_y + 1);
      lp.restartTask();
      li.anyPointLocalization(sh_ll_x, sh_ll_y + 1);
      lp.killTask();
      travelTo(sh_ll_x, sh_ll_y + 1);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sh_ll_x + 1, sh_ll_y);
      lp.restartTask();
      li.anyPointLocalization(sh_ll_x + 1, sh_ll_y);
      lp.killTask();
      travelTo(sh_ll_x + 1, sh_ll_y);
    }
  }

  /**
   * Method will perform light localization at 1 point away from the exit of the bridge
   * 
   * @param sh_ll_x lower left hand corner of horizontal shallow water zone x
   * @param sh_ll_y lower left hand corner of horizontal shallow water zone y
   * @param sh_ur_x upper right hand corner of horizontal shallow water zone x
   * @param sh_ur_y upper right hand corner of horizontal shallow water zone y
   * @param sv_ll_x lower left hand corner of vertical shallow water zone x
   * @param sv_ll_y lower left hand corner of vertical shallow water zone y
   * @param lp light poller
   * @param li light localization object
   */
  protected void localizeAfterBridge(int sh_ll_x, int sh_ll_y, int sh_ur_x, int sh_ur_y,
      int sv_ll_x, int sv_ll_y, LightPoller lp, LightLocalization li) {
    if (sh_ll_x < sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sv_ll_x, sv_ll_y - 1);
      lp.restartTask();
      li.anyPointLocalization(sv_ll_x, sv_ll_y - 1);
      lp.killTask();
      travelTo(sv_ll_x, sv_ll_y - 1);
    } else if (sh_ll_x < sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sv_ll_x - 1, sv_ll_y);
      lp.restartTask();
      li.anyPointLocalization(sv_ll_x - 1, sv_ll_y);
      lp.killTask();
      travelTo(sv_ll_x - 1, sv_ll_y);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y > sh_ur_y) {
      travelTo(sv_ll_x + 1, sv_ll_y);
      lp.restartTask();
      li.anyPointLocalization(sv_ll_x + 1, sv_ll_y);
      lp.killTask();
      travelTo(sv_ll_x + 1, sv_ll_y);
    } else if (sh_ll_x > sh_ur_x && sh_ll_y < sh_ur_y) {
      travelTo(sv_ll_x, sv_ll_y - 1);
      lp.restartTask();
      li.anyPointLocalization(sv_ll_x, sv_ll_y - 1);
      lp.killTask();
      travelTo(sv_ll_x, sv_ll_y - 1);
    }
  }
}
