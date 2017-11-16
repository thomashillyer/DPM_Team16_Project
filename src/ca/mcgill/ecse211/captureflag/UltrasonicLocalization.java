package ca.mcgill.ecse211.captureflag;

import ca.mcgill.ecse211.captureflag.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The UltrasonicLocalization class is used to orient the robot to 0 degrees. It
 * is used only in the beginning of the competition, when the robot is still in
 * the corner.
 */

public class UltrasonicLocalization {

	private final int HORIZONTAL_CONST = 35;
	private final int HORIZONTAL_MARGIN = 2;

	private static double alphaAngle = 0.0;
	private static double betaAngle = 0.0;
	private static double deltaTheta = 0.0;

	private int distanceUS = 255;
	private int previousDistance;
	private int filterControl;
	private static final int FILTER_OUT = 10;
	private int counter = 0;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private Object lock = new Object();

	private volatile boolean isLocalizing = false;

	private boolean fallingEdgeDetected = false;

	/**
	 * The constructor for the UltrasonicLocalization class
	 * 
	 * @param leftMotor
	 *            An instance of the EV3LargeRegulatedMotor that controls the left
	 *            motor.
	 * @param rightMotor
	 *            An instance of the EV3LargeRegulatedMotor that controls the right
	 *            motor.
	 * @param odometer
	 *            An instance of the Odometer class.
	 */

	public UltrasonicLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;

	}
	
//	public void risingEdge() {
//	  synchronized (lock) {
//        isLocalizing = true;
//    }
//
//    leftMotor.stop();
//    leftMotor.setAcceleration(CaptureFlag.ACCELERATION);
//    rightMotor.stop();
//    rightMotor.setAcceleration(CaptureFlag.ACCELERATION);
//
//    leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
//    rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
//
//
//      //rotate robot in a clock-wise fashion
//      leftMotor.forward();
//      rightMotor.backward();
//
//      //Case 1: robot starts facing towards from wall
//      if(usPoll.readUSDistance() <= NOISE_MARGIN_FALLING) {  
//
//          /*
//           * enter while loop if robot has not detected both
//           * walls yet
//           */
//          while(detectBack == false || detectLeft == false) {
//
//              /*
//               * if robot detects a rising edge and the left wall has 
//               * not been detected yet, stop both motors, set current 
//               * theta as leftWallTheta and rotate robot in opposite 
//               * direction as before (counter clock-wise)
//               */
//              if(usPoll.readUSDistance() >= NOISE_MARGIN_RISING && detectLeft == false) {
//                  Sound.beep();
//                  leftMotor.stop(true);
//                  rightMotor.stop();
//                  leftWallTheta = odometer.getTheta();
//                  detectLeft = true;
//                  leftMotor.backward();
//                  rightMotor.forward();
//
//                  /* 
//                   * pause the ultrasonic poller for 2 second 
//                   * so the robot would not detect the first risingEdge
//                   * again as it is turning ccw
//                   */
//                  try {
//                      usPoll.sleep(2000);
//                  } catch (InterruptedException e) {
//                      // TODO Auto-generated catch block
//                      e.printStackTrace();
//                  }
//              }
//
//              /*
//               * if left wall has been detected and robot detects another
//               * rising edge, set current angle to be backWallTheta
//               */
//              else if(usPoll.readUSDistance() >= NOISE_MARGIN_RISING && detectLeft == true) {
//                  Sound.beep();
//                  leftMotor.stop(true);
//                  rightMotor.stop();
//                  backWallTheta = odometer.getTheta();
//                  detectBack = true;
//              }
//          }
//      }
//
//      //Case 2: robot starts facing away from the wall
//      else if(usPoll.readUSDistance() >= NOISE_MARGIN_RISING) {   
//
//          //when both walls haven't been detected, enter this loop
//          while(detectBack == false || detectLeft == false) {
//
//              /*
//               * if robot detects a fallingEdge and the back wall hasn't been detected, means
//               * the back wall is detected now.
//               * stop the motors, set current angle as backWallTheta and continue rotating 
//               * clock-wise
//               */
//              if(usPoll.readUSDistance() <= NOISE_MARGIN_FALLING && detectBack == false) {
//                  Sound.beep();
//                  leftMotor.stop(true);
//                  rightMotor.stop();
//                  backWallTheta = odometer.getTheta();
//                  detectBack = true;
//                  leftMotor.forward();
//                  rightMotor.backward();
//              }
//
//              /*
//               * if robot detects a risingEdge and back wall has been detected, means the 
//               * left wall is detected now. 
//               * stop the motors and record the current theta as leftWallTheta
//               */
//              else if(usPoll.readUSDistance() >= NOISE_MARGIN_RISING && detectBack == true) {
//                  Sound.beep();
//                  leftMotor.stop(true);
//                  rightMotor.stop();
//                  leftWallTheta = odometer.getTheta();
//                  detectLeft = true;
//              }
//
//          }
//      }
//
//      //the following 2 if statements take care of angle correction
//      if(backWallTheta < leftWallTheta) {     
//          correctedTheta = 235 - (backWallTheta + leftWallTheta)/2; 
//      }
//      else if(backWallTheta > leftWallTheta) {    
//          correctedTheta = 40 - (backWallTheta + leftWallTheta)/2;  
//      }
//
//      odometer.setTheta(odometer.getTheta() + correctedTheta); 
//
//      /*
//       * rotate robot counter clock-wise by its current angle, 
//       * so its orientation would be 0 deg in the end
//       */
//      leftMotor.rotate(-navigation.convertAngle(WHEEL_RADIUS, TRACK, odometer.getTheta()), true);
//      rightMotor.rotate(navigation.convertAngle(WHEEL_RADIUS, TRACK, odometer.getTheta()), false);
//  }

	/**
	 * The falling and rising edges are detected in this method. The rising edge is
	 * when the sensor goes from facing the wall to away from the wall and the
	 * falling edge is from away from the wall to towards the wall. If no falling
	 * edge is detected and the distance drops below the constant then a falling
	 * edge is said to have been detected. Then if a falling edge is detected and
	 * the distance rises above our horizontal constant, a rising edge is said to be
	 * detected. The angle return by the odometer is then saved. Then, the method
	 * uses both angles to determine by how much the robot should turn to make it
	 * face north (0 degrees)
	 */
	protected void localize() {
		synchronized (lock) {
			isLocalizing = true;
		}

		leftMotor.stop();
		leftMotor.setAcceleration(CaptureFlag.ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(CaptureFlag.ACCELERATION);

		leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);

		// rotate 360 degrees to scan walls
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);

		/*
		 * while robot is rotating, compare the distance returned by the ultrasonic
		 * sensor to the threshold, and if it is less than threshold, a falling edge is
		 * detected. Save the angle and set the flag to true, so when the distance gets
		 * bigger than the threshold, the program enters the second if statement and
		 * save the anlge of the rising edge.
		 */
		while ((rightMotor.isMoving() && leftMotor.isMoving())) {

			if (!fallingEdgeDetected && (this.distanceUS <= HORIZONTAL_CONST + HORIZONTAL_MARGIN)
					&& (this.distanceUS < this.previousDistance) && counter == 0) {
				alphaAngle = odometer.getTheta();
				Sound.beep();
				fallingEdgeDetected = true;
				counter++;

			} else if (fallingEdgeDetected && (this.distanceUS > HORIZONTAL_CONST - HORIZONTAL_MARGIN)
					&& (this.distanceUS > this.previousDistance) && counter == 1) {
				betaAngle = odometer.getTheta();
				Sound.beep();
				counter++;
				fallingEdgeDetected = false;
			}
		}

		// compute the odometer's angular offset
		if (alphaAngle <= betaAngle) {
			deltaTheta = 5.0 * Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		} else if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		}

//		odometer.setTheta(odometer.getTheta() + deltaTheta);
		// System.out.println(deltaTheta);
		// rotate to face 0 degrees
		leftMotor.rotate(
				-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180 / Math.PI),
				true);
		rightMotor.rotate(
				CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180 / Math.PI),
				false);
		odometer.setTheta(0);

		synchronized (lock) {
			isLocalizing = true;
		}
	}

	public int readUSDistance() {
		return this.distanceUS;
	}

	/**
	 * Filter any extraneous distances returned by the sensor
	 * 
	 * @param distance
	 *            the distance recorded by the US sensor
	 */
	public void processUSData(int distance) {
		this.previousDistance = this.distanceUS;
		if (betaAngle != 0.0) {
			distance = 255;
		} else {

			// if (distance >= 255 && filterControl < FILTER_OUT) {
			// // bad value, do not set the distance var, however do increment
			// // the
			// // filter value
			// filterControl++;
			// } else if (distance >= 255) {
			if (distance >= 255) {
				// We have repeated large values, so there must actually be
				// nothing
				// there: leave the distance alone
				this.distanceUS = 255;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				filterControl = 0;

				this.distanceUS = distance;

			}
		}
	}

	protected boolean isLocalizing() {
		boolean result;
		synchronized (lock) {
			result = this.isLocalizing;
		}
		return result;
	}
}