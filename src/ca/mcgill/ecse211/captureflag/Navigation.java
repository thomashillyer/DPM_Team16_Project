package ca.mcgill.ecse211.captureflag;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 *The Navigation class implements the logic of moving from one point to another, using the shortest path.
 */

public class Navigation extends Thread {

	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	/**
	 * This is the constructor for the Navigation class.
	 * @param odometer An instance of the Odometer class.
	 * @param leftMotor An instance of the EV3LargeRegulatedMotor that controls the left motor.
	 * @param rightMotor An instance of the EV3LargeRegulatedMotor that controls the right motor.
	 */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	public void run() {
		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	/**
	 * This method causes the robot to travel to the absolute field location (x, y)
	 * ,specified in tile points.This method should continuously call turnTo(double
	 * theta) and then set the motor speed to forward(straight). This will make sure
	 * that your heading is updated until you reach your exact goal. This method
	 * will pool the odometer for information.
	 * 
	 * @param x
	 *            The new x destination of the robot
	 * @param y
	 *            The new y destination of the robot
	 */

	public void travelTo(double x, double y) {

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
		double deltaTheta = Math.atan2(deltaX, deltaY) - currTheta;

		// rotate the robot towards its new way point
		turnTo(deltaTheta);

		// calculate the distance to next point using the built in pythagore
		// theorem
		// TODO try Math.Pyth
		double distToTravel = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
		distToTravel = Math.sqrt(distToTravel);

		leftMotor.setAcceleration(CaptureFlag.ACCELERATION); //added
        rightMotor.setAcceleration(CaptureFlag.ACCELERATION); //added
		// travel to the next point, and don't wait until the action is
		// complete. So the boolean in both rotate method should be true
		leftMotor.setSpeed(CaptureFlag.FORWARDSPEED-1);
		rightMotor.setSpeed(CaptureFlag.FORWARDSPEED);

		rightMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, distToTravel), true);
		leftMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, distToTravel), false);

		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	/**
	 * This method takes an angle theta as input. It starts by setting the speed of
	 * both motors. The angle is between -360 and 360 degrees, but since this
	 * methods should always make the robot turn with the minimum angle possible,
	 * theta should be updated. If theta is greater than 180 degrees, instead of
	 * turning positively, the robot turns negatively by an angle of theta - 360.
	 * And if theta is less than or equal to -180, instead of turning negatively,
	 * the robot turns positevly by an angle of theta + 360.
	 * 
	 * @param theta
	 *            The angle by which the cart should turn.
	 */
	public void turnTo(double theta) {
		leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);

		// adjusting the angle in order to have an optimal turn (a turn with the
		// minimum angle)
		if (theta <= -Math.PI) {
			theta += Math.PI * 2;
		} else if (theta > Math.PI) {
			theta -= Math.PI * 2;
		}

		theta = theta * 180.0 / Math.PI;

		// turn to the left if angle is negative
		if (theta < 0) {
			leftMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, -theta+2), true);
			rightMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, -theta+2), false);
		}
		// turn to the right if angle is positive
		else {
			leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, theta-2), true);
			rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, theta-2), false);
		}

	}
}
