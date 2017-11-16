package ca.mcgill.ecse211.captureflag;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotorListener;
import lejos.robotics.SampleProvider;

/**
 * CaptureFlag is the main class that instantiates all of the individual units
 * that will be needed to operate the robot. It also has constants that are used
 * in the other classes of the project, as well as static methods. This is to
 * make sure that setting any value is consistent among all classes, and that
 * certain methods are not copied over into all of the other classes.
 */

public class CaptureFlag {
	protected static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	protected static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	protected static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lightSampler = LocalEV3.get().getPort("S2");
	//private static final Port flagLightSampler = LocalEV3.get().getPort("S3");

	public static TextLCD t = LocalEV3.get().getTextLCD();

	// Values based on 360 degree turn test
	protected static final double WHEEL_RADIUS = 2.093;
	protected static final double TRACK = 14.8;

	protected static final double PULLEY_RADIUS = 1;

	protected static final int ROTATIONSPEED = 160;

	protected static final int ACCELERATION = 500; // 1000
	protected static final int NAV_ACCELERATION = 100;
	protected static final int FORWARDSPEED = 200;

	private static final int FILTER_OUT = 23;
	private static int filterControl;

	protected static final double BOT_LENGTH = 14.3;

	protected static final double TILE_LENGTH = 30.48;

	private static final String SERVER_IP = "192.168.2.23";
	protected static final int TEAM_NUMBER = 16;
	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	private static int x0 = 0;
	private static int y0 = 0;
	private static int xC = 0;
	private static int yC = 0;
	private static int corner = 0;

	public static void main(String[] args) {
		// set up screen for use
		int option = 0;
		final TextLCD screen = LocalEV3.get().getTextLCD();

		// set up us sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from // this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are

		// set up color sensor
		SensorModes colorSamplerSensor = new EV3ColorSensor(lightSampler);
		SampleProvider colorSensorValue = colorSamplerSensor.getMode("Red");
		float[] colorSensorData = new float[colorSamplerSensor.sampleSize()];

		// set up flag color sensor
//		EV3ColorSensor flagColorSamplerSensor = new EV3ColorSensor(flagLightSampler);
//		SampleProvider flagColorSensorValue = flagColorSamplerSensor.getRGBMode();
//		float[] flagColorSensorData = new float[flagColorSensorValue.sampleSize()];

		// instantiate classes
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odoDispl = new OdometryDisplay(odometer, screen);
		Navigation nav = new Navigation(odometer, leftMotor, rightMotor);
		UltrasonicLocalization usLocal = new UltrasonicLocalization(leftMotor, rightMotor, odometer);
		FlagDetection flag = new FlagDetection(leftMotor, rightMotor, odometer, nav, 0, 0, 1);
		UltrasonicPoller usPoller = new UltrasonicPoller(usSensor, usData, usLocal, flag);
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
//		do {
//			printMenu();
//			option = Button.waitForAnyPress();
//		} while (option != Button.ID_LEFT && option != Button.ID_RIGHT); // and wait for a button press. The button

		// pass points to light localization
		// TODO the following code is vestigial, game data will be retrieved from the
		// server. Remove it to make way for getting data from server.
		int[] points = { x0, y0, xC, yC, corner };
		LightLocalization lightLocal = new LightLocalization(leftMotor, rightMotor, odometer, nav);

		LightPoller lp = new LightPoller(colorSensorValue, colorSensorData, lightLocal);
//		LightPoller lp_flag = new LightPoller(flagColorSensorValue, flagColorSensorData, flag);

		GameController gc = new GameController(rightMotor, leftMotor, sensorMotor, odometer, lightLocal, usLocal, lp,
				usPoller, nav, conn, flag);

		// TODO remove the switch case as it will not be used for the final project.
		// Could maybe stay for testing purposes.
		// switch (option) {
		// case Button.ID_LEFT:
		// odometer.start();
		// // usPoller.start();
		// odoDispl.start();
		// // screen.clear();
		// // usLocal.start();
		// // lightLocal.start();
		// // lp.start();
		// gc.start();
		// t.clear();
		// break;
		// case Button.ID_RIGHT:
		// // UltrasonicPoller usPoll = new UltrasonicPoller(usSensor, usData, usLocal);
		// // odometer.start();
		// // usPoll.start();
		// // odoDispl.start();
		// // screen.clear();
		// // usLocal.start();
		// // Button.waitForAnyPress();
		// // lightLocal.start();
		// break;
		// default:
		// System.out.println("Error - invalid button"); // None of the above - abort
		// System.exit(-1);
		// break;
		// }

		// dont start until button pressed
//		Button.waitForAnyPress();
		odometer.start();
		odoDispl.start();
		gc.start();
		t.clear();

		// escape at any point during program execution if escape button is pressed
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	/**
	 * Prints to the screen a menu for setting the travel to points for the ZipLine
	 * lab, as well as a menu for choosing either rising or falling edge
	 * localization.
	 */
	private static void printMenu() {
		boolean enter = false;
		t.clear();
		int counter = 0;

		// x0 = printXY("X0: ", 0, 0);
		// y0 = printXY("Y0: ", 0, 1);
		// xC = printXY("XC: ", 0, 2);
		// yC = printXY("YC: ", 0, 3);

		// enter = false;
		// counter = 0;
		// t.drawString("Corner: ", 0, 4);
		// while (!enter) {
		// int buttonPressed = Button.waitForAnyPress();
		// if (buttonPressed != Button.ID_ENTER) {
		// if (buttonPressed == Button.ID_DOWN && counter == 0) {
		// counter = 0;
		// } else if (buttonPressed == Button.ID_DOWN && counter > 0) {
		// counter--;
		// } else if (buttonPressed == Button.ID_UP && counter == 3) {
		// counter = 3;
		// } else if (buttonPressed == Button.ID_UP && counter < 3) {
		// counter++;
		// }
		// t.drawInt(counter, 8, 4);
		// } else {
		// corner = counter;
		// enter = true;
		// }
		// }
		//
		// t.clear(); // the screen at initialization
		t.drawString("left/right = start", 0, 0);
		// t.drawString("right = risingEdge", 0, 1);
	}

	/**
	 * Prints to the screen a menu for setting the travel to points for the ZipLine
	 * lab. For setting the points, the used can push the up or down buttons to
	 * change the value of the displayed value. When the user wants submit the
	 * value, the enter button should be pressed.
	 * 
	 * @param disp
	 *            The string to display to the LCD screen
	 * @param x
	 *            The x position on the screen to display the string
	 * @param y
	 *            The y position on the screen to display the string
	 * @return An integer that represents what the user has chosen for the displayed
	 *         value.
	 */
	public static int printXY(String disp, int x, int y) {
		boolean enter = false;
		int counter = 0;
		t.drawString(disp, x, y);
		while (!enter) {
			int buttonPressed = Button.waitForAnyPress();
			if (buttonPressed != Button.ID_ENTER) {
				if (buttonPressed == Button.ID_DOWN && counter == 0) {
					counter = 0;
				} else if (buttonPressed == Button.ID_DOWN && counter > 0) {
					counter--;
				} else if (buttonPressed == Button.ID_UP && counter == 8) {
					counter = 8;
				} else if (buttonPressed == Button.ID_UP && counter < 8) {
					counter++;
				}
				t.drawInt(counter, 3, y);
			} else {
				enter = true;
				break;
			}
		}
		return counter;
	}

	/**
	 * 
	 * @param distance
	 *            A distance collected from sampling the ultrasonic sensor.
	 * @return A filtered value
	 */
	protected static int filter(int distance) {
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
			return distance;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			return distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			return distance;
		}
	}

	/**
	 * This method converts a distance into a number of rotations the motor should
	 * complete to go the distance.
	 * 
	 * @param radius
	 *            The radius of the robots wheel.
	 * @param distance
	 *            The desired distance that the robot should travel.
	 * @return An integer representing the amount of rotations the robot should turn
	 *         to go the given distance.
	 */
	protected static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method converts a given angle into a number of rotations the motor
	 * should complete to rotate the angle. Uses the convertDistance method.
	 * 
	 * @param radius
	 *            The radius of the robots wheel.
	 * @param TRACK
	 *            The track length, in cm, of the robot.
	 * @param angle
	 *            The angle that should be turned, in degrees.
	 * @return An integer representing the amount of rotations the robot should turn
	 *         to go turn the given angle.
	 */
	protected static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

}
