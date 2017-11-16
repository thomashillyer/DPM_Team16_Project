package ca.mcgill.ecse211.captureflag;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * This class will implement the overall game logic that will follow according
 * to our software flowchart
 */
public class GameController extends Thread {

	protected static EV3LargeRegulatedMotor leftMotor;
	protected static EV3LargeRegulatedMotor rightMotor;
	private static EV3MediumRegulatedMotor zip;

	private LightLocalization li;
	private UltrasonicLocalization us;
	private LightPoller lp;
	private UltrasonicPoller usPoller;
	private Navigation nav;
	private static WifiConnection conn;
	private Odometer odo;

	// private EV3LargeRegulatedMotor[] syncList = new EV3LargeRegulatedMotor[1];

	private FlagDetection flag;
	private LightPoller lp_flag;

	// all game variables
	static int redTeam; // team number starting from red
	static int greenTeam; // team number starting from green
	static int redCorner; // red team starting corner
	static int greenCorner; // green team starting corner
	static int greenOpFlag; // green opponent flag color
	static int redOpFlag; // red opponent flag color
	static int red_ll_x; // lower left hand corner of red zone x
	static int red_ll_y; // lower left hand corner of red zone y
	static int red_ur_x; // upper right hand corner of red zone x
	static int red_ur_y; // upper right hand corner of red zone y
	static int green_ll_x; // lower left hand corner of green zone x
	static int green_ll_y; // lower left hand corner of green zone y
	static int green_ur_x; // upper right hand corner of green zone x
	static int green_ur_y; // upper right hand corner of green zone y
	static int zc_r_x; // end point corresponding to zip line in red zone x
	static int zc_r_y; // end point corresponding to zip line in red zone y
	static int zo_r_x; // end point together with ZC_R indicates direction of zip line x
	static int zo_r_y; // end point together with ZC_R indicates direction of zip line y
	static int zc_g_x; // end point corresponding to zip line in green zone x
	static int zc_g_y; // end point corresponding to zip line in green zone y
	static int zo_g_x; // end point together with ZC_G indicates direction of zip line x
	static int zo_g_y; // end point together with ZC_G indicates direction of zip line y

	static int sh_ll_x; // lower left hand corner of horizontal shallow water zone x
	static int sh_ll_y; // lower left hand corner of horizontal shallow water zone y
	static int sh_ur_x; // upper right hand corner of horizontal shallow water zone x
	static int sh_ur_y; // upper right hand corner of horizontal shallow water zone y
	static int sv_ll_x; // lower left hand corner of vertical shallow water zone x
	static int sv_ll_y; // lower left hand corner of vertical shallow water zone y
	static int sv_ur_x; // upper right hand corner of vertical shallow water zone x
	static int sv_ur_y; // upper right hand corner of vertical shallow water zone y

	// Flag search areas
	static int sr_ll_x; // lower left hand corner of search region in red player zone x
	static int sr_ll_y; // lower left hand corner of search region in red player zone y
	static int sr_ur_x; // upper right hand corner of search region in red player zone x
	static int sr_ur_y; // upper right hand corner of search region in red player zone y
	static int sg_ll_x; // lower left hand corner of search region in green player zone x
	static int sg_ll_y; // lower left hand corner of search region in green player zone y
	static int sg_ur_x; // upper right hand corner of search region in green player zone x
	static int sg_ur_y; // upper right hand corner of search region in green player zone y

	int flag_zone_x; // calculated middle point of the search region x
	int flag_zone_y; // calculated middle point of the search region y

	boolean assignedGreen = true;

	/**
	 * Constructor for the game controller, the class that runs all game logic.
	 * 
	 * @param rightMotor
	 *            Right motor object
	 * @param leftMotor
	 *            Left motor object
	 * @param zip
	 *            Top (pulley) motor object
	 * @param odo
	 *            Odometer object, keeps track of motor rotations
	 * @param li
	 *            Light localization object, used to determine accurate location
	 *            based on grid lines
	 * @param us
	 *            Ultrasonic localization object, used to roughly orient the robot
	 *            for initial light localization based on the corner walls
	 * @param lp
	 *            Light poller object used to get data from the rear downward-facing
	 *            light sensor
	 * @param usPoller
	 *            Ultrasonic poller object, used to get data from the front
	 *            ultrasonic sensor
	 * @param nav
	 *            Navigation object, used to turn and navigate to points
	 * @param conn
	 *            Wifi connection object, used to get the data from the server
	 * @param flag
	 *            Flag detection object, used to search the region that holds the
	 *            flag and find the correct flag
	 * @param lp_flag
	 *            Light poller object used to get data from the front-facing light
	 *            sensor
	 */
	public GameController(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor,
			EV3MediumRegulatedMotor zip, Odometer odo, LightLocalization li, UltrasonicLocalization us, LightPoller lp,
			UltrasonicPoller usPoller, Navigation nav, WifiConnection conn, FlagDetection flag) {
		this.li = li;
		this.us = us;
		this.lp = lp;
		this.usPoller = usPoller;
		this.nav = nav;
		this.conn = conn;

		this.zip = zip;
		this.odo = odo;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.flag = flag;
		this.lp_flag = lp_flag;
	}

	/**
	 * This method is the controller for all game logic. It gets data from the
	 * server and then runs through a set of tasks, the order of which depends on
	 * whether the robot is assigned to the red or green team.
	 */
	@SuppressWarnings("rawtypes")
	public void run() {
		// --------Integration---------
		// directly updates the variables in GameController with the values from the
		// server
		getDataFromServer();

		// calculate length of zipline
		double zip_x = Math.abs(zc_g_x - zc_r_x) * CaptureFlag.TILE_LENGTH;
		double zip_y = Math.abs(zc_g_y - zc_r_y) * CaptureFlag.TILE_LENGTH;
		// 1.3 is a safety factor
		double zipLength = 1.3 * Math.sqrt(Math.pow(zip_x, 2) + Math.pow(zip_y, 2));

		flag_zone_x = 0;
		flag_zone_y = 0;

		// Ultrasonic localization
		 usPoller.start();
		 us.localize();
		 usPoller.killTask();
		// End ultrasonic localization

		// check what team you have been assigned to
		if (greenTeam == CaptureFlag.TEAM_NUMBER) {
			System.out.println("green team corner " + greenCorner);
			assignedGreen = true;
			// Light localization
			lp.start();
			li.cornerLocalization(greenCorner);
			lp.killTask();
			
			Button.waitForAnyPress();
			// end light localization
//			Button.waitForAnyPress();

			// green uses zipline and returns on bridge

			// travel to point before zipline
			// zc_g is zipline start
			// zo_g is point before
			// zc_r is zipline end
			// zo_r is point after
			nav.travelTo(zo_g_x, zo_g_y);

			// anypoint light localize
			lp.restartTask();
			li.do_localization(zo_g_x, zo_g_y); // dont pass a point because it assumes its close to the point it should
			lp.killTask();
			nav.travelTo(zo_g_x, zo_g_y);
			// set x and y to point before the ramp
			// odo.setX(zo_g_x * CaptureFlag.TILE_LENGTH);
			// odo.setY(zo_g_y * CaptureFlag.TILE_LENGTH);
			// end anypoint light localize

			// traverse zipline
			// travel to start of zipline
			zip.setSpeed(400);

			zip.backward();
			// zip.rotate(-CaptureFlag.convertDistance(CaptureFlag.PULLEY_RADIUS, 12 *
			// CaptureFlag.TILE_LENGTH), true);
			nav.travelTo(zc_g_x, zc_g_y);
//			leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, -5), true);
//			rightMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 5), false);
			// nav.turnTo(zc_g_x, zc_g_y);

			leftMotor.forward();
			rightMotor.forward();
			long milli = System.currentTimeMillis();
			while (System.currentTimeMillis() - milli < 10000)
				;

			leftMotor.stop(true);
			rightMotor.stop(true);
			milli = System.currentTimeMillis();
			while (System.currentTimeMillis() - milli < 10000);
			odo.setX(zc_r_x);
			odo.setY(zc_r_y);
			nav.travelTo(zo_r_x, zo_r_y);
			
//			zip.stop();

			// should now be about at end of zipline
			// drive forward a block length
			// localize assuming at zo_r
			
			
			lp.restartTask();
			
//			li.afterZipLine();
			
			li.do_localization(zo_r_x, zo_r_y);
			lp.killTask();
			nav.travelTo(zo_r_x, zo_r_y);
//			Button.waitForAnyPress();

			// travel to flag region
			nav.travelTo(sr_ll_x, sr_ll_y);
//			Button.waitForAnyPress();

			// robot is now on the ground at the end of the zipline
			// drive to approximately zo-r
			// leftMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS,
			// CaptureFlag.TILE_LENGTH), true);
			// rightMotor.rotate(CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS,
			// CaptureFlag.TILE_LENGTH), false);
			//
			// // localize at point after zipline
			// lp.restartTask();
			// li.do_localization(zo_r_x, zo_r_y);
			// lp.killTask();
			// // odo.setX(zo_r_x * CaptureFlag.TILE_LENGTH);
			// // odo.setY(zo_r_y * CaptureFlag.TILE_LENGTH);
			// // end localize at point after zipline
			//
			// // navigate to flag region
			// calculateSearchRegionPoint();
			// nav.travelTo(flag_zone_x, flag_zone_y);
			// search for flag - not needed for beta demo
			// flag.findFlag(); // TODO needs a lot of work
			// end search for flag

			// TODO after beta add logic for bridge traversal etc

		} else if (redTeam == CaptureFlag.TEAM_NUMBER) {
			assignedGreen = false;

			// Light localization
			lp.start();
			li.cornerLocalization(redCorner);
			lp.killTask();
			// end light localization

			// red uses bridge and returns over water

			// TODO the rest

		}
		// ------END Integration-------
		/* eric code */
		//
		//
		// lp.start();
		// li.cornerLocalization(0);
		// lp.killTask();
		//
		// nav.travelTo(2, 1);
		////
		//// Button.waitForAnyPress();
		// lp.restartTask();
		// li.do_localization(2, 1);
		// lp.killTask();
		// //Button.waitForAnyPress();
		// nav.travelTo(2, 1);
		//
		//
		// nav.travelTo(2, 2);
		//
		// lp.killTask();
		//
		// leftMotor.forward();
		// rightMotor.forward();
		// zip.setSpeed(150);
		// zip.backward();
		// //time
		// lp.restartTask();
		// li.afterZipLine();
		//
		//
		//
		//
		//// nav.turn2(45,true);
		//
		//// lp.restartTask();
		//// li.do_localization(2, 2);
		//// lp.killTask();
		////
		//// nav.travelTo(2, 2);
		////
		////
		//// nav.travelTo(3, 1);
		//
		//// nav.turn2(45,true);
		//
		//// lp.restartTask();
		//// li.do_localization(3, 1);
		//// lp.killTask();
		////
		//// nav.travelTo(3, 1);
		//// nav.turn2(45,true);
		//// lp.restartTask();
		//// li.anyPointLocalization();
		//// lp.killTask();
	}

	/**
	 * Calculate the middle point of the search region. Depends on whether red team
	 * or green team and also the orientation of the search region.
	 */
	private void calculateSearchRegionPoint() {
		/* green */
		if (assignedGreen) {
			// the search region is long in the x direction
			if (Math.abs(sg_ll_x - sg_ur_x) == 2) {
				flag_zone_x = (sg_ll_x + sg_ur_x) / 2;

				// y is less than 6, search region is in the lower half of the board
				// TODO may have weird effects if the search region is along the center axis of
				// the board
				if (sg_ll_y < 6 || sg_ur_y < 6) {
					flag_zone_y = Math.max(sg_ll_y, sg_ur_y);
				} else { // search region is in upper half of board
					flag_zone_y = Math.min(sg_ll_y, sg_ur_y);
				}

			} // the search region is long in y direction
			else if (Math.abs(sg_ll_y - sg_ur_y) == 2) {
				flag_zone_y = (sg_ll_y + sg_ur_y) / 2;

				// search region is on left side of board
				// TODO may have weird effects if the search region is along the center axis of
				// the board
				if (sg_ll_x < 6 || sg_ur_x < 6) {
					flag_zone_x = Math.max(sg_ll_x, sg_ur_x);
				} else {// search region is on right side of board
					flag_zone_x = Math.min(sg_ll_x, sg_ur_x);
				}
			}
		} /* red */
		else if (!assignedGreen) {
			// the search region is long in the x direction
			if (Math.abs(sr_ll_x - sr_ur_x) == 2) {
				flag_zone_x = (sr_ll_x + sr_ur_x) / 2;

				// y is less than 6, search region is in the lower half of the board
				// TODO may have weird effects if the search region is along the center axis of
				// the board
				if (sr_ll_y < 6 || sr_ur_y < 6) {
					flag_zone_y = Math.max(sr_ll_y, sr_ur_y);
				} else { // search region is in upper half of board
					flag_zone_y = Math.min(sr_ll_y, sr_ur_y);
				}

			} // the search region is long in y direction
			else if (Math.abs(sr_ll_y - sr_ur_y) == 2) {
				flag_zone_y = (sr_ll_y + sr_ur_y) / 2;

				// search region is on left side of board
				// TODO may have weird effects if the search region is along the center axis of
				// the board
				if (sr_ll_x < 6 || sr_ur_x < 6) {
					flag_zone_x = Math.max(sr_ll_x, sr_ur_x);
				} else {// search region is on right side of board
					flag_zone_x = Math.min(sr_ll_x, sr_ur_x);
				}
			}
		}

	}

	/**
	 * Obtains all game parameters from the server and assigns them to the
	 * appropriate variable. Game logic uses these variables to navigate to
	 * waypoints.
	 */
	private void getDataFromServer() {
		try {
			Map data = conn.getData();

			redTeam = ((Long) data.get("RedTeam")).intValue();
			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			redCorner = ((Long) data.get("RedCorner")).intValue();
			greenCorner = ((Long) data.get("GreenCorner")).intValue();
			greenOpFlag = ((Long) data.get("OG")).intValue();
			redOpFlag = ((Long) data.get("OR")).intValue();
			red_ll_x = ((Long) data.get("Red_LL_x")).intValue();
			red_ll_y = ((Long) data.get("Red_LL_y")).intValue();
			red_ur_x = ((Long) data.get("Red_UR_x")).intValue();
			red_ur_y = ((Long) data.get("Red_UR_y")).intValue();
			green_ll_x = ((Long) data.get("Green_LL_x")).intValue();
			green_ll_y = ((Long) data.get("Green_LL_y")).intValue();
			green_ur_x = ((Long) data.get("Green_LL_x")).intValue();
			green_ur_y = ((Long) data.get("Green_LL_y")).intValue();
			zc_r_x = ((Long) data.get("ZC_R_x")).intValue();
			zc_r_y = ((Long) data.get("ZC_R_y")).intValue();
			zo_r_x = ((Long) data.get("ZO_R_x")).intValue();
			zo_r_y = ((Long) data.get("ZO_R_y")).intValue();
			zc_g_x = ((Long) data.get("ZC_G_x")).intValue();
			zc_g_y = ((Long) data.get("ZC_G_y")).intValue();
			zo_g_x = ((Long) data.get("ZO_G_x")).intValue();
			zo_g_y = ((Long) data.get("ZO_G_y")).intValue();
			sh_ll_x = ((Long) data.get("SH_LL_x")).intValue();
			sh_ll_y = ((Long) data.get("SH_LL_y")).intValue();
			sh_ur_x = ((Long) data.get("SH_UR_x")).intValue();
			sh_ur_y = ((Long) data.get("SH_UR_y")).intValue();
			sv_ll_x = ((Long) data.get("SV_LL_x")).intValue();
			sv_ll_y = ((Long) data.get("SV_LL_y")).intValue();
			sv_ur_x = ((Long) data.get("SV_UR_x")).intValue();
			sv_ur_y = ((Long) data.get("SV_UR_y")).intValue();
			sr_ll_x = ((Long) data.get("SR_LL_x")).intValue();
			sr_ll_y = ((Long) data.get("SR_LL_y")).intValue();
			sr_ur_x = ((Long) data.get("SR_UR_x")).intValue();
			sr_ur_y = ((Long) data.get("SR_UR_y")).intValue();
			sg_ll_x = ((Long) data.get("SG_LL_x")).intValue();
			sg_ll_y = ((Long) data.get("SG_LL_y")).intValue();
			sg_ur_x = ((Long) data.get("SG_UR_x")).intValue();
			sg_ur_y = ((Long) data.get("SG_UR_y")).intValue();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

	}

}
