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
	

	// constructor
	public GameController(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor,
			EV3MediumRegulatedMotor zip, Odometer odo, LightLocalization li, UltrasonicLocalization us, LightPoller lp,
			UltrasonicPoller usPoller, Navigation nav, WifiConnection conn, FlagDetection flag, LightPoller lp_flag) {
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

	@SuppressWarnings("rawtypes")
	public void run() {
		// directly updates the variables in GameController with the values from the
		// server
		getDataFromServer();
		// --------Integration---------
		//Ultrasonic localization
		usPoller.start();
		us.localize();
		usPoller.killTask();
		//End ultrasonic localization
		//Light localization
		lp.start();
		li.cornerLocalization(corner);
		lp.killTask();
		//end light localization
		//travel to ramp corresponding to starting zone
		nav.travelTo(someX, someY);
		//end travel
		//localize at ramp
		lp.restartTask();
		li.anyPointLocalization();
		lp.killTask();
		odo.setX(someX * CaptureFlag.TILE_LENGTH);
		odo.setY(someY * CaptureFlag.TILE_LENGTH);
		//end localize at ramp
		//traverse river (either zip or bridge)
		//end traversal
		//localize
		lp.restartTask();
		li.anyPointLocalization();
		lp.killTask();
		odo.setX(someX * CaptureFlag.TILE_LENGTH);
		odo.setY(someY * CaptureFlag.TILE_LENGTH);
		//end localize
		//search for flag
		flag_zone_x = 0;
		flag_zone_y = 0;
		calculateSearchRegionPoint();
		nav.travelTo(flag_zone_x, flag_zone_y);
		flag.findFlag(); // TODO needs a lot of work
		//end search for flag
		
		// ------END Integration-------

		//
		// // Wait until user decides to end program
		// Button.waitForAnyPress();

		// nav.turnTo(1,0);
		// Button.waitForAnyPress();
		//// nav.turnTo(0);
		// //Button.waitForAnyPress();
		// nav.turnTo(1,1);
		// Button.waitForAnyPress();
		// nav.turnTo(0,1);
		// Button.waitForAnyPress();
		// nav.turnTo(1,1);
		// -----------
		lp.start();
		li.cornerLocalization(corner);
		lp.killTask();

		Button.waitForAnyPress();
		nav.travelTo(1, 1);
		lp.restartTask();
		li.anyPointLocalization();
		lp.killTask();

		odo.setX(1 * CaptureFlag.TILE_LENGTH);
		odo.setY(1 * CaptureFlag.TILE_LENGTH);

		nav.travelTo(0, 2);
		lp.restartTask();
		li.anyPointLocalization();
		lp.killTask();
		// --------------------------

		// rightMotor.forward();
		// leftMotor.forward();
		// zip.setSpeed(150);
		// zip.forward();
		// leftMotor.rotate(CaptureFlag.convertDistance( CaptureFlag.WHEEL_RADIUS, 8),
		// true);
		// rightMotor.rotate(CaptureFlag.convertDistance( CaptureFlag.WHEEL_RADIUS, 8),
		// false);
		// lp.restartTask();
		// li.anyPointLocalization();
		// lp.killTask();
		// rightMotor.endSynchronization();

	}

	/**
	 * Calculate the middle point of the search region. Depends on whether red team
	 * or green team and also the orientation of the search region.
	 */
	private void calculateSearchRegionPoint() {
		if (/* red */true) {// TODO implement check for whether on red team or green team
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
		} /* green */
		else {
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
