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
	static WifiConnection conn;
	private Odometer odo;
	private Wifi wifi;

	// private EV3LargeRegulatedMotor[] syncList = new EV3LargeRegulatedMotor[1];

	private FlagDetection flag;

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
	static int sr_ll_x; // lower left hand corner of search region in red player zone x
	static int sr_ll_y; // lower left hand corner of search region in red player zone y
	static int sr_ur_x; // upper right hand corner of search region in red player zone x
	static int sr_ur_y; // upper right hand corner of search region in red player zone y
	static int sg_ll_x; // lower left hand corner of search region in green player zone x
	static int sg_ll_y; // lower left hand corner of search region in green player zone y
	static int sg_ur_x; // upper right hand corner of search region in green player zone x
	static int sg_ur_y; // upper right hand corner of search region in green player zone y

	// constructor
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
	}

	@SuppressWarnings("rawtypes")
	public void run() {
		// directly updates the variables in GameController with the values from the
		// server
		wifi.updateDataPoints();

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

}
