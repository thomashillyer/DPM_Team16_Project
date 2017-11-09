package ca.mcgill.ecse211.captureflag;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * This class will implement the overall game logic that will follow according to our software flowchart
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
	private WifiConnection conn;
	private Odometer odo;
	
//	private EV3LargeRegulatedMotor[] syncList = new EV3LargeRegulatedMotor[1];

	private FlagDetection flag;

	
	//all game variables
	int redTeam;   //team number starting from red
	int greenTeam; //team number starting from green 
	int redCorner; //red team starting corner
	int greenCorner; //green team starting corner
	int greenOpFlag; //green opponent flag color
	int redOpFlag; //red opponent flag color
	int red_ll_x;  //lower left hand corner of red zone x
	int red_ll_y;  //lower left hand corner of red zone y
	int red_ur_x;  //upper right hand corner of red zone x
	int red_ur_y;  //upper right hand corner of red zone y
	int green_ll_x; //lower left hand corner of green zone x
	int green_ll_y; //lower left hand corner of green zone y
	int green_ur_x; //upper right hand corner of green zone x
	int green_ur_y; //upper right hand corner of green zone y
	int zc_r_x; //end point corresponding to zip line in red zone x
	int zc_r_y; //end point corresponding to zip line in red zone y
	int zo_r_x; //end point together with ZC_R indicates direction of zip line x
	int zo_r_y; //end point together with ZC_R indicates direction of zip line y
	int zc_g_x; //end point corresponding to zip line in green zone x
    int zc_g_y; //end point corresponding to zip line in green zone y
    int zo_g_x; //end point together with ZC_G indicates direction of zip line x
    int zo_g_y; //end point together with ZC_G indicates direction of zip line y
    int sh_ll_x; //lower left hand corner of horizontal shallow water zone x
    int sh_ll_y; //lower left hand corner of horizontal shallow water zone y
    int sh_ur_x; //upper right hand corner of horizontal shallow water zone x
    int sh_ur_y; //upper right hand corner of horizontal shallow water zone y
    int sv_ll_x; //lower left hand corner of vertical shallow water zone x
    int sv_ll_y; //lower left hand corner of vertical shallow water zone y
    int sv_ur_x; //upper right hand corner of vertical shallow water zone x
    int sv_ur_y; //upper right hand corner of vertical shallow water zone y
    int sr_ll_x; //lower left hand corner of search region in red player zone x
    int sr_ll_y; //lower left hand corner of search region in red player zone y
    int sr_ur_x; //upper right hand corner of search region in red player zone x
    int sr_ur_y; //upper right hand corner of search region in red player zone y
    int sg_ll_x; //lower left hand corner of search region in green player zone x
    int sg_ll_y; //lower left hand corner of search region in green player zone y
    int sg_ur_x; //upper right hand corner of search region in green player zone x
    int sg_ur_y; //upper right hand corner of search region in green player zone y
    
    
	//constructor
	public GameController(EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, EV3MediumRegulatedMotor zip, Odometer odo, LightLocalization li, 
	    UltrasonicLocalization us, LightPoller lp, UltrasonicPoller usPoller, Navigation nav, WifiConnection conn,FlagDetection flag) {
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

	  //obtain all game parameters from server and assign them to the appropriate variable
//	  try {
//	    Map data = conn.getData();
//	    
//	    redTeam = ((Long) data.get("RedTeam")).intValue();
//	    greenTeam = ((Long) data.get("GreenTeam")).intValue();
//	    redCorner = ((Long) data.get("RedCorner")).intValue();
//	    greenCorner = ((Long) data.get("GreenCorner")).intValue();
//	    greenOpFlag = ((Long) data.get("OG")).intValue(); 
//	    redOpFlag = ((Long) data.get("OR")).intValue();
//	    red_ll_x = ((Long) data.get("Red_LL_x")).intValue(); 
//	    red_ll_y = ((Long) data.get("Red_LL_y")).intValue();
//	    red_ur_x = ((Long) data.get("Red_UR_x")).intValue(); 
//	    red_ur_y = ((Long) data.get("Red_UR_y")).intValue();
//	    green_ll_x = ((Long) data.get("Green_LL_x")).intValue();
//	    green_ll_y = ((Long) data.get("Green_LL_y")).intValue();
//	    green_ur_x = ((Long) data.get("Green_LL_x")).intValue(); 
//	    green_ur_y = ((Long) data.get("Green_LL_y")).intValue(); 
//	    zc_r_x = ((Long) data.get("ZC_R_x")).intValue();
//	    zc_r_y = ((Long) data.get("ZC_R_y")).intValue();
//	    zo_r_x = ((Long) data.get("ZO_R_x")).intValue();
//	    zo_r_y = ((Long) data.get("ZO_R_y")).intValue();
//	    zc_g_x = ((Long) data.get("ZC_G_x")).intValue(); 
//	    zc_g_y = ((Long) data.get("ZC_G_y")).intValue(); 
//	    zo_g_x = ((Long) data.get("ZO_G_x")).intValue();
//	    zo_g_y = ((Long) data.get("ZO_G_y")).intValue();
//         sh_ll_x = ((Long) data.get("SH_LL_x")).intValue();
//	     sh_ll_y = ((Long) data.get("SH_LL_y")).intValue();
//	     sh_ur_x = ((Long) data.get("SH_UR_x")).intValue();
//	     sh_ur_y = ((Long) data.get("SH_UR_y")).intValue();
//	     sv_ll_x = ((Long) data.get("SV_LL_x")).intValue();
//	     sv_ll_y = ((Long) data.get("SV_LL_y")).intValue();
//	     sv_ur_x = ((Long) data.get("SV_UR_x")).intValue();
//	     sv_ur_y = ((Long) data.get("SV_UR_y")).intValue();
//	     sr_ll_x = ((Long) data.get("SR_LL_x")).intValue();
//	     sr_ll_y = ((Long) data.get("SR_LL_y")).intValue();
//	     sr_ur_x = ((Long) data.get("SR_UR_x")).intValue(); 
//	     sr_ur_y = ((Long) data.get("SR_UR_y")).intValue();
//	     sg_ll_x = ((Long) data.get("SG_LL_x")).intValue();
//	     sg_ll_y = ((Long) data.get("SG_LL_y")).intValue();
//	     sg_ur_x = ((Long) data.get("SG_UR_x")).intValue();
//	     sg_ur_y = ((Long) data.get("SG_UR_y")).intValue();
//	  }catch (Exception e) {
//	      System.err.println("Error: " + e.getMessage());
//	    }

//
//	    // Wait until user decides to end program
//	    Button.waitForAnyPress();
	  
	  
//	  nav.turnTo(1,0);
//	  Button.waitForAnyPress();
////	  nav.turnTo(0);
//	  //Button.waitForAnyPress();
//	  nav.turnTo(1,1);
//	  Button.waitForAnyPress();
//	  nav.turnTo(0,1);
//	  Button.waitForAnyPress();
//	  nav.turnTo(1,1);
//-----------
		lp.start();
		li.cornerLocalization();
		Button.waitForAnyPress();
		lp.killTask();

		nav.travelTo(1,1);
		lp.restartTask();
        li.anyPointLocalization();
        lp.killTask();
        
        odo.setX(1*CaptureFlag.TILE_LENGTH);
        odo.setY(1*CaptureFlag.TILE_LENGTH);
        
		nav.travelTo(0, 2);
		  lp.restartTask();
	        li.anyPointLocalization();
	        lp.killTask();
//--------------------------
	        
	        
	        
//    	rightMotor.forward();
//    	leftMotor.forward();
//    	zip.setSpeed(150);
//    	zip.forward();
//		leftMotor.rotate(CaptureFlag.convertDistance( CaptureFlag.WHEEL_RADIUS, 8), true);
//	    rightMotor.rotate(CaptureFlag.convertDistance( CaptureFlag.WHEEL_RADIUS, 8), false);
//		lp.restartTask();
//		li.anyPointLocalization();
//		lp.killTask();
//		rightMotor.endSynchronization();		
	}
	
}
