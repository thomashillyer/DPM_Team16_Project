package ca.mcgill.ecse211.captureflag;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * This class will implement the overall game logic that will follow according to our software flowchart
 */
public class GameController extends Thread {
	
	private LightLocalization li;
	private UltrasonicLocalization us;
	private LightPoller lp;
	private UltrasonicPoller usPoller;
	private Navigation nav;
	private WifiConnection conn;
	
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
	public GameController(LightLocalization li, UltrasonicLocalization us, LightPoller lp, UltrasonicPoller usPoller, Navigation nav, WifiConnection conn) {
		this.li = li;
		this.us = us;
		this.lp = lp;
		this.usPoller = usPoller;
		this.nav = nav;
		this.conn = conn;
	}
	@SuppressWarnings("rawtypes")
	public void run() {
	  
	  //obtain all game parameters from server and assign them to the appropriate variable
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
	  }catch (Exception e) {
	      System.err.println("Error: " + e.getMessage());
	    }

	    // Wait until user decides to end program
	    Button.waitForAnyPress();
	    
	    
	    
//		usPoller.start();
//		us.localize();
//		usPoller.killTask();
//		
//		Button.waitForAnyPress();
//		
//		lp.start();
//		li.cornerLocalization();
//		lp.killTask();
		
		//nav.travelTo(1, 1);
	}
	
}
