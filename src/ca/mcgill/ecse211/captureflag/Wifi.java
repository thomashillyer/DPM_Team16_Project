package ca.mcgill.ecse211.captureflag;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

public class Wifi {
	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 16;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	public Wifi() {
		
	}
	public void updateDataPoints() {
		// obtain all game parameters from server and assign them to the appropriate
		// variable
		try {
			Map data = GameController.conn.getData();

			GameController.redTeam = ((Long) data.get("RedTeam")).intValue();
			GameController.greenTeam = ((Long) data.get("GreenTeam")).intValue();
			GameController.redCorner = ((Long) data.get("RedCorner")).intValue();
			GameController.greenCorner = ((Long) data.get("GreenCorner")).intValue();
			GameController.greenOpFlag = ((Long) data.get("OG")).intValue();
			GameController.redOpFlag = ((Long) data.get("OR")).intValue();
			GameController.red_ll_x = ((Long) data.get("Red_LL_x")).intValue();
			GameController.red_ll_y = ((Long) data.get("Red_LL_y")).intValue();
			GameController.red_ur_x = ((Long) data.get("Red_UR_x")).intValue();
			GameController.red_ur_y = ((Long) data.get("Red_UR_y")).intValue();
			GameController.green_ll_x = ((Long) data.get("Green_LL_x")).intValue();
			GameController.green_ll_y = ((Long) data.get("Green_LL_y")).intValue();
			GameController.green_ur_x = ((Long) data.get("Green_LL_x")).intValue();
			GameController.green_ur_y = ((Long) data.get("Green_LL_y")).intValue();
			GameController.zc_r_x = ((Long) data.get("ZC_R_x")).intValue();
			GameController.zc_r_y = ((Long) data.get("ZC_R_y")).intValue();
			GameController.zo_r_x = ((Long) data.get("ZO_R_x")).intValue();
			GameController.zo_r_y = ((Long) data.get("ZO_R_y")).intValue();
			GameController.zc_g_x = ((Long) data.get("ZC_G_x")).intValue();
			GameController.zc_g_y = ((Long) data.get("ZC_G_y")).intValue();
			GameController.zo_g_x = ((Long) data.get("ZO_G_x")).intValue();
			GameController.zo_g_y = ((Long) data.get("ZO_G_y")).intValue();
			GameController.sh_ll_x = ((Long) data.get("SH_LL_x")).intValue();
			GameController.sh_ll_y = ((Long) data.get("SH_LL_y")).intValue();
			GameController.sh_ur_x = ((Long) data.get("SH_UR_x")).intValue();
			GameController.sh_ur_y = ((Long) data.get("SH_UR_y")).intValue();
			GameController.sv_ll_x = ((Long) data.get("SV_LL_x")).intValue();
			GameController.sv_ll_y = ((Long) data.get("SV_LL_y")).intValue();
			GameController.sv_ur_x = ((Long) data.get("SV_UR_x")).intValue();
			GameController.sv_ur_y = ((Long) data.get("SV_UR_y")).intValue();
			GameController.sr_ll_x = ((Long) data.get("SR_LL_x")).intValue();
			GameController.sr_ll_y = ((Long) data.get("SR_LL_y")).intValue();
			GameController.sr_ur_x = ((Long) data.get("SR_UR_x")).intValue();
			GameController.sr_ur_y = ((Long) data.get("SR_UR_y")).intValue();
			GameController.sg_ll_x = ((Long) data.get("SG_LL_x")).intValue();
			GameController.sg_ll_y = ((Long) data.get("SG_LL_y")).intValue();
			GameController.sg_ur_x = ((Long) data.get("SG_UR_x")).intValue();
			GameController.sg_ur_y = ((Long) data.get("SG_UR_y")).intValue();
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		/* OLD VERSION */
		// obtain all game parameters from server and assign them to the appropriate
		// variable
		// try {
		// Map data = conn.getData();
		//
		// redTeam = ((Long) data.get("RedTeam")).intValue();
		// greenTeam = ((Long) data.get("GreenTeam")).intValue();
		// redCorner = ((Long) data.get("RedCorner")).intValue();
		// greenCorner = ((Long) data.get("GreenCorner")).intValue();
		// greenOpFlag = ((Long) data.get("OG")).intValue();
		// redOpFlag = ((Long) data.get("OR")).intValue();
		// red_ll_x = ((Long) data.get("Red_LL_x")).intValue();
		// red_ll_y = ((Long) data.get("Red_LL_y")).intValue();
		// red_ur_x = ((Long) data.get("Red_UR_x")).intValue();
		// red_ur_y = ((Long) data.get("Red_UR_y")).intValue();
		// green_ll_x = ((Long) data.get("Green_LL_x")).intValue();
		// green_ll_y = ((Long) data.get("Green_LL_y")).intValue();
		// green_ur_x = ((Long) data.get("Green_LL_x")).intValue();
		// green_ur_y = ((Long) data.get("Green_LL_y")).intValue();
		// zc_r_x = ((Long) data.get("ZC_R_x")).intValue();
		// zc_r_y = ((Long) data.get("ZC_R_y")).intValue();
		// zo_r_x = ((Long) data.get("ZO_R_x")).intValue();
		// zo_r_y = ((Long) data.get("ZO_R_y")).intValue();
		// zc_g_x = ((Long) data.get("ZC_G_x")).intValue();
		// zc_g_y = ((Long) data.get("ZC_G_y")).intValue();
		// zo_g_x = ((Long) data.get("ZO_G_x")).intValue();
		// zo_g_y = ((Long) data.get("ZO_G_y")).intValue();
		// sh_ll_x = ((Long) data.get("SH_LL_x")).intValue();
		// sh_ll_y = ((Long) data.get("SH_LL_y")).intValue();
		// sh_ur_x = ((Long) data.get("SH_UR_x")).intValue();
		// sh_ur_y = ((Long) data.get("SH_UR_y")).intValue();
		// sv_ll_x = ((Long) data.get("SV_LL_x")).intValue();
		// sv_ll_y = ((Long) data.get("SV_LL_y")).intValue();
		// sv_ur_x = ((Long) data.get("SV_UR_x")).intValue();
		// sv_ur_y = ((Long) data.get("SV_UR_y")).intValue();
		// sr_ll_x = ((Long) data.get("SR_LL_x")).intValue();
		// sr_ll_y = ((Long) data.get("SR_LL_y")).intValue();
		// sr_ur_x = ((Long) data.get("SR_UR_x")).intValue();
		// sr_ur_y = ((Long) data.get("SR_UR_y")).intValue();
		// sg_ll_x = ((Long) data.get("SG_LL_x")).intValue();
		// sg_ll_y = ((Long) data.get("SG_LL_y")).intValue();
		// sg_ur_x = ((Long) data.get("SG_UR_x")).intValue();
		// sg_ur_y = ((Long) data.get("SG_UR_y")).intValue();
		// }catch (Exception e) {
		// System.err.println("Error: " + e.getMessage());
		// }
	}
}
