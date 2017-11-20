//////////////////////////////////////////////////////////
//Image Width(pixel)	: 320
//Image Height(pixel)	: 240
//Focal Length(mm)		:2,1
//Sensor (inch) 		:1/3
//Vanishing Point(pixel): 120
//Camera Height(cm)		:23
//////////////////////////////////////////////////////////

//#define VEHICLE_A
#define VEHICLE_B




//攝影水平偏移校正設定
//640*480 no fischeye
#define CAMERA_HORIZONTAL_SHIFT	0
#define S_ev            280
#define S_eu            280
#define S_evu           1.000000
#define S_euv           1.000000
#define S_HOCam         21.5
#define S_PixelArea     129430

#define S_IMGW		640
#define S_IMGH		480
#define S_IMGCW		320

#ifdef VEHICLE_A
#define S_IMGCH		239             //car A
#else
#define S_IMGCH		242            //car B
#endif

#define S_IMGLB		18
#define S_IMGRB		622
#define S_IMGTB		468
#define S_IMGBB		12

#define L_IB_TB_SearchLane                  180//210
#define L_IB_BB_SearchLane                  130
#define L_IB_TB_TrackingLane                205//220//210
#define L_IB_BB_TrackingLane                L_IB_BB_SearchLane
#define L_IB_TB_DrawLane                    180//168
#define L_IB_ThChangeOrder                  40//30//60//120
#define L_IP_MaintainInCtn_LossOneSide		10//20
#define L_IP_DetectedRowToUpdateLW          5//10
#define L_IP_DetectedRowSetROIwithLM		10//20
#define L_IP_DetectedRowPromoteToCtn		15//30
#define L_IP_LinePixelPromoteToCtn          3//10
#define O_IA_FewTotal                       400
#define P_IS_DoShadowTh                     24
#define O_IB_BB_SearchVehicle               L_IB_BB_SearchLane
#define O_IB_TB_SearchVehicle               204

#ifdef VEHICLE_A
#define EV_ROAD_SLOPE           0.87568     //640*480 no fischeye "Car A"
#else
//#define EV_ROAD_SLOPE           1.66725     //640*480 no fischeye "Car B"
#define EV_ROAD_SLOPE           0.758429     //640*480 no fischeye "Car B"
#endif







//640*480 Fischeye
//#define CAMERA_HORIZONTAL_SHIFT		0
//#define S_ev            135
//#define S_eu            135
//#define S_evu           (S_ev / S_eu)
//#define S_euv           (S_eu / S_ev)
//#define S_HOCam         21.5
//#define S_PixelArea     129430

//#define S_IMGW		640
//#define S_IMGH		480
//#define S_IMGCW		320
//#define S_IMGCH		237        //Car B
////#define S_IMGCH		235        //Car A
//#define S_IMGLB		18
//#define S_IMGRB		622
//#define S_IMGTB		468
//#define S_IMGBB		12

//#define L_IB_TB_SearchLane                  220
//#define L_IB_BB_SearchLane                  175 //130
//#define L_IB_TB_TrackingLane                230
//#define L_IB_BB_TrackingLane                L_IB_BB_SearchLane  //130
//#define L_IB_TB_DrawLane                    210//168
//#define L_IB_ThChangeOrder                  60//120
//#define L_IP_MaintainInCtn_LossOneSide		10//20
//#define L_IP_DetectedRowToUpdateLW          5//10
//#define L_IP_DetectedRowSetROIwithLM		10//20
//#define L_IP_DetectedRowPromoteToCtn		15//30
//#define L_IP_LinePixelPromoteToCtn          5//10
//#define O_IA_FewTotal                       400
//#define P_IS_DoShadowTh                     24
//#define O_IB_BB_SearchVehicle               L_IB_BB_SearchLane
//#define O_IB_TB_SearchVehicle               240

//#define EV_ROAD_SLOPE 0.564366




/*#if (PEC_LENS_DEFINE == PEC_TRUCK_8MM_033INCH )
	#define S_ev 960
	#define S_eu 880
	#define S_evu 1.0909 						//S_ev/S_eu
	#define S_euv 0.9166 						//S_eu/S_ev
	#define S_HOCam 278 						//Height Of Camera
	#define S_PixelArea 65289523200				//S_HOCam*S_HOCam*S_ev*S_evu
        //////////////////////////影架設參數調整類///////////////////////////////
	#define S_IMGW 352
	#define S_IMGH 288
	#define S_IMGCW 176							//(S_IMGW/2)
	#define S_IMGCH 187							//Default vanishing point for tilt angle 2.6
	#define S_IMGLB 6
	#define S_IMGRB 341
	#define S_IMGTB 286
	#define S_IMGBB 2
        //////////////////////////依影像大小調整類/////////////////////////////////
	#define L_IB_TB_SearchLane 131				//(short)(S_IMGCH*0.7)
	#define L_IB_BB_SearchLane 2				//(short)(S_IMGBB)
	#define L_IB_TB_TrackingLane 168			//(short)(S_IMGCH*0.9)
	#define L_IB_BB_TrackingLane 2				//(short)(S_IMGBB)
	#define L_IB_TB_DrawLane 131				//(short)(S_IMGCH*0.7)
	#define L_IB_ThChangeOrder 93				//(S_IMGCH*0.5)
	#define L_IP_MaintainInCtn_LossOneSide 12	//(short)(10*S_FrameRatioH)
	#define L_IP_DetectedRowToUpdateLW 6		//(short)(5*S_FrameRatioH)
	#define L_IP_DetectedRowSetROIwithLM 12		//(short)(10*S_FrameRatioH)
	#define L_IP_DetectedRowPromoteToCtn 18		//(short)(15*S_FrameRatioH)
	#define L_IP_LinePixelPromoteToCtn 6		//(short)(5*S_FrameRatioH)
	#define O_IA_FewTotal 100					//(short)(100*S_FrameRatioH*S_FrameRatioW)
	#define P_IS_DoShadowTh 18					//(short)(S_IMGCH/10)
	#define O_IB_BB_SearchVehicle L_IB_BB_TrackingLane//(short)(S_IMGCH*0.10)  //L_IB_BB_TrackingLane
	#define O_IB_TB_SearchVehicle L_IB_TB_TrackingLane//(short)(S_IMGCH*0.85)  //L_IB_TB_TrackingLane
#endif  */

#define FPS30 1
#define FPS15 2
#define S_FrameRate FPS15
//符號定義類
#define L_FinLan 1
#define L_RtSd 1
#define L_LtSd -1
//道路模型類
//#define L_MP_bDefault 0
//#define L_MP_mDefault 0//0.1
//#define L_MP_kDefault -0.0043//1/60000.0
//真實世界類
#define L_RW_ROILRegion 15    //20cm serach Roi  由總寬度最外邊往內多少距離
#define L_RW_ROILW 50         //50cm insagesam ROI 搜尋總寬度，由中心線算起就是各半
#define L_RW_DefaultLW 44
#define L_RW_DefaultMarkW 3		//20090630 為了大點點
#define L_RW_MaxMarkW 4//4
#define L_RW_MinMarkW 1//1				//20090813
#define L_RW_MaxLWInLossOneSide 75   //650
#define L_RW_MinLWInLossOneSide 30   //270
#define L_RW_MinLWInSig 30//35           //220
#define L_RW_MaxLWInSig 60           //550
#define L_RW_MinLWInCtn 30//35           //220
#define L_RW_MaxLWInCtn 60           //550
//#define L_RW_DiffFromAvgW 40
#define L_RT_RoadSlope 0.0524
#define L_RR_WLaneDeparture 0.3

//創盟//創盟//創盟//創盟//創盟//創盟//創盟//創盟//創盟
#define L_TH_DiffGrayIMS	20 //20
#define L_TH_DiffMeanIMS	23 //23
#define L_TH_VE			60//60


//依張數決定類
#define L_FC_MaxLaneStableNumber 15
#define L_FC_LossOneLaneInCtn 20//10
#define L_FC_LossBothLaneInCtn 10//5
#define L_FC_GoingStable 5
#define L_FC_InStable 10
//權重類
#define L_PW_FindOneSide 1
#define L_PW_FindBothSide 5
#define L_PW_SmoothLaneWidth 6.0
#define L_PW_ReserveLaneModel 1.8//3.8//1.8


//==========================================================================================
//車輛偵測
#define O_TH_VE 60//30
#define O_TH_HE 60//60
//#define O_TH_VE2 30//20
//#define O_TH_HE2 40//50

//******************無關像大小類******************
//依張數決定類

#define O_FC_GoingStable 5
#define O_FC_InDeadZone 3
#define O_FC_BigBlackOK 4				//4/S_FrameRate
#define O_FC_UnknownObjOK 6				//6/S_FrameRate
#define O_FC_MaxCountFuzzyArray 20
#define O_FC_MaxVihecleStableNumber 10		//20/S_FrameRate
#define O_FC_MaxToSearchUp 9		//18/S_FrameRate
#define O_FC_MinToSearchUp 5		//10/S_FrameRate
#define O_FC_ToSearchDown 7		//14/S_FrameRate

//門檻值類
#define O_MAX_DISTANCE 3200
#define O_TH_PVE_First 80
#define O_TH_PVE_Interval 30
#define O_TH_ModifyVE_FirstTH O_TH_PVE_First	//O_TH_PVE_First
#define O_TH_ModifyVE_SecondTH 50 	//(O_TH_PVE_First-O_TH_PVE_Interval)
#define O_TH_ModifyVE_ThirdTH 20	//(O_TH_PVE_First-2*O_TH_PVE_Interval)
//#define O_TH_LightGrayLevel 210//230
//#define O_TH_MinShadowGrayLevel 40
//真實世界類
#define O_RW_JumpLaneMark 20
#define O_RW_DefaultRoadWidth 44
#define O_RW_ShadowGroupEDC 100
#define O_RW_MaxCarWidth 330//330
#define O_RW_MinCarWidth 150//130
#define O_RL_MaxDiffDistance 350
#define O_RL_RegionToCheckBSD 800
#define O_RL_RegionToDoGLPDF 1000
#define O_RH_CarEdgeToGround 30
#define O_RH_DefaultCarHeigth 150
#define O_RD_MinDistanceOfAlgorithm BIAS_FORWARD_EXTEND//800
#define O_RD_ToCheckBottomEdge 2000
#define O_RD_SetMore25W 2000
#define O_RD_SetMore12W 4500
#define O_RD_CanEnhanceBoundry 5000
#define O_RD_MinDistanceToDoGLPDF 1500
#define O_RR_MaxTrackRegion 1.2
#define O_RR_MinTrackRegion 0.8
//影像值類

#define O_ID_DeadZoneEdge 15
//#define O_ID_Area_MakeSureMode 80
//#define O_ID_VFill_MakeSureMode 80
//#define O_ID_HFill_MakeSureMode 80

#define O_ID_Area_TrackingRegion 80
#define O_ID_VFill_TrackingRegion 100
#define O_ID_HFill_TrackingRegion 80

#define O_ID_Area_SearchUp 80
#define O_ID_VFill_SearchUp 60
#define O_ID_HFill_SearchUp 80

#define O_ID_Area_SearchDown 80
#define O_ID_VFill_SearchDown 50
#define O_ID_HFill_SearchDown 80
#define O_ID_BlockOfBSD 70
#define O_ID_RoadHEVE 45
#define O_ID_RoadVE 8
#define O_IW_EDC_SingalGroup 15
//程式加速類
#define P_PC_ProgramMaxEffor 15

//L_UsingFunction.c
#define VNumberSet 15
#define NumberBlockSize 8
#define NumberDrawSize (NumberBlockSize*1)
#define NumberWidth (NumberBlockSize*3)
#define NumberHeight (NumberBlockSize*5)
#define NumberSeperate 1
//定義數值量
#define N_MaxPairGroup 10
#define N_MaxSingleGroup 20
#define N_CarType 9
#define N_FeatureInPairEdge 6
#define N_AllFeature 10

//==========================================================================================
//HorizontalMarkDetection
#define O_Mark_Def_H            5//10 no fischeye
#define O_Mark_Min_H            4 //2  no fischeye
#define O_Mark_Max_H            8//10 no fischeye
#define O_Min_Mark_W            30
#define O_Not_Continue_TH       5
#define O_Max_H_Mark_counter    10



