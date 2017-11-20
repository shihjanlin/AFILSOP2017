#ifndef _INITIALVARIABLE_H_
#define _INITIALVARIABLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


//攝影機參數 includs 設定
#include "CameraEnvironment.h"


//偏移預設值值設定
#define VEHICLE_WIDTH 			20//28
#define BIAS_FORWARD_EXTEND 	40

#define TRANSFER_ERROR 0//0.5



//#define SPEED_OF_A_MOTOR_VEHICLE_ON_OFF    //車速訊號輸入開/關
#define LOW_VELOCITY_TH	15
#define HIGH_VELOCITY_TH	60

#define PARAMETER_EXTRACTION    //創盟參數抽出



//攝影機安裝功能(開/關)
//#define CAMERA_SETUP_FUNCTION_ON_OFF

#ifdef CAMERA_SETUP_FUNCTION_ON_OFF
#define CAM_SETUP_USE_LANE
//#define CAM_SETUP_USE_BALL
#endif



//車輛偵測(開/關)
#ifndef CAMERA_SETUP_FUNCTION_ON_OFF
#define OBSTACLE_DETECTION_ON_OFF
#endif

//車距資訊
#define O_WARNING_DISTANCE                    3200
#define O_DANGEROUS_DISTANCE                  1600
#define O_IS_TOO_CLOSE_STOP_LANE_DETECTION    1100



//Corner配對
#define CORNER_DIFFERENCE_ROW_THRESHOLD    5    //Corner配對列數高度差異門檻值
//#define O_DRAW_PAIR_CORNER_DEBUG              //畫出成對Corner
//#define O_DRAW_CORNER_DEBUG                   //畫出全部Corner
//#define O_DRAW_ALL_PAIR_CORNER_DEBUG          //畫出全部配對成功的Corner



//對稱性運算開關
#define VEHICLE_VERIFY_SYMMETRY_PROCESS_ON_OFF



//實虛線判斷動態門檻值(開/關)
#define SOLID_LINE_DYNAMIC_TH_ON_OFF
//#define SOLID_OR_DASHED_LINE_DEBUG			  //顯示debug數值
#define DASHED_CHANGE_SOLID_COUNT		10    //虛線轉實線frame count
#define SOLID_CHANGE_DASHED_COUNT 		 5    //實線轉虛線frame count
#define FRAME_COUNT_LIMIT               30    //frame count最大值
#define INITIAL_FRAME_COUNT             15    //frame count初始值
#define INITIAL_THRESHOLD               70    //門檻值初始值

//實虛線判斷動態門檻值參數設定
#ifdef SOLID_LINE_DYNAMIC_TH_ON_OFF
#define INITIAL_SOLID_DATA                  40    //實線初始值
#define INITIAL_DASHED_DATA                 35    //虛線初始值
#define DATA_AVERAGE                        16    //Data儲存值每XX平均一次，目前矩陣最大為16
#define DATA_AVERAGE_SHIFT                   4    //Data儲存值每XX平均一次，用位移的方式做除法以上面為標準
#define THRESHOLD_DATA_AVERAGE               4    //Data門檻儲存值每X平均一次，目前矩陣最大為4
#define THRESHOLD_DATA_AVERAGE_SHIFT         2    //Data門檻儲存值每X平均一次，用位移的方式做除法以上面為標準
#define SOLID_LINE_DYNAMIC_HIGHEST_LIMIT    100    //動態門檻值上限
#define SOLID_LINE_DYNAMIC_LOWEST_LIMIT     30    //動態門檻值下限
#endif





#define VEHICLE_WARNING_BOUND_1 100   //cm
#define VEHICLE_WARNING_BOUND_2 150   //cm
#define VEHICLE_WARNING_BOUND_3 200   //cm

//#define VEHICLE_WARNING_BOUND_1 50   //cm
//#define VEHICLE_WARNING_BOUND_2 60   //cm
//#define VEHICLE_WARNING_BOUND_3 80   //cm



//畫面顯示設定
//#define SHOW_LDWS_WARNING_BLOCK
//#define DURATION_DISPLAY          		//顯示執行時間
//#define L_AVG_ROAD_WIDTH                  //顯示車道寬度
//#define SHOW_LANE_STABLE_COUNTER_ON_OFF   //顯示車道線Stable Counter
//#define SHOW_LANE_ROAD_DIFFERENT_VALUE    //顯示車道線與路面平均灰階值的差值
//#define L_DRAW_LAND_BOUND_ON_OFF          //畫出車道線範圍
//#define BIAS_RATIO_CALCULATION            //顯示偏移計算
//#define DRAW_VEHICLE_WIDTH_AND_CENTER		//畫出本車邊界
//#define DRAW_LANE_MARK_ROI                //畫出車道線ROI
//#define DRAW_LANE_MARK                    //畫出偵測到的車道標線
//#define SINGLE_LANE_ROAD_MODEL_DEBUG
//#define DRAW_TB_AND_BB

//#define O_DRAW_VEHICLE_WARNING_BOUND          //畫出車輛警示範圍
//#define SHOW_BIG_F                            //顯示大F
//#define SHOW_UNIT_OF_DISTANCE                 //顯示車距單位
//#define O_SHOW_TRI_LEVEL_RESULT               //顯示三值化結果
//#define O_FZ_PVE_INFORMATION_DISPLAY          //顯示模糊規則數字
//#define O_SHOW_OBSTACLE_STBCOUNTER_AND_CLASS	//顯示目標分類數字
//#define O_DRAW_LAND_BOUND_ON_OFF              //畫出車輛偵測範圍
//#define SHOW_VEHICLE_SHADOW_AND_LIGHT_TH      //顯示車輛陰影和光線門檻值
//#define DRAW_VEHICLE_POSITION_ON_OFF          //畫出車輛底部
//#define O_DRAW_PAIR_GROUP                     //劃出成對投影量
//#define O_SHOW_HORIZONTAL_PROJECTION_ON_OFF   //畫出水平投影量
//#define O_SHOW_VERTICAL_PROJECTION_ON_OFF     //畫出垂直投影量
//#define O_DRAW_VE_VP_DETECT_AREA              //畫出車輛垂直投影量偵測範圍
//#define O_Draw_FromFuzzy                      //根據Fuzzy畫出車輛
//#define O_DRAW_PAIR_CAR_LIGHT                 //畫出成對車燈配對
//#define O_DRAW_VEHICLE_CANDIDATE              //畫出車輛候選物件


#ifdef BIAS_RATIO_CALCULATION
#define DRAW_BIAS_RATIO_NUMBER
//#define FILE_OUTPUT_FOR_BIAS
#endif

//灰階值校正框框
//#define GRAY_LEVEL_REGULATE



//切換車道線模型 curve or line
#define LANE_MODEL_IS_CURVE	    1	 //指定為 curve
#define LANE_MODEL_IS_LINE	    2	 //指定為 line
#define LANE_MODEL_IS_BOTH      3    //兩者
#define LANE_MODEL_SELECTION	LANE_MODEL_IS_BOTH



//單或雙車道搜尋設定
#define SINGLE_LANE_DETECTION_PROCESS           1	//單車道搜尋
#define DUAL_LANE_DETECTION_PROCESS             2	//雙車道搜尋
#define MERGE_SINGLE_AND_DUAL_LANE_DETECTION    3	//單雙車道搜尋
#define SINGLE_OR_DUAL_LANE_DETECTION	        MERGE_SINGLE_AND_DUAL_LANE_DETECTION




//ROI設#定
#define L_SINGLE_LANE_DEFAULT_L_W       44
#define L_SINGLE_LANE_DEFAULT_REGION    55//50//45     //400
#define L_SINGLE_LANE_MAIN_REGION		15//20//30     //100
#define L_SINGLE_LANE_SUB_REGION		10//15//20     //50
#define L_SINGLE_TRACE_MAIN_REGION		8//10//15      //50
#define L_SINGLE_TRACE_SUB_REGION		6//8//10      //35
#define L_SINGLE_LANE_SEARCH_TB			L_IB_TB_SearchLane
#define L_SINGLE_LANE_SEARCH_BB			L_IB_BB_SearchLane
#define L_SINGLE_LANE_TRACE_TB			L_IB_TB_TrackingLane
#define L_SINGLE_LANE_TRACE_BB			L_IB_BB_TrackingLane



//門檻值設定
#define L_SINGLE_LANE_CHANGE_ROI_TH		  3
#define L_SINGLE_LANE_LINE_CURVE_TH		  10//20//80
#define L_SINGLE_LANE_CORRECT_POINT_TH    3	 //原10改1 (緯力 8mm 1/3")
#define L_SINGLE_LANE_U_SHIFT_TH		  10//30//5
#define L_SINGLE_LANE_V_SHIFT_TH		  15
#define L_SINGLE_LANE_EDGE_POINT_TH		  10//20
#define L_PW_SL_PW_MODEL_RESERVE		  5.9//3.6//5.9
#define L_SINGLE_LANE_IN_STABLE			  20
#define L_SINGLE_LANE_MAX_STABLE          25



//搜尋方向設定
#define L_SINGLE_LANE_SEARCH_RIGHT_START		1
#define L_SINGLE_LANE_SEARCH_LEFT_START			2
#define L_SINGLE_LANE_SEARCH_MERGE_DIRECTION	3
#define L_SINGLE_LANE_SEARCH_DIRECTION			L_SINGLE_LANE_SEARCH_MERGE_DIRECTION



//警示時間延遲設定
#define L_WARNING_TIME_DELAY	0.0



//grayscale remapping設定(開/關)
//#define GRAYSCALE_REMAPPING_ON_OFF

#ifdef GRAYSCALE_REMAPPING_ON_OFF
//#define DRAW_MEAN_VALUE
#define GRAYSCALE_DEFAULT_MARK_MEAN		 190//123    //重要的常數
#define GRAYSCALE_SCALING_REGION		  60
#define GRAYSCALE_SCALING_HALF_REGION             30    //GRAYSCALE_SCALING_REGION / 2
#define GRAYSCALE_MEAN_BUFFER			   7    //存儲緩衝區
#define GRAYSCALE_DIVISOR			   3    //GRAYSCALE_MEAN_BUFFER + 1;
#define GRAYSCALE_REMAPPING_MAXIMUM		 215    //255 - (GRAYSCALE_SCALING_HALF_REGION + 10)
#define GRAYSCALE_REMAPPING_MINIMUM		  50	//0 + (GRAYSCALE_SCALING_HALF_REGION + 20)
#endif



//增強影像對比度
//#define INCREASE_CONTRAST_ON_OFF

#ifdef INCREASE_CONTRAST_ON_OFF
#define INCREASE_CONTRAST_PARAMETER     50
#define INCREASE_CONTRAST_PARAMETER2    2
#endif



//車道線動態門檻值
#define LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF					//動態門檻值(開/關)

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
//#define LM_GRAYSCALE_ADPTH_VISIBLE_DEBUGGING						//appear the number of threshold by rectangle
#define DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA					//draw black point and white point for lane marking and road surface area
//此版本調整後主要參數 Threshold Table
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_0				25				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0				25				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_1				20				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1				20				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_2				17				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2				18				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_3				15				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3				15				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_4				14				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4				16				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_5				18				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5				20				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_6				16				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6				18				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_7				14				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7				17				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_8				20				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8				18				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_9				16				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_9				20				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_10			20				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_10			21				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_11			18				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_11			22				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_12			19				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_12			21				//searching index grayscale lower limit
#define LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_13			22				//lane marking grayscale lower limit
#define LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_13			23				//searching index grayscale lower limit
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0				30				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1				40				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2				50				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3				60				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4				70				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5				80				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6				90				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7				100				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_8				110				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_9				120				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_10				130				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_11				140				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12				150				//new addition
#define LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_13				160	    		//new addition
//此版本調整後主要參數
#define LM_GRAY_ADP_TH_BUFFER_SIZE					7				//存儲緩衝區
#define LM_GRAY_ADP_GRAY_TH_MEAN_DIVISOR            3				//LM_GRAY_ADP_TH_BUFFER_SIZE + 1;
#define LM_GRAY_ADP_GRAY_TH_DIVISOR					2				//fixed value (grayscale threshold / 4)
#define LM_GRAY_ADP_MEAN_TH_DIVISOR					3				//fixed value (maen threshold / 2)			//原2改4 (緯力 8mm 1/3") 20110819
#define LM_GRAY_ADP_TH_LONG_MAXIMUM					4200000000                      //數值上限
#define LM_GRAY_ADP_LANE_POINT_TH					10				//動態切成讀Table之車道線點數限制
#endif



//車道線偏離過大時會重新SEARCH
#define L_LOSS_LANE_FOR_BIAS_TO_LARGE

#ifdef L_LOSS_LANE_FOR_BIAS_TO_LARGE
//#define L_SHOW_FIRST_ROW                   //顯示所偵測到的第一個row的位置(下往上數)
#define L_SINGLE_FIRST_ROW_TO_UP	  135 //單車道偵測到第一個row其row座標判斷是否過高之門檻值
#define L_SINGLE_FIRST_ROW_TO_SIDE    10 //單車道偵測到第一個row其col座標判斷是否太過於兩側之門檻值
#define L_DUAL_FIRST_ROW_TO_UP		  135 //雙車道偵測到第一個row其row座標判斷是否過高之門檻值
#define L_DUAL_FIRST_ROW_TO_SIDE      5 //雙車道偵測到第一個row其col座標判斷是否太過於兩側之門檻值
#endif    //L_LOSS_LANE_FOR_BIAS_TO_LARGE



//依據偏移量來決定單車道搜尋方向
#define L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN



//系統保護功能(開/關)(保護機制, 使用者觀感)
//#define L_PROTECTION_MAIN_ON_OFF

#ifdef L_PROTECTION_MAIN_ON_OFF

//*****系統保護功能之顯示(開/關)*****
//#define L_SHOW_PROTECTION_WARNING_SIGN								//顯示(保護機制, 使用者觀感)之警示訊號
//#define L_USING_PERCEPTION_CHANGE_LANE_COLOR						//啟動時(使用者觀感)改變車道線顏色(PointShift:PURPLE, LaneTilt:ORANGE)

//*****系統保護功能之各項子功能(開/關)*****
#define L_PROTECTION_SIDE_BLOCK_ON_OFF								//保護機制_夜間強光之側邊區域(開/關)
#define L_PROTECTION_MIDDLE_BLOCK_ON_OFF							//保護機制_夜間強光之中間區域(開/關)
#define L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF						//保護機制_CCD攝影機漏光(開/關)
#define L_GROUND_REFLECTOR_PROTECTION_ON_OFF						//保護機制_夜間雨天路面反光(開/關)
#define L_WATER_STREAK_DETECTION_ON_OFF								//保護機制_水痕偵測(開/關)
#define L_HORIZONTAL_MARKING_DETECTION_ON_OFF						//保護機制_水平標線偵測(開/關)
#define L_USING_PERCEPTION_POINT_SHIFT_ON_OFF						//使用者觀感_車道線四點位移量之差異值(開/關)
#define L_USING_PERCEPTION_LANE_TILT_ON_OFF							//使用者觀感_車道線傾斜量之差異值(開/關)

//*****系統保護功能之各項子功能(參數設定)*****
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF//===============================保護機制_夜間強光之側邊區域=========================
//#define L_DUAL_LANE_EDGE_SETUP								  	 //設定模式(決定區塊在預設時的位置)
//#define L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE                        	 //顯示區塊位置與相關數據
#define L_BLOCK_ROW_DISTANCE_VANISHPOINT					 10 	 //決定偵測區塊於vanish point下方的距離
#define L_BLOCK_COL_DISTANCE_LANE			    			 20 	 //決定偵測區塊於車道線左右相隔的距離
#define L_PROTECTION_SIDE_BLOCK_ROW_SIZE					 10  	 //決定偵測區塊的寬度
#define L_PROTECTION_SIDE_BLOCK_COL_SIZE					100 	 //決定偵測區塊的長度
#define L_LEFT_EDGE											156 	 //根據設定模式下所決定的預設值(左)
#define L_RIGHT_EDGE										176 	 //根據設定模式下所決定的預設值(右)
#define L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD			200 	 //SideBlock的平均門檻值
#define L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD			 	230 	 //SideBlock的最大像素門檻值
#define L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD			 30 	 //SideBlock內超出最大像素門檻之百分比值
#define L_PROTECTION_SIDE_BLOCK_DIFFERENCE_THRESHOLD		 20 	 //車道線與路面差值門檻
#define L_PROTECTION_SIDE_BLOCK_TOPBLOCKMEAN_THRESHOLD      170 	 //TopBlock的平均門檻值
#endif    //L_PROTECTION_SIDE_BLOCK_ON_OFF
#ifdef L_PROTECTION_MIDDLE_BLOCK_ON_OFF	//============================保護機制_夜間強光之中間區域=========================
//#define L_SHOW_MIDDLE_BLOCK_EDGE									 //顯示計算區域的邊界範圍
//#define L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER						 //顯示符合判斷條件數(1-4)
#define L_PROTECTION_MIDDLE_BLOCK_HEIGHT_RATIO				  0.5	 //iTS->F_H_C * 0.5
#define L_PROTECTION_MIDDLE_BLOCK_FIRST_THRESHOLD			120		 //M_totalBlock的總平均門檻值
#define L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD			150		 //M_subBlock的平均門檻值
#define L_PROTECTION_MIDDLE_BLOCK_SECOND_THRESHOLD			  1		 //計算條件2門檻M_SecondThreshold所乘以的比例
#define L_PROTECTION_MIDDLE_BLOCK_FOURTH_THRESHOLD			  0.75	 //計算條件4門檻M_FourthThreshold所乘以的比例
#endif    //L_PROTECTION_MIDDLE_BLOCK_ON_OFF
#ifdef L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF//=========================保護機制_CCD攝影機漏光==============================
//#define L_SMEAR_SHOW_VALUE_AND_LOCATION								//顯示相關參數
#define SMEAR_GRAY_SCALE_LOWEST_LIMIT						130		 //漏光之灰階門檻值下限(L_CCD_FindSmear(*))
#define SMEAR_SEARCH_GRAY_SCALE_LOWEST_LIMIT				100		 //漏光之灰階門檻值下限(暫無功能)
#define SMEAR_ROW_NUMBER									  8		 //間隔row數
#define SMEAR_SWITCH_DISTANCE_RATE							  0.14	 //切換成SMEAR_ON的有效距離(S_IMGW*0.14)
#define SMEAR_SWITCH_ON_COUNT								  5		 //功能啟動之門檻值
#endif //L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
#ifdef L_GROUND_REFLECTOR_PROTECTION_ON_OFF//=========================夜間雨天路面反光保護機制============================
//#define L_SHOW_BINARIZE_ROW				   		  		         //顯示經過二值化的5行row
//#define L_SHOW_REFLECTOR_WIDTH							 	     //顯示4行row上反光寬度
//#define L_SHOW_DETECTION_BLOCK					  		 	     //顯示偵測到反光區域的方塊
//#define L_SHOW_GROUND_REFLECTOR_VALUE      		 		 	     //顯示相關參數
//#define L_SHWO_SOBEL_EDGE                      		 		     //顯示Sobel的垂直Edge
//#define L_SHOW_SOBEL_PROJECTION				         		     //顯示Sobel垂直Edge的投影
#define L_VANISHING_POINT_DOWN_ROW			 		  		 45      //vanishing point下方n個row開始偵測
#define L_ROW_1_2_DISTANCE					 		  		 15      //row_1和row_2的距離
#define L_ROW_2_3_DISTANCE					 		  		 20      //row_2和row_3的距離
#define L_ROW_3_4_DISTANCE					 		  		 20      //row_3和row_4的距離
#define L_ROW_4_5_DISTANCE					 		  		 25      //row_4和row_5的距離
#define L_BLOCK_WIDTH_LIMIT					 		  		 50    	 //限制經過二值化的row其寬度是否區塊化的門檻
#define L_OVER_TH_PIXEL_PERCENT				        		  0.8125 //區塊內像素門檻比例
#define L_OVER_TH_EDGE_PERCENT				                  0.5625 //區塊內Edge門檻比例
#define L_MINUS_GRAYVALUE_TH_IN_BLOCK        		  		  0      //減少區塊內計算比例的灰階門檻
#define L_RAISE_GRAYVALUE_TH		    	    	  		  0      //增加二值化動態門檻值的門檻
#define L_STABLE_FRAME				  		 		  	   1000      //達到stable的frame張數之門檻值
#endif //L_GROUND_REFLECTOR_PROTECTION_ON_OFF
#ifdef L_WATER_STREAK_DETECTION_ON_OFF//=========================保護機制_水痕偵測========================================
//#define L_SHOW_WATER_STREAK_VALUE                                  //顯示區塊位置與相關數據
#define L_OVER_EDGE_TH_PERCENT								  0.42   //區塊內Edge門檻比例
#define L_OVER_EDGE_TH_PERCENT_OTHER_SIDE					  0.21   //另一側區塊內Edge門檻比例
#define L_PIXEL_DIFFERENT_TH								 30      //區塊內與所抓車道線中心兩者像素差值的門檻
#endif //L_GROUND_REFLECTOR_PROTECTION_ON_OFF
#ifdef L_HORIZONTAL_MARKING_DETECTION_ON_OFF//=======================保護機制_水平標線偵測================================
//#define L_MARKING_SHOW_VALUE_AND_EDGE								//顯示相關參數與偵測區域
#define L_MARKING_HORIZONTAL_EDGE_THRESHOLD				   100		//Edge門檻值
#define L_MARKING_GRAY_THRESHOLD							13		//Edge之灰階門檻值
#define L_MARKING_AREA_HEIGHT_RATE							 0.1	//偵測範圍高度(iTS->F_H_C * 0.1 * 2)
#define L_MARKING_AREA_WIDTH_RATE							 0.6	//偵測範圍寬度(iTS->F_W_C * 0.6 * 2)
#define L_MARKING_DENSITY_ROW_NUMBER						 3		//計算Edge密度之Row數
#define L_MARKING_ONE_ROW_EDGE_RATE							 0.85	//單一Row之Edge密度門檻值
#define L_MARKING_MANY_ROW_EDGE_RATE						 0.7	//多個Row之Edge密度門檻值
#define L_MARKING_HEIGHT_THRESHOLD							 4		//標線高度(Row)門檻值
#define L_MARKING_COUNT_THRESHOLD							25		//啟動之FrameCounter
#endif    //L_HORIZONTAL_MARKING_DETECTION_ON_OFF
//===================================================================使用者觀感_避免誤啟動之防護限制參數==================
//#define L_ADAPTIVE_THRESHOLD_SHOW_VALUE							//顯示動態門檻值相關參數
#define L_ADAPTIVE_THRESHOLD_SINGLE_LOWER_BOUND				30		//SL_TRACE模式之下限動態門檻值
#define L_ADAPTIVE_THRESHOLD_DUAL_LOWER_BOUND    			20		//LTRACE模式之下限動態門檻值
#define L_ADAPTIVE_THRESHOLD_DUAL_BIAS_LOWER_BOUND  	  	10		//LTRACE模式下有偏移之下限動態門檻值
#define L_ADAPTIVE_THRESHOLD_UPPER_BOUND					40		//SL_TRACE與LTRACE模式之上限動態門檻值
#define L_ADAPTIVE_THRESHOLD_FRAME_COUNT				  1000		//flag之影像張數
//使用者觀感_共用參數
#define L_USING_PERCEPTION_FRAME_COUNT 				 		 5		//用於更新之影像張數
#define L_USING_PERCEPTION_SWITCH_ON_THRESHOLD				65		//直接啟動之門檻值
#ifdef L_USING_PERCEPTION_POINT_SHIFT_ON_OFF//========================使用者觀感_車道線四點位移量之差異值=================
//#define L_POINT_SHIFT_SHOW_VALUE									//顯示位移量
#define L_POINT_SHIFT_DUAL_FIRST_THRESHOLD					25		//LTRACE模式1st偏移量門檻值
#define L_POINT_SHIFT_DUAL_SECOND_THRESHOLD					30		//LTRACE模式2nd偏移量門檻值(加快counter累加速度)
#define L_POINT_SHIFT_SINGLE_FIRST_THRESHOLD				25		//SL_TRACE模式1st偏移量門檻值
#define L_POINT_SHIFT_SINGLE_SECOND_THRESHOLD				35		//SL_TRACE模式2nd駑驩e值(加快counter累加速度)
#endif    //L_USING_PERCEPTION_POINT_SHIFT_ON_OFF
#ifdef L_USING_PERCEPTION_LANE_TILT_ON_OFF//==========================使用者觀感_車道線傾斜量之差異值=====================
//#define L_LANE_TILT_SHOW_VALUE									//顯示傾斜量誤差值
#define L_LANE_TILT_DUAL_FIRST_THRESHOLD					14		//LTRACE模式1st差值門檻值
#define L_LANE_TILT_DUAL_SECOND_THRESHOLD					20		//LTRACE模式2nd差值門檻值(加快counter累加速度)
#define L_LANE_TILT_SINGLE_FIRST_THRESHOLD					15		//SL_TRACE模式1st差值門檻值
#define L_LANE_TILT_SINGLE_SECOND_THRESHOLD					25		//SL_TRACE模式2nd差值門檻值(加快counter累加速度)
#endif    //L_USING_PERCEPTION_LANE_TILT_ON_OFF

#endif    //L_PROTECTION_MAIN_ON_OFF



//曲率計算(開/關)
//#define CURVATURE_CALCULATION_ON_OFF

#ifdef CURVATURE_CALCULATION_ON_OFF
#define CURVATURE_CALCULATION_BUFFER_SIZE		   7		 //storage buffer
#define CURVATURE_CALCULATION_DIVISOR			   3		 //sqrt(CURVATURE_CALCULATION_BUFFER_SIZE + 1);
#define CURVATURE_CALCULATION_SHOW_NUMBER_LIMIT    999999    //fixed value
#endif



#define WORLD_X 0
#define WORLD_Y 1



#define HEINFO			0x01
#define VEINFO			0x02
#define HE2INFO			0x04
#define VE2INFO			0x08
#define RSDINFO			0x10
#define RLTINFO			0x20
#define HEVEINFO		0x03
#define LTSDINFO		0x0C
#define RSDRLTINFO		0x30
#define RSDHEVEINFO		0x13



#define LANE_DETECTION       0x01
#define STOP_LINE_DETECTION  0x02
#define ADULT_DETECTION      0x04
#define CHILD_DETECTION      0x08




#define DIRECTION (3.1415926/9)

//For line width (draw size) caculation
#define LineWidth(row_, Size_) (short) Size_ - (((float) row_ / S_IMGCH) * Size_)



typedef unsigned char UBYTE;

typedef struct
{
	char element[195];
}Num;


#define GetImageDataIndex(row_) ((iTS->F_H-1-(row_))*iTS->F_W)
#define SinkDataIndexStart(row_) ((iTS->F_H-1-(row_-1))*iTS->F_W)
#define SinkDataIndexNextRow(r) (r-iTS->F_W)



enum LANEMODEL_t {CURVE ,LINE};
enum DETECTIONMODE_t {LSEARCH, LTRACE, SL_SEARCH, SL_TRACE};
enum MODE_t {SEARCH, TRACE};

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
enum SUPERSED_FLAG_t {REGULAR_PROCESS, SINGLE_SUPERSEDE_DUAL, DUAL_SUPERSEDE_SINGLE};
#endif

enum ROI_t {MAIN, SUB, LSUB, RSUB} ;
enum Mode_t {NONE, LEFT, RIGHT, BOTH} ;
enum CarType_t {Day, Night_CL, Night_HE, BigBlack, MidLight, UnknownObj, None, CarCorner, PatchMethod_01, PatchMethod_02, PatchMethod_03, PatchMethod_04};
enum Feature_t {VE_,CL_,SD_,BB_,HE_,IR_,DS,PV,BSD};
enum LaneWarnning_t {LaneNoWarnning,LeftDeparture,RightDeparture};
enum ObstacleWarnning_t {LeftDeadZone, RightDeadZone, BothDeadZone, SafeDistance, WarningDistance, DangerousDistance};
enum DetectionWarnning_t {DetectionNoWarnning,ObstacleDetectionFail};
enum FindOneSideLane {SL_NotFound, SL_Left, SL_Right};
enum SL_SchROI {SL_SCH_DEFAULT_ROI, SL_SCH_MAIN_ROI, SL_SCH_SUB_ROI};
enum SL_TckROI {SL_TCK_MAIN_ROI, SL_TCK_SUB_ROI};
enum LANE_DETECTION_RETURN_VALUE {L_DUAL_SUCCESS, L_DUAL_FAIL, L_SINGLE_SUCCESS, L_SINGLE_FAIL};
enum OSD_Color_Basis {OCN_RED, OCN_GREEN, OCN_BLUE, OCN_YELLOW, OCN_BLACK, OCN_WHITE, OCN_GRAY, OCN_ORANGE, OCN_PURPLE};
enum OSD_Road_Environment {SIMPLE_ROAD, COMPLEX_ROAD, VERY_COMPLEX_ROAD};
enum SYS_Velocity {VELOCITY_OFF, LOW_VELOCITY, HIGH_VELOCITY};

//#ifdef L_PROTECTION_MAIN_ON_OFF
enum L_PROTECTION_MAIN {PROTECTION_OFF, PROTECTION_ON, USING_PERCEPTION_OFF, USING_PERCEPTION_ON};
enum L_PROTECTION_SUB {MIDDLE_BLOCK_OFF, MIDDLE_BLOCK_ON, SIDE_BLOCK_OFF, SIDE_BLOCK_SINGLE_ON, SMEAR_OFF, SMEAR_SKIP_ON, SMEAR_ON,
	                   GROUND_REFLECTOR_ON, GROUND_REFLECTOR_OFF, WATER_STREAK_ON, WATER_STREAK_OFF, MARKING_OFF, MARKING_ON};
enum L_PROTECTION_USING_PERCEPTION {POINT_SHIFT_OFF, POINT_SHIFT_ON, LANE_TILT_OFF, LANE_TILT_ON, LANE_DELAY_OFF, LANE_DELAY_ON};
//#endif    //L_PROTECTION_MAIN_ON_OFF

#ifdef PARAMETER_EXTRACTION
enum F_DETECTION_SENSITIVITY {F_DETECTION_LOW, F_DETECTION_MIDDLE, F_DETECTION_HIGH};
enum F_WARNING_SENSITIVITY {F_WARNING_LOW, F_WARNING_MIDDLE, F_WARNING_HIGH};
#endif



typedef struct
{
	unsigned char Y;
	unsigned char U;
	unsigned char V;
}S_COLOR;

typedef struct
{
	short Frt;
	short Scd;
}FORWARD_CROI;

typedef struct
{
	short Sd;
	short PixCtr;
	short CurCol;
	short LstCol;
	FORWARD_CROI SchRoi;
	FORWARD_CROI MrkEdg;
	double B_i;
}LaneInfo;

typedef struct
{
	short TB;
	short BB;
	short LB;
	short RB;
	short BW;
	short BH;
}REC_;

typedef struct
{
	REC_ Position;
	char Ctype;
}CARINFO_;

typedef struct
{
	short MaxValue;
	short MaxIndex;
}ArrayPtr;

typedef struct
{
	CARINFO_ CarInfo;
	char FZArr[S_IMGH];
	char StableCtr;
	char UpExtCtr;
	char DownExtCtr;
	char CarSign;
	short RealDistance;
}SECTION_;

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
typedef struct
{
	short LastUPosition;
	short LastVPosition;
	short RightCounter;
	short FindLaneCounter;
	short UpdateCounter;
	char SL_StableCounter;
}L_SL_GROUP_CONDITION;

typedef struct
{
	double K;
	double M;
	double Bm;
	double k;
	double m;
	double bm;
	double br;
	double bl;
	double BR;
	double BL;
	double BM;
	double *Um;
	double *U;
	double *dU;
	double *V;
	double L_UVXY[11];
	double evRodSlp;
	double Vrow;
	short Ctr_DetectedRows;
	short LastFinalRow;
	short CurFinalRow_;
	short LastFinalRow_;
	short WCur;
	short WAvg;
	char L_StbCtr;
	char L_SL_LorR;
	char LaneModelType;
	LaneInfo LaneR,LaneL,LaneM;
	char Flag_RideMarking;
	short Estimation_Last_u1; //Bottom of left
	short Estimation_Last_u2; //Top of left
	short Estimation_Last_u3; //Bottom of right
	short Estimation_Last_u4; //Top of right
}L_SL_LaneModel;
#endif

//road surface grayscale remapping
#ifdef GRAYSCALE_REMAPPING_ON_OFF
typedef struct
{
	short *NewMapping;
	short *Illumination_Mapping;
	short LastRoadMeanValue;
	short LastCorrectMarkMeanValue;
	char MarkMeanFailCounter;
}L_GRAYSCALE_REMAPPING;
#endif

//calculation lane marking area standard deviation
#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
typedef struct
{
	long Threshold;
	unsigned long MarkMean;
	unsigned long RoadMean;
	unsigned short LastRoadMean;
	int ScanRow;
	unsigned int MarkMean_Counter;
	unsigned int RoadMean_Counter;
	short GrayThreshold;
	short MeanThreshold;
	short GrayDiffernce_Value;
}LM_GRAY_ADP_TH;
#endif

#ifdef CURVATURE_CALCULATION_ON_OFF
typedef struct
{
	double RadiusOfCurvature;
	int Avg_Curvature;
}RADIUS_OF_CURVATURE;
#endif

typedef struct
{
	char Velocity;
}SYSTEM_STRUCTURE;

typedef struct
{
    short TB;
    short BB;
    short LB;
    short RB;
    short BW;
    short BH;
    short distance;
    short start_row;
    short start_col;
    short start_flag;
    short stable_counter;
    char mode;


}HORIZONTAL_MARK;

typedef struct
{
    REC_ boundary;
    short distance;
    short stable_counter;
    char mode;
    int direction;

}OBJECT_DATA;

typedef struct
{
    char input_flag;
    char lane_detection;
    char stop_line_detection;
    char child_detection;
    char adult_detection;

}FUNCTION_SWITCH;

typedef struct
{
    unsigned char *O_InfoPlane;
    unsigned char *O_P_InfoPlane;
    unsigned char *L_ColProjection;
    unsigned char *O_MarkInfoPlane;
    unsigned char *Gxy_InfoPlane;
    unsigned char *Axy_InfoPlane;
    unsigned char *Hog_InfoPlane;
	short *O_HD_Array;
	short *O_CL_Array;
	short *O_SD_VerPrjArray;
	short *O_VE_VP_Array;
	unsigned char *Showimage;
	unsigned char *YImg;
	unsigned char *UImg;
	unsigned char *VImg;
	unsigned char *ShowUImg,*ShowVImg;
	short F_W,F_H;
	short F_W_C,F_H_C;
	char L_DetectMode;
	char L_ROI;
	char L_DStatus;
	char L_LaneModelType;
	char L_StbCtr;
	char L_Ctr_LossBothLane;
	char L_Ctr_LossSingleLane;
	LaneInfo LaneR,LaneL;
	double L_evRodSlp;
	double L_Vrow;
	short L_CurFinalRow_;
	short L_LastFinalRow_;
	short L_Ctr_DetectedRows;
	short L_WCur;
	short L_WAvg;
	short L_Bias;
	short L_BiasWarn;
	double K,M,Bm;
	double L_UVXY[11];
	double *Um,*U;
	double *dU,*V;
	double k,m,bm,br,bl;
	short O_WAvg;
	double O_evRodSlp;
	unsigned short *PixelArea;
	short O_SD_ShadowTh;
	short O_BridgeShadow;
	short O_Distance;
	short O_CarLightTH;
	short O_SD_VerPrjArrMean;
	short O_VE_VP_PairGroupNum;
	ArrayPtr O_SD_VerPrjArrPtr;
	ArrayPtr O_VE_VP_ArrPtr;
	FORWARD_CROI O_VE_VP_PairGroupArray[N_MaxPairGroup];
	char O_FZ_PVEInfo[N_MaxPairGroup][N_FeatureInPairEdge];
	char O_FZ_Feature[N_MaxPairGroup][N_AllFeature];
	CARINFO_ V_FZCar;
	SECTION_ *CAR;
	FORWARD_CROI *O_LaneBound;
	FORWARD_CROI *L_LaneMBound;
	FORWARD_CROI *O_LaneMBound;
	SECTION_ CARM;
	FORWARD_CROI *L_LaneLBound;
	SECTION_ CARL;
	FORWARD_CROI *L_LaneRBound;
	SECTION_ CARR;

	char P_ShadowSpeedFlag[10];
	short P_Effort;
	char W_Lane;
	char W_Obstacle;

	short VanishingPoint_V;
	short Skyline_V;
	short CenterPoint_U;

	unsigned char ECU_SelectStatus;

	char SolidlineL;
	char SolidlineR;
	char SolidlineCountL;
	char SolidlineCountR;
	unsigned short SolidlineTH_L;
	unsigned short SolidlineTH_R;

	unsigned char L_Cur_Left_Or_Right;
	short L_LeftLane_Difference_Value;
	short L_RightLane_Difference_Value;

#ifdef L_PROTECTION_MAIN_ON_OFF
	unsigned char L_Protection_Main;
	unsigned char L_UsingPerception_Main;

	unsigned char L_Protection_MiddleBlock;
	unsigned char L_Protection_SideBlockAdaptiveTH;
	unsigned char L_Protection_Smear;
	unsigned char L_Protection_GroundReflector;
	unsigned char L_Protection_WaterStreak;
	unsigned char L_UsingPerception_PointShift;
	unsigned char L_UsingPerception_LaneTilt;
	unsigned char L_UsingPerception_RoadMarking;

	short L_UpdatePoints_MinThreshold;
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
	short L_SideBlock_L_Edge;
	short L_SideBlock_R_Edge;
	unsigned char L_Left_SideBlock_Counter;
	unsigned char L_Right_SideBlock_Counter;
	short Protection_Sideblock_Left_Draw;
	short Protection_Sideblock_Right_Draw;
	unsigned short L_TopBlock_Avg;
#endif    //L_PROTECTION_SIDE_BLOCK_ON_OFF
#ifdef L_PROTECTION_MIDDLE_BLOCK_ON_OFF
	short L_MiddleBlock_LaneBount_L;
	short L_MiddleBlock_LaneBount_R;
#endif    //L_PROTECTION_MIDDLE_BLOCK_ON_OFF
#ifdef L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
	short L_Smear_LB[3];
	short L_Smear_RB[3];
	short L_LightLeak_GrayScaleThreshold;
#endif    //L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
#ifdef L_GROUND_REFLECTOR_PROTECTION_ON_OFF
	short GroundReflector_Ctr;
	short L_Check_High_Pixel_Quantity;
	short L_Last_Mode;
	short L_Dual_to_Single_Flag;
	short L_Lane_Stable;
	short L_Edge_Quantity;
	short L_Over_TH_Pixel_Quantity;
	short L_Pixel_Quantity_in_Block;
#endif //L_GROUND_REFLECTOR_PROTECTION_ON_OFF
#ifdef L_WATER_STREAK_DETECTION_ON_OFF
	short L_Water_Streak_Ctr;
	short L_Block_Point;
	unsigned int L_Left_Block_Pixel_Total;
	unsigned int L_Right_Block_Pixel_Total;
	short L_Left_Block_Pixel_Average;
	short L_Right_Block_Pixel_Average;
	short L_Left_Block_Edge_Total;
	short L_Right_Block_Edge_Total;
	short L_Block_Position;

	int L_Left_Lane_Pixel_Total;
	int L_Right_Lane_Pixel_Total;
	int L_Left_Lane_Pixel_Point;
	int L_Right_Lane_Pixel_Point;
#endif    //L_WATER_STREAK_DETECTION_ON_OFF
#endif    //L_PROTECTION_MAIN_ON_OFF

#ifdef L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
//#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	short L_Last_BiasWarn;
//#endif
#endif //L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN

#ifdef	L_LOSS_LANE_FOR_BIAS_TO_LARGE
	short L_SingleFlag;
	short L_DualLeftFlag;
	short L_DualRightFlag;
	short L_LeftFirstRow;
	short L_LeftFirstCol;
	short L_RightFirstRow;
	short L_RightFirstCol;
#endif //L_LOSS_LANE_FOR_BIAS_TO_LARGE

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	L_SL_GROUP_CONDITION L_SL_PointGroup;
	short SingleLaneDirectionSwitch;
	short MaximumErrorDistance;
	L_SL_LaneModel SL_LaneModel;
#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	char SL_SupersedeFlag;
	char SupersedeCounter;
#endif
#endif

	char L_DetectionResultReturn;
	double L_DelayWarningCounter;

#ifdef GRAYSCALE_REMAPPING_ON_OFF
	L_GRAYSCALE_REMAPPING L_Grayscale_Remapping;
#endif

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
	LM_GRAY_ADP_TH LM_R_GrayScale_AdaptiveThresholding;
	LM_GRAY_ADP_TH LM_L_GrayScale_AdaptiveThresholding;
#endif

#ifdef CURVATURE_CALCULATION_ON_OFF
	RADIUS_OF_CURVATURE CurvatureStructure;
#endif

#ifdef BIAS_RATIO_CALCULATION
	short L_Last_Bias;
	short L_Last_BiasRatio;
#endif

#ifdef PARAMETER_EXTRACTION
	short LDWS_detection_sensitivity;
	short LDWS_warning_sensitivity;
	short FCW_detection_sensitivity;

	short L_draw_single_lane_mark;
	short L_draw_dual_lane_mark;
	short L_warning_vehicle_width;
	short V_draw_vehicle_position;
#endif

	float Duration_Time;
	S_COLOR OSD_Color;

	unsigned int L_EdgeMeanValue;

	SYSTEM_STRUCTURE System_Parameter;

	unsigned int row_width_culmulative[S_IMGH];
	short min_image_car_width[S_IMGCH];
	short max_image_car_width[S_IMGCH];
	char shadow_projection;

	//Corner 變數宣告
	short left_corner_mean;
	short right_corner_mean;
	short have_corner_row;
	short left_corner[10][2];
	short right_corner[10][2];
	short left_corner_count;
	short right_corner_count;
	short pair_corner_count;
	char find_corner_flag;
	short first_search_row;

	//日夜間判斷變數宣告
	char day_or_night_count;
	char day_or_night_flag;


#ifdef INCREASE_CONTRAST_ON_OFF
	unsigned char increase_contrast_map[256];
#endif


	short L_last_bias_flag;

	short V_LAST_BB;
	short V_LAST_BB_use_flag;

	short V_tail_edge_one_time_flag;
	short V_tail_edge_found_row;
	short V_tail_edge_detection_top_boundary;
	short V_serching_boundary;
	short V_distance_number[3];

	short V_last_vehicle_left_bound;
	short V_last_vehicle_right_bound;
	short V_last_vehicle_distance_number_position;

	short L_bias_vehicle_left_wheel_position;
	short L_bias_vehicle_right_wheel_position;
	short L_Bias_vehicle_left_wheel_position_correct_return_value;
	short L_Bias_vehicle_right_wheel_position_correct_return_value;
	short L_bias_vehicle_center_axis_position;
	short L_wheel_position_row;


	//車燈變數宣告
	unsigned char single_tail_light_number;
	short single_tail_light_information[10][6];

	float L_center_lane_curve[S_IMGCH][2];

	float L_W_test[S_IMGCH][4];

    double default_k; //from wheels
    double default_m;
    double default_b;


    short im_debug;

     int max;
     int min;
    int sobel_V_TH;
    int sobel_H_TH;
    int laplacian_TH;

    short O_MarkLightTH;
    short O_Objekt_TH;
   // HORIZONTAL_MARK *Hor_mark;
    HORIZONTAL_MARK Stop_Line;

    FUNCTION_SWITCH function_switch;


    int training_data[36];
    int training_counter;
    double projection_data_average[36];


    OBJECT_DATA adult;
    OBJECT_DATA child;
    char people_detection_change_flag;

    FILE* traindata; // debug file

}ITS;
#endif
