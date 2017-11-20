#include "InitialVariable.h" //c++

//L_ResetVariable.c
void SetITSBuffer(ITS *iTS);
void FreeMemory(ITS *iTS);
short ResetConstant(ITS *iTS);

void RstDualLM_Variables(ITS *iTS);
void L_RstDualLM_Variables(ITS *iTS);
void F_ResetParameterExtraction(ITS *iTS);

void F_L_CalculationWheelPositionOnScreen(ITS *iTS);


void CreateEdge(ITS *iTS);

//L_DecisionFlow.c
void L_DualLM_DecisionTree(short row_,ITS *iTS);
short L_DualLM_FindInitBothPoints(short row_,ITS *iTS);



//L_DetectionFlow.c
short DWtoDI(double DW,double Vrow_);
short DItoDW(short DI,double Vrow_);
short L_DualLM_Searching(ITS *iTS);
short L_DualLM_Tracking(ITS *iTS);



//L_Estimation.c
void L_DualLM_BuildModel(ITS *iTS);
short L_DualLM_Estimation(ITS *iTS);
void Upt_Coef_Fitting_CURVE(register double a[5],register double b[3], register double c[3]);
void Upt_Coef_Fitting_LINE(const double a[5], const double b[2], double c[2]);



//L_Search.c
short IsMark(LaneInfo *Lane,short row_,UBYTE *pSrc, ITS *iTS);
short MarkSearch(short index,short row_,LaneInfo *Lane,short maxMW,short minMW,UBYTE *pSrc,ITS *iTS);



//L_SpecifyROI.c
void SetROI(FORWARD_CROI *roi,short Rmin,short Rmax,short last,short d);
typedef void(SpecifyMainROIFunc)(LaneInfo *Lane,ITS *iTS);
SpecifyMainROIFunc  SpfMainRoi_DftROI_KMB;
SpecifyMainROIFunc  SpfMainRoi_CtnRoi_KMB;
void SpecifyROI(SpecifyMainROIFunc *SpecifyMainROI,ITS *iTS );



//L_UsingFunction.c

short ITSLANE_MAIN(ITS *iTS);
void DetectionFunctionDecision(ITS *iTS);

void L_DetectionDecision(ITS *iTS);
short LaneDetection(ITS *iTS);
short LaneModel( double Vrow,  double _K,  double _M,  double _B);
void DrawDistance(short Value,short ROW_,short COL,ITS *iTS);
void DrawShowLane_By_L_LandBound(short Colorflag,int r,short DrawCol,short size,ITS *iTS);
void CalculateCenterLaneCurve(ITS *iTS);
void L_GetBiasByImage(ITS *iTS);
void L_DrawLaneDepartureWarningBlock(ITS *iTS);
short LimitW(short i);
short LimitH(short i);
short LimitVP(short i);
char LimitCount(char i,char Bound);
short LimitSearchingBoundary(short row, ITS *iTS);
//void F_L_DrawWheelPositionInformationOnScreen(ITS *iTS);
void Draw_cm(char Small_or_Big, short StartCol, short StartRow, ITS *iTS);

#ifdef PARAMETER_EXTRACTION
void F_SetParameterExtraction(ITS *iTS);
#endif

#ifdef DRAW_VEHICLE_WIDTH_AND_CENTER
void F_L_DrawWheelPositionOnScreen(ITS *iTS);
void F_L_DrawWheelPositionInformationOnScreen(ITS *iTS);
#endif



//F_L_SystemProtection.c
#ifdef L_PROTECTION_MAIN_ON_OFF
void L_ShowWarningForProtection(short Protection_Mode, ITS *iTS);
void L_Protection_Initial(ITS *iTS);
void L_DecisionForSystemProtection(ITS *iTS);
void L_Protection_DecisionMinAdaptiveTH(ITS *iTS);
void L_SetLandBoundForProtection(unsigned char *Img,short row, ITS *iTS);
void L_Protection_MiddleBlock(ITS *iTS);
void L_DecisionSkyline(ITS *iTS);
void L_FindSideBlockMean(short StartRow, short L_StartCol, short R_StartCol, short RowSize, short ColSize, ITS *iTS);
void L_Protection_DecisionSide(ITS *iTS);
void L_DecisionTopBlockMean(ITS *iTS);
char L_CCD_FindSmear(ITS *iTS);
void L_Decision_CCD_Smear(ITS *iTS);
void F_L_WaterStreakDetection(ITS *iTS);
void F_L_WaterStreakDetectionBlock(short start_row, short start_col, short cut_middle_width, short left_or_right, ITS *iTS);
void L_HorizontalMrakingDetection(ITS *iTS);
#endif    //L_PROTECTION_MAIN_ON_OFF



//F_L_ReflectorProtection.c
#ifdef L_PROTECTION_MAIN_ON_OFF
void F_L_GroundReflector(ITS *iTS);
short F_L_DecisionBondary(short start_row, short start_col_middle, short last_road_mean, short col_width, short start_col_first, short block_width, ITS *iTS);
short F_L_CountBlock(short start_row, short start_col, short col_width, short last_road_mean_half, short block_target, short row_target, ITS *iTS);
void F_L_DecisionStableOrUnstable(ITS *iTS);
void F_L_DualToSingle(ITS *iTS);
void F_L_DecisionGroundReflector(ITS *iTS);
void F_L_Sobel(ITS *iTS);
void F_L_ReflectorEdge(ITS *iTS);
void F_L_ShowVEVP(unsigned char *Dst, short *VE_VP_Array, ITS *iTS);
#endif    //L_PROTECTION_MAIN_ON_OFF



//F_L_UsingPerception.c
#ifdef L_PROTECTION_MAIN_ON_OFF
void L_ShowWarningForUsingPerception(short UsingPerception_Mode, ITS *iTS);
void L_AdaptiveThresholdFromUpdateLanePoints(ITS *iTS);
void L_FourPointsShift(ITS *iTS);
void L_ShowLaneTiltValue(ITS *iTS);
void L_ChangingShowLaneColor(char Color, ITS *iTS);
#endif //L_PROTECTION_MAIN_ON_OFF



//L_SundriesFunction.c
void CameraSetUp(ITS *iTS);
void DrawRect(short RectStartRow, short RectStartCol, short DrawSize, unsigned char ColorNum, ITS *iTS);
void SolidOrDashedLine(ITS *iTS);
void VelocityMode_Control(short VelocityIn, ITS *iTS);
void GetGrayLevel(short RectStartRow, short RectStartCol, short DrawSize, ITS *iTS);

#ifdef INCREASE_CONTRAST_ON_OFF
void IncreaseContrast(ITS *iTS);
#endif //INCREASE_CONTRAST_ON_OFF

void GrayLevelRegulate(ITS *iTS);



//O_DrawFunc.c
void DrawRectangle(short BB, short TB, short LB, short RB, ITS *iTS);
void DrawColorLine(short row, short LB, short RB, ITS *iTS);
void DrawBigF(short Number, short StartCol, short StartRow, ITS *iTS);
void DrawRect_Has_UV(short RectStartRow, short RectStartCol, short DrawSize, ITS *iTS);
void DrawTriangle_Has_UV_Right(short start_row, short start_column, short draw_size, ITS *iTS);
void DrawTriangle_Has_UV_Left(short start_row, short start_column, short draw_size, ITS *iTS);
void ScalableNumber(int Number, short DrawSize, unsigned short UPosition, unsigned short Vposition, ITS *iTS);
void DrawBar(short BarStartRow, short BarStartCol, short Size_of_Column, short Size_of_Row, ITS *iTS);
void OSD_Color_Setup(unsigned char OSD_Color_Number, ITS *iTS);

//for single lane detection
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
void L_RstSingleLM_Variables(ITS *iTS);
void L_RstSingleLM_Tracking_Variables(ITS *iTS);
void L_SingleLM_SpecifyROI(short ROI_Mode, LaneInfo *Lane, ITS *iTS);
short IsSingleMark(LaneInfo *Lane,short row_,UBYTE *pSrc, ITS *iTS);
short L_SingleLM_UpdateModel(short row_,LaneInfo *Lane,ITS *iTS);
short L_SingleLM_MarkSearch(short row_,ITS *iTS);
short L_SingleLM_RoadModel(ITS *iTS);
short L_SingleLM_DecideModeOrder(ITS *iTS);
void L_SingleLM_MeanSquareAddingData(double weight,double x,double y,double MatrixA[5],double VectorB[3]);
void L_SingleLM_AddLanMdl(register short weight,register short row_,register short L,register short R, ITS *iTS);
void L_SingleLM_BuildLaneModel(ITS *iTS);
short L_SingleLM_Estimation(ITS *iTS);
short L_SingleLM_Searching(ITS *iTS);
short L_SingleLM_Tracking(ITS *iTS);
short L_SingleLM_Tracking_PreProcess(short Final_row, ITS *iTS);
short L_SingleLMProcess(ITS *iTS);

#ifdef L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
void L_DecisionSingleDirection(ITS *iTS);
#endif //L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
#endif



#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
// initial variables
void Dual_ReplacedBy_Single_Reset(ITS *iTS);
//sigle ==> Dual 
short Replaced_SingleLane_With_DualLane(ITS *iTS);
//Dual ==> Single
short Replaced_DualLane_With_SingleLane(ITS *iTS);
#endif



//for grayscale distribution remapping
#ifdef GRAYSCALE_REMAPPING_ON_OFF
void GrayScaleRemapping(ITS *iTS);
void CreateIlluminationTable(ITS *iTS);
#endif



#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
void LM_GrayScale_Adaptive_Thresholding_Initial(LM_GRAY_ADP_TH *TargetStructure);
void LM_GrayScale_Adaptive_Thresholding_PreProcess(LM_GRAY_ADP_TH *TargetStructure, short LBound, short RBound, UBYTE *pSrc, ITS *iTS);
char LM_GrayScale_Adaptive_Thresholding_Calculation(LM_GRAY_ADP_TH *TargetStructure, ITS *iTS);
#endif



#ifdef CURVATURE_CALCULATION_ON_OFF
void Radius_of_Curvature(int Y_AxisPosition, ITS *iTS);
#endif



// for lane model supersedes
#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
void DualSupersedeSingle_Reset(ITS *iTS);
short DualLane_Supersede_SingleLane(ITS *iTS);
#endif


//====== obstacle ======//
//O_ArrayUse.c
short F_V_FindContinuousProjection(short not_successive_threshold, unsigned char *image_buffer, short start, short end, short row, ITS *iTS);
short O_FindMax(short *Array,short start,short end,short *MaxIndex);
short O_NormalizeArray(short *Array,short start,short end,short MaxValue);
short O_GetMean(short *Array,short start,short end);
short O_FindSingleGroup(short Th,short Endurance,short MinWidth,short start,short end,short *InArr,FORWARD_CROI *Result);
short O_FindPairGroup(short MinW,short MaxW,FORWARD_CROI *In,short InGrpNum,FORWARD_CROI *Out);
void ArrayPreprocessing(ITS *iTS);
void CalculateImageCarWidth(ITS *iTS);
short F_V_ProjectionFeatrueExtraction(short left_boundary, short right_boundary, ITS *iTS);
short F_V_FindBoundaryUsedExistedVerticalEdgeProjection(short left_boundary, short right_boundary, ITS *iTS);



//O_Main.c
short O_Main(ITS *iTS);
short O_GetDistance(short row_,ITS *iTS);
short O_GetRowDistance(short distance,ITS *iTS);
short O_GetRow_(short distance,ITS *iTS);
short O_DItoDW(short value,short row_,ITS *iTS);
short O_DWtoDI(short value,short row_,ITS *iTS);
void F_V_DetectionDayOrNight(short bottom_bound, short top_bound, short left_bound, short right_bound, ITS * iTS);
short O_FindObstacle(ITS *iTS);
void V_DrawVehiclePosition(CARINFO_ *CarInfo, short bottom_bound, ITS *iTS);
void O_Training_LR_Datan(int XOffset, int YOffset, int width, int height, ITS *iTS);
void O_WriteTrainDataToTxt(float *data, int number, ITS *iTS);


void O_PedestrianDetection(ITS *iTS);

int O_SearchAdultPedestrian(ITS *iTS);
int O_TrackingAdultPedestrian(ITS *iTS);
int O_FindAdultPeopleCandidate(int star_row, int star_col,int image_width, int image_height, unsigned short *new_bound, ITS *iTS);
int O_AdultHumanDecision(int star_row, int star_col,int cell_width, int cell_height, ITS *iTS);

int O_SearchChildPedestrian(ITS *iTS);
int O_TrackingChildPedestrian(ITS *iTS);
int O_FindChildPeopleCandidate(int star_row, int star_col,int image_width, int image_height, unsigned short *new_bound, ITS *iTS);
int O_ChildHumanDecision(int star_row, int star_col,int image_width, int image_height, ITS *iTS);

float classifier(float *weight, float *data, int data_amount);


int O_FindCorrectBoundary(int star_row, int star_col,int image_width, int image_height,  ITS *iTS);
void O_Training(int XOffset, int YOffset, int width, int height, ITS *iTS);
void O_TrainingCandidate(int XOffset, int YOffset, int width, int height, ITS *iTS);
int O_CalculateCellHogDirection(unsigned char *Gxy_result, unsigned char *Axy_result, int cell_width, int cell_height, ITS *iTS);
int O_CalculateHogCellDirection(unsigned char *Gxy_result, unsigned char *Axy_result, int cell_width, int cell_height, ITS *iTS);


void O_FindHorizontalLine(ITS *iTS);
short O_StopLineSearch(short bottom_bound, short top_bound, short left_bound, short right_bound, ITS *iTS);
short O_StopLineTracking(ITS *iTS);
short O_IsHorizontalMark(short current_row, short current_col,short right_bound, short min_mark_height, short max_mark_height, ITS *iTS);
short O_FindHorizontalMarkEdge(short current_col,  short from_row,  short to_row, ITS *iTS);
short O_GetRowDistanceBetweenRowAndRow(short distance, short row_, ITS *iTS);





//F_V_CornerDetection.c
void F_V_BigCornerMask(unsigned char *image_buffer, short row, short column, ITS * iTS);
void F_V_MiniCornerMask(unsigned char *image_buffer, short row, short column, ITS * iTS);
void F_V_CornerReset(ITS * iTS);
void F_V_CleanOldCorner(short row, ITS * iTS);
void F_V_FindCorner(unsigned char *image_buffer, short row, short column, ITS * iTS);
short F_V_CornerPairDetector(short row, ITS * iTS);



//F_V_RowByRow.c
short O_Search_RowByRow(short BB,short TB,ITS *iTS);
short O_Tracking_RowByRow(short row_,short Margin,ITS *iTS);
short V_CleanCarInfo(CARINFO_ *CarInfo);
void V_UpdateCarInfoFromFZCar(CARINFO_ *CarInfo,ITS *iTS);
short O_SearchDownArea(short BB,short TB,ITS *iTS);
short O_SearchUpArea(short BB,short TB,ITS *iTS);
short O_GetImgLaneWidth(short row_,ITS *iTS);



//F_V_SD.c
void O_SetPixelArea(ITS *iTS);
void O_ProjectRoadShadow(short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS);
void O_CheckBridgeShadow(short Srow_,short Erow_,ITS *iTS);
short O_CheckShadowInPariVE(short Srow_,short LB,short RB,ITS *iTS);



//F_V_Fuzzy.c
void O_FuzzyUpdatePerFrame(ITS *iTS);
short O_FuzzySystem_row(short row_,ITS *iTS);



//F_V_TailLightDetection.c
short O_CheckCarLightInPairVE(short BB,short TB,short LB,short RB,ITS *iTS);



//O_VD.c
short O_VE_VP_SetArray(short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS);
void O_VE_VP_FindPairGroupFlow(short Srow_,short Scol,short Ecol,short RefIW,ITS *iTS);
void F_V_VerticalProjection(short start_row, short end_row, short start_col, short end_col, ITS *iTS);

#ifdef O_SHOW_VERTICAL_PROJECTION_ON_OFF
short O_ShowVEVP(unsigned char *Dst,ITS *iTS);
#endif



//F_V_DD.c
short O_DeadZoneDetection(ITS *iTS);
short O_CheckVEDensityInPairVE(short BB,short TB,short LB,short RB,ITS *iTS);
short O_CheckAllDarkLightInPairVE(short BB,short TB,short LB,short RB,ITS *iTS);
short O_CheckHEDensityInPairVE(short BB,short TB,short LB,short RB,ITS *iTS);
short F_V_AgainstLightUsingVerticalEdgeFeature(ITS *iTS);



//F_V_HD.c
short O_ShowHEHP(unsigned char *Dst,ITS *iTS);
short O_SetPossiblePosition(short BB,short TB,short Irow,ITS *iTS);
short V_DetemineContinuousProjectionPosition(short row_, short start, short end, ITS *iTS);



//V_VehicleCompare.c
short V_CleanCarInfo(CARINFO_ *CarInfo);
short V_VehicleCompare(CARINFO_ *VOld,CARINFO_ *VNew, short AreaTH ,short FillTH,short HFillTH);


//F_V_INFOProc.c
void O_CreateRtgRoadLightShadowInfo(short LightTH,short ShadowTH,short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS);
void O_CreateMarkInfo(ITS *iTS);
void F_O_CreateInfoPlan(ITS *iTS);



//P_SpeedProgram.c
void P_ResetShadowSpeedFlag(ITS *iTS);
void P_O_CreateRtgRoadLightShadowInfo(short SINKrow_,short LightTH,short ShadowTH,short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS);
short P_AddEffortAndCheck(short add,ITS *iTS);



//O_Region.c
short O_CheckInTrackingROI(short LB,short RB,ITS *iTS);
void SetLaneBnd(ITS *iTS);
short O_ModifyPosition_TROI(short BB,short TB,ITS *iTS);
short O_ModifyBoundary_TROI(ITS *iTS);

#ifdef L_DRAW_LAND_BOUND_ON_OFF
short L_DrawLandBnd(unsigned char *Img,ITS *iTS);
#endif

#ifdef O_DRAW_LAND_BOUND_ON_OFF
short O_DrawLandBnd(unsigned char *Img,ITS *iTS);
#endif


