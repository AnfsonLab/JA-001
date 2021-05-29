#include "r_cg_macrodriver.h"
#include "r_cg_adc.h"
#include "r_cg_userdefine.h"
#include "R_adc.h"

/***********************************************************************************************************************
Private Function Prototypes
***********************************************************************************************************************/
void ADC_conver(void);
static uint16_t Calculate_sensor( uint16_t * DataPoint);
uint16_t	AD_ANI0_result[sampling_count];/*Array for A/D ANI0 result*/
uint16_t	AD_ANI1_result[sampling_count];/*Array for A/D ANI1 result*/
uint16_t	AD_ANI2_result[sampling_count];/*Array for A/D ANI2 result*/
uint16_t	AD_ANI3_result[sampling_count];/*Array for A/D ANI3 result*/
uint16_t	AD_ANI4_result[sampling_count];/*Array for A/D ANI4 result*/
uint16_t	AD_ANI5_result[sampling_count];/*Array for A/D ANI5 result*/
uint16_t	AD_ANI6_result[sampling_count];/*Array for A/D ANI6 result*/
uint16_t	AD_ANI7_result[sampling_count];/*Array for A/D ANI7 result*/
uint16_t	AD_ANI16_result[sampling_count];/*Array for A/D ANI7 result*/
uint16_t	AD_ANI17_result[sampling_count];/*Array for A/D ANI7 result*/
uint16_t	AD_ANI18_result[sampling_count];/*Array for A/D ANI7 result*/
uint16_t	AD_ANI19_result[sampling_count];/*Array for A/D ANI7 result*/
uint8_t	sampling_Counter = 0;          /* A/D result counter */
uint16_t	T_DECP[AD_total_chanel];
uint16_t	ADDECP,DP_POINT;
uint8_t	Ta_SnrOpen_Error_buff,Ta_SnrShort_Error_buff,Tc_SnrOpen_Error_buff,Tc_SnrShort_Error_buff;
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

int16_t	CONTROL_TEMP,COMP_OUT_TEMP,PRE_CONTROL_TEMP= 0xffff,PRE_COMP_OUT_TEMP= 0xffff,calcu_controltemp,calcu_COMP_OUTtemp;
int16_t	CONTROL_TEMP_TEST = 250;

int16_t  BASE_TEMP,PRE_BASE_TEMP= 0xffff,BASE_TEMP_BUFF,calcu_BASEtemp;
int16_t  ANTIFRZ_TEMP,PRE_ANTIFRZ_TEMP= 0xffff,ANTIFRZ_TEMP_BUFF,calcu_Antifrztemp;
int16_t  CONDENSER_TEMP,PRE_CONDENSER_TEMP= 0xffff,CONDENSER_TEMP_BUFF,calcu_CONDENSERtemp;



uint16_t	CONTROL_TEMP_BUFF,COMP_OUT_TEMP_BUFF,Check_themometer_timer;
uint8_t	temp_index,T_DECP_buffer_counter[AD_total_chanel];
uint8_t	adc_conv_end,TempRoom_conv_ok_flag,TempPipe_conv_ok_flag,first_detect_Pipetemp,chk_ad_afterconv;
uint8_t	Room_SnrOpen_Error,Pipe_SnrOpen_Error,Room_SnrShort_Error,Pipe_SnrShort_Error,No_pipe_sensor;

# if 0
const  unsigned  int  cal_table[]={    
 /* start at -20 degree C  advalue start from 123, B value = 3330, 4.99K 20120202*/ 
 /* -20 ~ -11 */	123,129,135,141,147,153,160,167,174,181,
 /* -10 ~  -1 */	189,196,204,212,220,228,236,245,253,262,
 /*    0 ~    9 */	271,280,289,298,307,317,326,336,345,355,
 /*   10 ~  19 */	365,375,384,394,404,414,424,434,444,454,
 /*   20 ~  29 */	463,473,483,493,502,512,522,531,540,550,
 /*   30 ~  39 */	559,568,577,586,595,604,613,621,629,638,
 /*   40 ~  49 */	646,654,662,670,677,685,692,700,707,714,
 /*   50 ~  59 */	721,727,734,740,747,753,759,765,771,777,
 /*   60 ~  69 */	782,788,793,798,804,809,813,818,823,828,
 /*   70 ~  79 */	832,836,841,845,849,853,857,860,864,868,
 /*   80 ~  81 */	871,875
 }; /* To與Te的溫度Table,分壓電阻5k */
#endif
/* this for anfson 10K-7.5K **************************** */

const  unsigned  int  cal_table[]={    
 /* start at -20 degree C  advalue start from 123, B value = 4100, 7.5K 20120202*/ 
 /* -20 ~ -11 */	66,70,74,78,82,87,92,97,102,107,
 /* -10 ~  -1 */	113,119,125,131,138,144,151,159,166,174,
 /*    0 ~    9 */	182,190,198,207,216,225,234,243,253,263,
 /*   10 ~  19 */	273,283,293,304,315,325,336,347,359,370,
 /*   20 ~  29 */	381,393,404,416,427,439,450,462,474,485,
 /*   30 ~  39 */	497,508,520,531,542,553,564,575,586,597,
 /*   40 ~  49 */	607,618,628,638,648,658,668,677,686,695,
 /*   50 ~  59 */	704,713,722,730,738,746,754,762,769,776,
 /*   60 ~  69 */	783,790,797,804,810,816,822,828,834,840,
 /*   70 ~  79 */	845,850,855,860,865,870,874,879,883,887,
  /*  80 ~  89 */	891,895,899,903,906,910,913,916,919,922,
 /*   90 ~  99 */	925,928,931,934,936,939,941,944,946,948,
 /* 100 ~  109 */	950,952,954,956,958,960,962,964,966,968,
 /* 110 ~  119 */	970,972,973,974,976,977,978,979,980,981,
 /* 120 ~  129 */	983,985,987,989,991,993,996,998,1010,1020
 }; /* To與Te的溫度Table,分壓電阻7.5k */

/* this for jason **************************** */
const  unsigned  int  cal_table_1[]={    
 /* start at -20 degree C  advalue start from 123, B value = 3330, 4.99K 20120202*/ 
 /* -20 ~ -11 */	73, 	77, 	81, 	85, 	90, 	95, 	100, 	105, 	110, 	116, 
 /* -10 ~  -1 */	121, 	127, 	134, 	140, 	147, 	153, 	160, 	168, 	175, 	183, 
 /*    0 ~    9 */	191, 	199, 	207, 	215, 	224, 	233, 	242, 	251, 	261, 	270, 
 /*   10 ~  19 */	280, 	290, 	300, 	310, 	320, 	331, 	341, 	352, 	362, 	373, 
 /*   20 ~  29 */	384, 	395, 	406, 	417, 	428, 	439, 	450, 	461, 	472, 	483,
 /*   30 ~  39 */	494, 	505, 	515, 	526, 	537, 	548, 	558, 	568, 	579, 	589, 
 /*   40 ~  49 */	599, 	609, 	619, 	628, 	638, 	647, 	657, 	666, 	675, 	683, 
 /*   50 ~  59 */	692, 	700, 	709, 	717, 	725, 	732, 	740, 	748, 	755, 	762,
 /*   60 ~  69 */	769, 	776, 	782, 	789, 	795, 	801, 	807, 	813, 	819, 	824, 
 /*   70 ~  79 */	830, 	835, 	840, 	845, 	850, 	855, 	860, 	864, 	868, 	873, 
 /*   80 ~  81 */	877, 	881,	891,	895,	899,	903,	906,	910,	913,	916,
 /*   90 ~  91 */	919,	922
 }; /* To與Te的溫度Table,分壓電阻5k */



  /*******************************************************************************
  * Function name : ADC_conver for key input
  * Description   : none
  * Argument  : none
  * Return value  : none
  *******************************************************************************/
  void convert_AD_channel_key(void) 
  {
 
	 ADS = _12_AD_INPUT_CHANNEL_18;
  /* ***************************************************************** */
	 R_ADC_Start();
 
	 /* Wait for the A/D conversion to complete */
 
 
	 while(0U == ADIF);//for CS+ CC
 
	 /* Clear ADC interrupt flag */
	 ADIF = 0U;
 /* ***************************************************************** */ 
	 current_key_now = (ADCR >> 6U);
 
  }
 
  /* ***************************************************************** */
 
   /*******************************************************************************
   * Function name : ADC_conver for voltage input
   * Description   : none
   * Argument  : none
   * Return value  : none
   *******************************************************************************/
   void convert_AD_channel_voltage(void) 
   {
  
	  ADS = _07_AD_INPUT_CHANNEL_7;
   /* ***************************************************************** */
	  R_ADC_Start();
  
	  /* Wait for the A/D conversion to complete */
  
  
	  while(0U == ADIF);//for CS+ CC
  
	  /* Clear ADC interrupt flag */
	  ADIF = 0U;
  /* ***************************************************************** */ 
	  voltage_data_now = (ADCR >> 6U);
  
   }
  
   /* ***************************************************************** */
  
  
 
 
 
   /*******************************************************************************
   * Function name : ADC_conver for key input
   * Description   : none
   * Argument  : none
   * Return value  : none
   *******************************************************************************/
   void convert_AD_channel_current(void) 
   {
  
	  ADS = _06_AD_INPUT_CHANNEL_6;
   /* ***************************************************************** */
	  R_ADC_Start();
  
	  /* Wait for the A/D conversion to complete */
  
  
	  while(0U == ADIF);//for CS+ CC
  
	  /* Clear ADC interrupt flag */
	  ADIF = 0U;
  /* ***************************************************************** */ 
	  current_data_now = (ADCR >> 6U);
  
   }
  
   /* ***************************************************************** */
  
  
 







/*******************************************************************************
* Function name	: ADC_conver
* Description 	: none
* Argument 	: none
* Return value  : none
*******************************************************************************/
void convert_AD_channel(void) 
{
	uint8_t current_channel;
	
	#if 0
	uint16_t i, j, min, tmp, result = 0;
	static uint32_t sensor_voltage;
	static double sensor_Rt;            // sensor present resistance
	static float temperature_value;
	#endif
	
	current_channel = ADS;
	/* Start an A/D conversion */
	R_ADC_Start();

	/* Wait for the A/D conversion to complete */


while(0U == ADIF);//for CS+ CC

	/* Clear ADC interrupt flag */
	ADIF = 0U;
	
	switch(current_channel)
	{
		case	_00_AD_INPUT_CHANNEL_0:
			AD_ANI0_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_01_AD_INPUT_CHANNEL_1:
			AD_ANI1_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_02_AD_INPUT_CHANNEL_2:
			AD_ANI2_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_03_AD_INPUT_CHANNEL_3:
			AD_ANI3_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_04_AD_INPUT_CHANNEL_4:
			AD_ANI4_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_05_AD_INPUT_CHANNEL_5:
			AD_ANI5_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_06_AD_INPUT_CHANNEL_6:
			AD_ANI6_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_07_AD_INPUT_CHANNEL_7:
			AD_ANI7_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_10_AD_INPUT_CHANNEL_16:
			AD_ANI16_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_11_AD_INPUT_CHANNEL_17:
			AD_ANI17_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_12_AD_INPUT_CHANNEL_18:
			AD_ANI18_result[sampling_Counter] = (ADCR >> 6U);
			break;
		case	_13_AD_INPUT_CHANNEL_19:
			AD_ANI19_result[sampling_Counter] = (ADCR >> 6U);
			break;
	}
	sampling_Counter++;
	if(sampling_Counter >= sampling_count)
	{
		sampling_Counter = 0;
		switch(current_channel)
		{
			case	_00_AD_INPUT_CHANNEL_0:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI0_result);
				ADS = _01_AD_INPUT_CHANNEL_1;
				break;
			case	_01_AD_INPUT_CHANNEL_1:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI1_result);
				ADS = _02_AD_INPUT_CHANNEL_2;
				break;
			case	_02_AD_INPUT_CHANNEL_2:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI2_result);
				ADS = _03_AD_INPUT_CHANNEL_3;
				break;
			case	_03_AD_INPUT_CHANNEL_3:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI3_result);
				ADS = _04_AD_INPUT_CHANNEL_4;
				break;
			case	_04_AD_INPUT_CHANNEL_4:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI4_result);
				ADS = _05_AD_INPUT_CHANNEL_5;
				break;
			case	_05_AD_INPUT_CHANNEL_5:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI5_result);
				ADS = _06_AD_INPUT_CHANNEL_6;
				break;
			case	_06_AD_INPUT_CHANNEL_6:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI6_result);
				ADS = _07_AD_INPUT_CHANNEL_7;
				break;
			case	_07_AD_INPUT_CHANNEL_7:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI7_result);
				ADS = _12_AD_INPUT_CHANNEL_18;
				break;
			case	_10_AD_INPUT_CHANNEL_16:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI7_result);
				ADS = _11_AD_INPUT_CHANNEL_17;
				break;
			case	_11_AD_INPUT_CHANNEL_17:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI7_result);
				ADS = _12_AD_INPUT_CHANNEL_18;
				break;
			case	_12_AD_INPUT_CHANNEL_18:
				//T_DECP[current_channel] = Calculate_sensor(AD_ANI18_result);
				T_DECP[12] = Calculate_sensor(AD_ANI18_result);
				ADS = _00_AD_INPUT_CHANNEL_0;
				adc_conv_end = SET;
				break;
			case	_13_AD_INPUT_CHANNEL_19:
				T_DECP[current_channel] = Calculate_sensor(AD_ANI7_result);
				ADS = _00_AD_INPUT_CHANNEL_0;
				adc_conv_end = SET;
				break;
		}
	}
}

/***********************************************************************************************************************
* Function Name : Calculate_Sensor
* Description : This function calculate the sensor value.
* Arguments : none
* Return Value : uint8_t- ANI0~ANI7 output
***********************************************************************************************************************/
static uint16_t Calculate_sensor( uint16_t * DataPoint)
{
    /* Local variables */
    uint8_t i = 0;
    uint16_t t_max_data,t_min_data;
    uint16_t t_TempData;
    t_max_data = *DataPoint;
    t_min_data = *DataPoint;
    t_TempData = 0;
    while(i++ < sampling_count)
    {
        if( *DataPoint > t_max_data )
            t_max_data = *DataPoint;
        if( *DataPoint < t_min_data )
            t_min_data = *DataPoint;
        t_TempData += *DataPoint;
        DataPoint++;
    }
/* Get the average value after delete the max data and min data */  
    t_TempData = ( t_TempData - t_max_data - t_min_data );
    t_TempData = t_TempData/(sampling_count-2);
    return((uint16_t)t_TempData);
}

/***********************************************************************************************************************
* Function Name : Convert_Real_value
* Description : This function convert ad value to real sensor value
* Arguments : none
* Return Value : uint8_t- ANI0~ANI7 output
***********************************************************************************************************************/
void	CAL_COUNT(uint16_t	ADTEMP)
{
	uint16_t	addex=0;
	int16_t		ADSUB;
	uint16_t	UP_VALUE ;
	uint16_t	DOWN_VALUE;
	uint16_t	ZONE_SIZE;
	uint16_t	ADSUB1;

	if(ADTEMP >= cal_table[sizeof(cal_table)/2-1])
	{
		ADDECP = sizeof(cal_table)/2 - 1;
		DP_POINT = 0;
	}
	else
	{
		do
		{
			ADSUB = (ADTEMP - cal_table[addex]);
			addex++;
		}while(ADSUB>0);

		UP_VALUE = cal_table[addex-1];
		DOWN_VALUE = cal_table[addex-2];
		ZONE_SIZE = UP_VALUE - DOWN_VALUE;
		ADSUB1 = ADTEMP - DOWN_VALUE;
		DP_POINT = ( ADSUB1* 10 ) / ZONE_SIZE;
		ADDECP = addex -1;
	}
}

/***********************************************************************************************************************
* Function Name : Convert_Real_value
* Description : This function convert ad value to real sensor value
* Arguments : none
* Return Value : uint8_t- ANI0~ANI7 output
***********************************************************************************************************************/
void	CAL_COUNT_1(uint16_t	ADTEMP)
{
	uint16_t	addex=0;
	int16_t		ADSUB;
	uint16_t	UP_VALUE ;
	uint16_t	DOWN_VALUE;
	uint16_t	ZONE_SIZE;
	uint16_t	ADSUB1;

	if(ADTEMP >= cal_table_1[sizeof(cal_table_1)/2-1])
	{
		ADDECP = sizeof(cal_table_1)/2 - 1;
		DP_POINT = 0;
	}
	else
	{
		do
		{
			ADSUB = (ADTEMP - cal_table_1[addex]);
			addex++;
		}while(ADSUB>0);

		UP_VALUE = cal_table_1[addex-1];
		DOWN_VALUE = cal_table_1[addex-2];
		ZONE_SIZE = UP_VALUE - DOWN_VALUE;
		ADSUB1 = ADTEMP - DOWN_VALUE;
		DP_POINT = ( ADSUB1* 10 ) / ZONE_SIZE;
		ADDECP = addex -1;
	}
}




void	Check_themometer(void)
{
	if(Check_themometer_timer++ > 100 && chk_ad_afterconv == SET)
	{
		Check_themometer_timer = 0;
		if(CONTROL_TEMP < -200)
		{
			Ta_SnrOpen_Error_buff++;
			if(Ta_SnrOpen_Error_buff > 3)
			{
				//Error_state_E1 = SET;
			}
		}
		else if(CONTROL_TEMP > 700)
		{
			Ta_SnrShort_Error_buff++;
			if(Ta_SnrShort_Error_buff > 3)
			{
				//Error_state_E1 = SET;
			}
		}
		else
		{
			Ta_SnrOpen_Error_buff = 0;
			Ta_SnrShort_Error_buff = 0;
			//Error_state_E1 = RESET;
		}
		/*************************************/
		if(BASE_TEMP< -200)
		{
			Tc_SnrOpen_Error_buff++;
			if(Tc_SnrOpen_Error_buff > 3)
			{
				//Error_state_E2 = SET;
			}
		}
		else if(BASE_TEMP > 700)
		{
			Tc_SnrShort_Error_buff++;
			if(Tc_SnrShort_Error_buff > 3)
			{
				//Error_state_E2 = SET;
			}
		}
		else
		{
			Tc_SnrOpen_Error_buff = 0;
			Tc_SnrShort_Error_buff = 0;
			//Error_state_E2 = RESET;
		}
	}
}

void Convert_Real_value(void)
{
	if(adc_conv_end)
	{
		adc_conv_end = RESET;
		/*******************************/
		DP_POINT = 0;
		ADDECP = 0;
		CAL_COUNT_1(T_DECP[_Control_sensor]);
		calcu_controltemp = (ADDECP - 21)*10 + DP_POINT;
		if(PRE_CONTROL_TEMP != calcu_controltemp)
		{
			if(CONTROL_TEMP_BUFF++ > 100)
			{
				CONTROL_TEMP_BUFF = 0;
				CONTROL_TEMP = calcu_controltemp;
				PRE_CONTROL_TEMP = CONTROL_TEMP;
			}
		}
		else
			CONTROL_TEMP_BUFF = 0;
		/*******************************/
		DP_POINT = 0;
		ADDECP = 0;
		CAL_COUNT(T_DECP[_Comp_out_sensor]);
		calcu_COMP_OUTtemp= (ADDECP - 21)*10 + DP_POINT;
		if(PRE_COMP_OUT_TEMP != calcu_COMP_OUTtemp)
		{
			if(COMP_OUT_TEMP_BUFF++ > 100)
			{
				COMP_OUT_TEMP_BUFF = 0;
				COMP_OUT_TEMP = calcu_COMP_OUTtemp;		
				PRE_COMP_OUT_TEMP = COMP_OUT_TEMP;
			}
		}
		else
			COMP_OUT_TEMP_BUFF = 0;
		/*******************************/
		DP_POINT = 0;
		ADDECP = 0;
		CAL_COUNT_1(T_DECP[_Base_sensor]);
		calcu_BASEtemp= (ADDECP - 21)*10 + DP_POINT;
		if(PRE_BASE_TEMP != calcu_BASEtemp)
		{
			if(BASE_TEMP_BUFF++ > 100)
			{
				BASE_TEMP_BUFF = 0;
				BASE_TEMP = calcu_BASEtemp;
				PRE_BASE_TEMP = BASE_TEMP;
			}
		}
		else
			BASE_TEMP_BUFF = 0;
		/*******************************/
		DP_POINT = 0;
		ADDECP = 0;
		CAL_COUNT(T_DECP[_Antifrz_sensor]);
		calcu_Antifrztemp= (ADDECP - 21)*10 + DP_POINT;
		if(PRE_ANTIFRZ_TEMP != calcu_Antifrztemp)
		{
			if(ANTIFRZ_TEMP_BUFF++ > 100)
			{
				ANTIFRZ_TEMP_BUFF = 0;
				ANTIFRZ_TEMP = calcu_Antifrztemp;
				PRE_ANTIFRZ_TEMP = ANTIFRZ_TEMP;
			}
		}
		else
			ANTIFRZ_TEMP_BUFF = 0;
		/*******************************/
		/*******************************/
		DP_POINT = 0;
		ADDECP = 0;
		CAL_COUNT(T_DECP[_Condenser_sensor]);
		calcu_CONDENSERtemp= (ADDECP - 21)*10 + DP_POINT;
		if(PRE_CONDENSER_TEMP != calcu_CONDENSERtemp)
		{
			if(CONDENSER_TEMP_BUFF++ > 100)
			{
				CONDENSER_TEMP_BUFF = 0;
				CONDENSER_TEMP = calcu_CONDENSERtemp;
				PRE_CONDENSER_TEMP = CONDENSER_TEMP;
			}
		}
		else
			CONDENSER_TEMP_BUFF = 0;
		/*******************************/
	}
}

void ADC_conver(void)
{
	convert_AD_channel();
	Convert_Real_value();
}
