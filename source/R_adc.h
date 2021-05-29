#ifndef R_ADC_H
#define R_ADC_H

//#define	AD_total_chanel	20      /* old 12 */
#define	sampling_count	10
#define	ADC_VREF	(5000)/* VREF(+): 5k mV VDD */
#define	sensor_B_constant	3330	//log(R25/R85)
#define	resistance_at_25	5000	//5k ohms
#define	series_resistance	4999	//4.99k ohms

void convert_AD_channel(void);
void	ADC_conver(void);
void	Check_themometer(void);
#endif