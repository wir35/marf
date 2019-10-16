#ifndef __DIP_CONFIG
#define __DIP_CONFIG

/*Union for the state of configuration dip switch*/
typedef union
{
		struct {		
			unsigned char V_OUT_1V2:1;
			unsigned char V_OUT_1V:1;
			unsigned char SAVE_V_LEVEL:1;
			unsigned char EXPANDER_ON:1;
			unsigned char NU1:1;
			unsigned char NU2:1;
			unsigned char NU3:1;
			unsigned char NU4:1;			
		} b;
	unsigned char DipConfig;
} uDipConfig;

void DipConfig_init(void);
uDipConfig GetDipConfig(void);


#endif
