#include "imu.h"
#include "bmi088.h"
#include "algorithm_math.h"
#include <math.h>


//static float NormAccz;
float NormAccz;  //»úÌåÔÚ´¹Ö±ÓÚµØÃæ·½ÏòÉÏµÄ¼ÓËÙ¶È£¨°üº¬ÖØÁ¦ÒÔ¼°ÔË¶¯¼ÓËÙ¶È£©¡£²¢·ÇÄ³¸öÖáµÄ¼ÓËÙ¶È£¬¶øÊÇÍ¨¹ýËùÓÐÖáµÄ¼ÓËÙ¶È¼ÆËã¶øÀ´¡£
//6msÒ»´Î
// 6ms调用一次
void IMU_GetAngle(IMU_t *imu, float dt) 
{
	volatile struct V{
        float x;
        float y;
        float z;
    } Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// ¼ÓËÙ¶È¹éÒ»»¯
    NormQuat = Q_rsqrt(squa(imu->acc[0])+ squa(imu->acc[1]) +squa(imu->acc[2]));
	
    Acc.x = imu->acc[0] * NormQuat;
    Acc.y = imu->acc[1] * NormQuat;
    Acc.z = imu->acc[2] * NormQuat;	
 	//ÏòÁ¿²î³ËµÃ³öµÄÖµ
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//ÔÙ×ö¼ÓËÙ¶È»ý·Ö²¹³¥½ÇËÙ¶ÈµÄ²¹³¥Öµ
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
	//½ÇËÙ¶ÈÈÚºÏ¼ÓËÙ¶È»ý·Ö²¹³¥Öµ
    Gyro.x = imu->gyro[0] * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//»¡¶ÈÖÆ
    Gyro.y = imu->gyro[1] * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = imu->gyro[2] * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// Ò»½×Áú¸ñ¿âËþ·¨, ¸üÐÂËÄÔªÊý

	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// ËÄÔªÊý¹éÒ»»¯
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	{
		 	/*»úÌå×ø±êÏµÏÂµÄZ·½ÏòÏòÁ¿*/  //ÕâÀïÃèÊö¸ü×¼È·Ó¦¸ÃÊÇµØÃæ×ø±êÏµÏÂµÄzÏòÁ¿¡£
		//float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*¾ØÕó(3,1)Ïî*/
		float vecxZ =  2 * NumQ.q1 * NumQ.q3 - 2 * NumQ.q0 *NumQ.q2 ;//¾ØÕó(3,1)Ïî£¬£¬ ¸ü¸Ä£¬ÉÏÃæÕâ¾ä·½Ïò·´ÁË£¬×¢Òâ£ºpAngE->pitchÒ²Òª¸Ä·½Ïò
		//ÒªÇó´ïµ½ÕâÑùµÄÐ§¹û£¬ÔÚ¾²Ö¹×´Ì¬ÏÂ£¬·É»ú·­¹ö»ò¸©ÑöÊ±£¬ËäÈ»ÍÓÂÝÒÇz¼ÓËÙ¶È²»¶Ï±ä»¯£¬µ«ÔÚµØÃæ×ø±êÏÂ£¬·É»ú´¹Ö±ÓÚµØÃæµÄ¼ÓËÙ¶È NormAccz Ê¼ÖÕÎªÖØÁ¦¼ÓËÙ¶ÈÖµ¡£
		
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*¾ØÕó(3,2)Ïî*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*¾ØÕó(3,3)Ïî*/		 
		
			#ifdef	YAW_GYRO
			imu->yaw = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				//ÕâÀï6msµ÷ÓÃÒ»´Î£¬Êµ¼Ê2ms²âÁ¿Ò»´Î
				//float yaw_G = pMpu->gyroZ * Gyro_G;//½«ZÖá½ÇËÙ¶ÈÍÓÂÝÒÇÖµ ×ª»»ÎªZ½Ç¶È/Ãë      Gyro_GÍÓÂÝÒÇ³õÊ¼»¯Á¿³Ì+-2000¶ÈÃ¿ÃëÓÚ1 / (65536 / 4000) = 0.03051756*2		
				//6msÄÚ¼¸´Î²âÁ¿µÄÆ½¾ùÖµ
				float yaw_G = imu->gyroZ_sum * Gyro_G; //2ms²âÁ¿ÖµµÄ×ÜºÍ
				yaw_G = yaw_G/imu->gyroZ_sum_cnt;//Çó2ms²âÁ¿Æ½¾ùÖµ
				imu->gyroZ_sum=0;//¸´Î»
				imu->gyroZ_sum_cnt=0;//¸´Î»
					
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //Êý¾ÝÌ«Ð¡¿ÉÒÔÈÏÎªÊÇ¸ÉÈÅ£¬²»ÊÇÆ«º½¶¯×÷,µ±yaw_G=1¶È/Ãë, gyroZ=16.384
				//if((yaw_G > 0.2f) || (yaw_G < -0.2f)) //Êý¾ÝÌ«Ð¡¿ÉÒÔÈÏÎªÊÇ¸ÉÈÅ£¬²»ÊÇÆ«º½¶¯×÷,µ±yaw_G=1¶È/Ãë, gyroZ=16.384
				{
						//×¢Òâ£º¶ªÆúÎ¢Ð¡yaw½ÇËÙ¶È£¬»áµ¼ÖÂyawÖð½¥Æ¯ÒÆ£¬½â¾ö·½·¨ÊÇÉèÖÃYAWÆ«º½ÁãÆ¯ÐÞÕý
						//pAngE->yaw  += yaw_G * dt;//½ÇËÙ¶È»ý·Ö³ÉÆ«º½½Ç£¬
						//Êµ¼Ê×ªÒ»È¦Ö»ÓÐ319¶È×óÓÒ¡£Õý·´×ª²î2¶È
						imu->yaw  +=   yaw_G * dt /0.89f; //Õâ¸ö½Ç¶ÈÊÇÁ¬ÐøµÄ£¬¿ÉÒÔ³¬¹ý+/-360¶È
				}
				
				//µ±rollºÍpitch½ÇËÙ¶ÈºÜÐ¡µÄÊ±ºò¡£²¢ÇÒµ±·É»ú¾²Ö¹Ê±£¬yaw½ÇËÙ¶ÈÒ²Ó¦¸ÃÎª0£¬·ñÔòÈÏÎªyaw½ÇËÙ¶ÈÐèÒª¸üÐÂÐÞÕýÖµ
				// else if( (ABS(imu->gyro[0])<10 || ABS(imu->gyro[1])<10 ) && absFloat(imu->pitch)<5.0f && absFloat(imu->roll)<5.0f){ 
					
				// 		//µ±·É»ú´¦ÓÚ¾²Ö¹×´Ì¬£¬²¢ÇÒË®Æ½×ËÌ¬½ÇÆ«ÒÆ²»´óµÄÊ±ºò
				// 		//ÄÄÅÂÖ»ÓÐ+/-3µÄ½ÇËÙ¶È²âÁ¿Æ¯ÒÆ£¬Ò²ÓÐ0.18¶È/ÃëÁË£¬Ò»·ÖÖÓ10.8¶È£¬6·ÖÖÓ64.8¶ÈÎó²î
				// 		//ËùÒÔÕâÀïÐèÒª°Ñ¾²Ö¹×´Ì¬Ê±ÕæÕýÐÞÕýµ½Áã
				// 		if(!ALL_flag.unlock || Aux_Rc.thr<1200 ){
							
				// 				//int16_t gyroZ_tmp = round( (float)pMpu->gyroZ + MPU6050.gyroZ_offset ); //ÕâÒ»¾äÔÚ mpu6050.cÀïÃæÖ´ÐÐ£¬Õâ±ßÁÙÊ±²âÊÔ
				// 				int16_t gyroZ_tmp = imu->gyro[2]; //¾­¹ýÐÞÕýµÄÔ­Ê¼Öµ

				// 				//ÐÞÕý·½ÏòÓÚÆ«²î·½ÏòÏà·´ //½ö½öÊÇ¾²Ì¬ÐÞÕý
				// 				imu->gyroZ_offset -= ( (float)gyroZ_tmp )*0.005f;  //¼ÆËãpMpu->gyroZÊ±£¬»á¼ÓÉÏÐÞÕýÖµ
							
				// 				//ÏÞÖÆÐÞÕý·ù¶È
				// 				if(imu->gyroZ_offset>10.0f) imu->gyroZ_offset =10.0f;
				// 				if(imu->gyroZ_offset<-10.0f) imu->gyroZ_offset =-10.0f;
								
				// 				imu->gyroZ_cnt++;
				// 		}
				// }
				
				
				// if(ALL_flag.unlock && Aux_Rc.thr>1300 ){//·ÉÐÐÖÐ,Æ«º½²âÁ¿Æ«²î²¹³¥£¬·É»ú³¯ÄÄ¸ö·½ÏòÆ¯ÒÆ£¬¾Í³¯ËûµÄ·´·½Ïò²¹³¥¡£
				// 				//±ÈÈçË³Ê±ÕëÆ¯ÒÆ£¬ÎÒÃÇ¾Í°Ñ²¹³¥ÉèÖÃÎªÕý£¨ÄæÊ±Õë²¹³¥£©£¬ÄæÊ±ÕëÆ¯ÒÆ£¬¾Í°Ñ²¹³¥ÉèÖÃÎª¸º£¨Ë³Ê±Õë²¹³¥£©
				// 				//²¹³¥Öµ£¬ÄÜÈÃ·É»úÎóÒÔÎª³¯Ïà·´·½Ïò×ª¶¯£¬×îÖÕµÃÒÔÐÞÕýÆ«º½½ÇËÙ¶ÈÆ¯ÒÆ
				// 				imu->gyroZ_offset1 = -Aux_Rc.offse_yaw; //Aux_Rc.offse_yaw=10±íÊ¾ÄæÊ±Õë²¹³¥10
				// }
				// else{
				// 				imu->gyroZ_offset1 = 0; //Æ½Ê±Îª0£¬Æ½Ê±Îª¾²Ì¬Öµ£¬Èç¹ûÓÐÎó²î¿ÉÒÔÍ¨¹ý ÍÓÂÝÒÇÐ£×¼À´Ïû³ý£¬Ò£¿Ø³¤°´K2£¨ÓÍÃÅÖµ·Åµ½×îµÍ£©
				// }
				
				
				
				//
				
				
			#endif
				
			//¸©Ñö½Ç·¶Î§Ö»ÓÐ£¨-90~90£©£¬ÎÞ·¨Çø·Ö·É»ú³¯ÉÏ»¹ÊÇ³¯ÏÂ¡£
			//³¯ÉÏÊ±	(-90~90)
			//	F   30¶È            B -30¶È
			//	   .             .    
			//	 .    B       F   .
			//³¯ÏÂÊ±(-90~90)     
		  //       .     F      B     .
			//	       .              .
		  //     B                     F
			//·­¹ö½Ç¿ÉÒÔÇø·Ö³¯ÉÏºÍ³¯ÏÂ£¬³¯ÉÏÊ±£¨0 ~ +/-90£©£¬³¯ÏÂÊ±(+/-90 ~ +/-180)

			//pAngE->pitch  =  asin(vecxZ)* RtA;	 //¸©Ñö½Ç	 0~+/-90	
			imu->pitch  =  -asin(vecxZ)* RtA;	 //¸©Ñö½Ç	 0~+/-90
				
			//pAngE->tmp = pAngE->pitch;
				
			imu->roll	= atan2f(vecyZ, veczZ) * RtA;	//ºá¹ö½Ç 0~+/-180
			NormAccz = imu->acc[0]* vecxZ + imu->acc[1] * vecyZ + imu->acc[2] * veczZ;		
	}
}

float GetAccz(void)
{
	return NormAccz;
}