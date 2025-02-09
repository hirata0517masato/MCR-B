/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �}�C�R���J�[�L�b�gVer.5.1 �g���[�X��{�v���O����(R8C/38A��) */
/* �o�[�W����   Ver.1.01                                                    */
/* Date         2013.04.18                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/
/*
���̃v���O�����́A���L��ɑΉ����Ă��܂��B
�ERY_R8C38�{�[�h
�E���[�^�h���C�u���Ver.5
�E�Z���T���Ver.5
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "printf_lib.h"                 /* printf�g�p���C�u����         */


/*======================================*/
/* �V���{����`                         */
/*======================================*/

/* �萔�ݒ� */
#define PWM_CYCLE       	9999           /* �쓮���[�^PWM�̎���       1ms       								*/
#define SERVO_PWM_CYCLE     10//16			   /* �T�[�{���[�^PWM�̎���     PWM_CYCLE * SERVO_PWM_CYCLE (ms)         */
#define HANDLE_STEP     	14//22              /* 1�K���̒l                    */
#define SERVO_CENTER    	16760          /* �T�[�{�̃Z���^�l             */


#define RUN_TIME	40000	//���s����

//#define WallOn //���[���`�F���W�̕ǂ�����Ƃ��ɗL�������邱��



/* �}�X�N�l�ݒ� �~�F�}�X�N����(����)�@���F�}�X�N����(�L��) */
#define MASK2_2         0x66            /* �~�����~�~�����~             */
#define MASK2_0         0x60            /* �~�����~�~�~�~�~             */
#define MASK0_2         0x06            /* �~�~�~�~�~�����~             */
#define MASK3_3         0xe7            /* �������~�~������             */
#define MASK0_3         0x07            /* �~�~�~�~�~������             */
#define MASK3_0         0xe0            /* �������~�~�~�~�~             */
#define MASK4_0         0xf0            /* ���������~�~�~�~             */
#define MASK0_4         0x0f            /* �~�~�~�~��������             */
#define MASK4_4         0xff            /* ����������������             */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
void timer( unsigned long timer_set );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char startbar_get( void );
void led_out( unsigned char led );
void motor( int accele_l, int accele_r );
void handle( int angle );


/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p                  */
unsigned long   cnt1;                   /* main���Ŏg�p                 */
unsigned long   run_time = 0;
int             pattern;                /* �p�^�[���ԍ�                 */

int angle_buf = 0;

unsigned long S_cnt = 0,LR_cnt = 0;

int s_motor = 0; //�����p���[�^�o��

unsigned long OUT_cnt = 0;

unsigned char out_flag = 0; //�R�[�X�A�E�g�����ꍇ�͂P


#define LOG_MAX 4025
signed char log[LOG_MAX] = {};
int log_num = 0;
int log_start = 0;
signed char log_buf_Lmotor,log_buf_Rmotor;
signed char log_buf_handle;


/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
   	int     i;
	int M = 0;
	int Out_cnt = 0;

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
	init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */

    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    motor( 0, 0 );
/*
	while(1){
		if(cnt0 > 500){
			cnt0 = 0;
		 	printf("%x \n", sensor_inp(MASK4_4)	);
		}
	}
*/	
    while( 1 ) {

	if(pattern > 10 && run_time > RUN_TIME){
		pattern = 99;
		cnt1 = 0;
	}
	
	if(pattern > 10 && check_crossline() ) {
		if(OUT_cnt > 1){
			OUT_cnt = 0;
			Out_cnt++;
			
			if(Out_cnt > 500){// 500ms�N�����N���肪��������R�[�X�A�E�g
				pattern = 99;
				out_flag = 1; //�R�[�X�A�E�g
				cnt1 = 0;
			}
		}
	}else{
		OUT_cnt = 0;
		Out_cnt = 0;
	}
	
    switch( pattern ) {

    /*****************************************************************
    �p�^�[���ɂ���
     0�F�X�C�b�`���͑҂�
     1�F�X�^�[�g�o�[���J�������`�F�b�N
    11�F�ʏ�g���[�X
    12�F�E�֑�Ȃ��̏I���̃`�F�b�N
    13�F���֑�Ȃ��̏I���̃`�F�b�N
    21�F�N���X���C�����o���̏���
    22�F�N���X���C����ǂݔ�΂�
    23�F�N���X���C����̃g���[�X�A�N�����N���o
    31�F���N�����N�N���A�����@���肷��܂ŏ����҂�
    32�F���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N
    41�F�E�N�����N�N���A�����@���肷��܂ŏ����҂�
    42�F�E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N
    51�F�E�n�[�t���C�����o���̏���
    52�F�E�n�[�t���C����ǂݔ�΂�
    53�F�E�n�[�t���C����̃g���[�X�A���[���`�F���W
    54�F�E���[���`�F���W�I���̃`�F�b�N
    61�F���n�[�t���C�����o���̏���
    62�F���n�[�t���C����ǂݔ�΂�
    63�F���n�[�t���C����̃g���[�X�A���[���`�F���W
    64�F�����[���`�F���W�I���̃`�F�b�N
    *****************************************************************/

    case 0:
        /* �X�C�b�`���͑҂� */
        if( pushsw_get() ) {
            pattern = 1;

			timer(1000);/////////////////////////////////////////////////////////////////////////////////////////////////
		
            cnt1 = 0;
            break;
        }
        if( cnt1 < 100 ) {              /* LED�_�ŏ���                  */
            led_out( 0x1 );
        } else if( cnt1 < 200 ) {
            led_out( 0x2 );
        } else {
            cnt1 = 0;
        }
        break;

    case 1:
        /* �X�^�[�g�o�[���J�������`�F�b�N */
        if( !startbar_get() ) {
            /* �X�^�[�g�I�I */
            led_out( 0x0 );
            pattern = 11;
            run_time = 0;
            cnt1 = 0;
			log_start = 1;  //���O�擾�J�n
            break;
        }
        if( cnt1 < 50 ) {              /* LED�_�ŏ���                   */
            led_out( 0x1 );
        } else if( cnt1 < 100 ) {
            led_out( 0x2 );
        } else {
            cnt1 = 0;
        }
        break;

    case 11:
		led_out( 0 );
		
        /* �ʏ�g���[�X */
        if( check_crossline() ) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // �E�n�[�t���C���`�F�b�N       
            pattern = 51;
            break;
        }
        if( check_leftline() ) {        // ���n�[�t���C���`�F�b�N       
            pattern = 61;
            break;
        }

		
		if(S_cnt > 1000){//�w�莞�Ԉȏ㒼�����������猸������i�G���R�[�_�ł̑��x���䕗�j T600x5 ��500���炢
			s_motor = 90;
		}else{
			s_motor = 100;
		}
		
        switch( sensor_inp(MASK4_4) ) {
            case 0x18: //0001 1000
                /* �Z���^���܂����� */
				
				handle( 0 );
                motor( s_motor ,s_motor );
                break;
			///////////////////////////////////////////////////////////�E�Ȃ�
			
			case 0x08://0000 1000
				handle( 10 );
                motor( s_motor ,s_motor );
                break;
			
			case 0x0c://0000 1100
				handle( 15 );
                motor( s_motor ,s_motor );
                break;
				
				
            case 0x04://0000 0100
                handle( 65 );
                motor( s_motor ,s_motor );
                break;

            case 0x06://0000 0110
                handle( 95 );
                motor( s_motor ,s_motor );
				
				pattern = 12;
				LR_cnt = 0;
                break;
				
			case 0x02://0000 0010
                handle( 115 );
                motor( s_motor ,s_motor );
				
				pattern = 12;
				LR_cnt = 0;
                break;
				
			case 0x07://0000 0111
                handle( 120 );
                motor( s_motor ,s_motor -5 );
				
				pattern = 12;
				LR_cnt = 0;
                break;

            case 0x03://0000 0011
                handle( 120 );
                motor( s_motor ,s_motor -5 );
				
                pattern = 12;
				LR_cnt = 0;
                break;
			
			case 0x83://1000 0011
                handle( 120 );
                motor( s_motor ,s_motor -5);
				
                pattern = 12;
				LR_cnt = 0;
                break;

			///////////////////////////////////////////////////////���Ȃ�
			
			case 0x10://0001 0000
                handle( -10 );
                motor( s_motor ,s_motor );
                break;
			
			case 0x30://0011 0000
                handle( -15 );
                motor( s_motor ,s_motor );
                break;
				
            case 0x20://0010 0000
                handle( -65 );
                motor( s_motor ,s_motor );
                break;

            case 0x60://0110 0000
                handle( -95 );
                motor( s_motor ,s_motor );
				
				pattern = 13;
				LR_cnt = 0;
                break;
			
			 case 0x40://0100 0000
                handle( -115 );
               	motor( s_motor ,s_motor );
				
				pattern = 13;
				LR_cnt = 0;
                break;
			
			
			 case 0xe0://1110 0000
                handle( -120 );
                motor( s_motor -5 ,s_motor );
				
				pattern = 13;
				LR_cnt = 0;
                break;

            case 0xc0://1100 0000
                handle( -120 );
                motor( s_motor -5 ,s_motor );
				
                pattern = 13;
				LR_cnt = 0;
                break;
				
			case 0xc1://1100 0001
                handle( -120 );
                motor( s_motor -5 ,s_motor );
				
                pattern = 13;
				LR_cnt = 0;
                break;
				
            default:
                break;
        }
        break;

    case 12://�E���
		
		led_out( 3 );
		
		
        if( check_crossline() ) {       // ��Ȃ������N���X���C���`�F�b�N 
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // �E�n�[�t���C���`�F�b�N       
            pattern = 51;
            break;
        }
	 
        if( check_leftline() ) {        // ���n�[�t���C���`�F�b�N       
            pattern = 61;
            break;
        }
		
		
		if(LR_cnt < 20){
			handle( 100 );
       		motor( 100 ,-100 );
			
		}else if((sensor_inp(MASK4_4)&0x20) != 0x00 ){ //xx1x xxxx �O��肷��
		//if((sensor_inp(MASK4_4)&0x10) != 0x00 ){ //xxx1 xxxx �O��肷��
			led_out( 1 );
			
			handle( 120 );
       		motor( 100 ,-100 );
		
			
		}else if((sensor_inp(MASK4_4)&0x40) != 0x00 ){ //x1xx xxxx �O��肷��
		//}else if((sensor_inp(MASK4_4)&0x20) != 0x00 ){ //xx1x xxxx �O��肷��
		
			led_out( 1 );
			
			if(LR_cnt < 200){
				handle( 120 );
       			motor( 100 ,30 );
				
			}else if(LR_cnt < 400){
				handle( 110 );
       			motor( 100 , 90 );
					
			}else{
				handle( 110 );
       			motor( 100 , 90 );
			}
			
		}else if((sensor_inp(MASK4_4)&0x80) != 0x00 ){ //1xxx xxxx �O��肷��
		//}else if((sensor_inp(MASK4_4)&0x40) != 0x00 ){ //x1xx xxxx �O��肷��
		
			led_out( 1 );
			
			if(LR_cnt < 500){
				handle( 110 );
       			motor( 100 ,95 );
				
			}else if(LR_cnt < 1000){
				handle( 110 );
       			motor( 100 ,100 );
				
			}else{
				handle( 110 );
       			motor( 100 ,100 );
			}
			

		}else if(LR_cnt < 400 ){ //�J�[�u�O�� 
		
			handle( 110 );
       		motor( 100 ,100 );
			
		
		}else if(LR_cnt < 800 ){ //�J�[�u����
		
			handle( 110 );
        	motor( 100 ,100 );
			
		
		}else{//�J�[�u�㔼
		
			handle( 110 );
        	motor( 100 ,100 );
			
		}
		
				
		/* �E�֑�Ȃ��̏I���̃`�F�b�N */
        if( sensor_inp(MASK4_4) == 0x0c || sensor_inp(MASK4_4) == 0x04 ) {//0000 1100  //0000 0100
		//if( (sensor_inp(MASK4_4) == 0x0c )) {//0000 1100
			

				pattern = 11;
			
				S_cnt = 0;
	
        }

        break;

    case 13: //�����
	
		led_out( 3 );
	
		
        if( check_crossline() ) {       // ��Ȃ������N���X���C���`�F�b�N 
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // �E�n�[�t���C���`�F�b�N       
            pattern = 51;
            break;
        }
		
        if( check_leftline() ) {        // ���n�[�t���C���`�F�b�N       
            pattern = 61;
            break;
        }
	   		
		if(LR_cnt < 20){
			
			handle( -100 );
       		motor( -100 ,100 );
			
		}else if((sensor_inp(MASK4_4)&0x04) != 0x00 ){ //xxxx x1xx �O��肷��
		//if((sensor_inp(MASK4_4)&0x08) != 0x00 ){ //xxxx 1xxx �O��肷��
		
			led_out( 1 );
			
			handle( -120 );
       		motor( -100 ,100 );
				
		}else if((sensor_inp(MASK4_4)&0x02) != 0x00 ){ //xxxx xx1x �O��肷��
		//}else if((sensor_inp(MASK4_4)&0x04) != 0x00 ){ //xxxx x1xx �O��肷��
		
			led_out( 1 );
			
			if(LR_cnt < 200){
				handle( -120 );
       			motor( 30 ,100 );
				
			}else if(LR_cnt < 400){
				handle( -110 );
       			motor( 90 ,100 );
					
			}else{
				handle( -110 );
       			motor( 90 ,100 );
			}
			
		}else if((sensor_inp(MASK4_4)&0x01) != 0x00 ){ //xxxx xxx1 �O��肷��
		//}else if((sensor_inp(MASK4_4)&0x02) != 0x00 ){ //xxxx xx1x �O��肷��
		
			led_out( 1 );
			
			if(LR_cnt < 500){
				handle( -110 );
       			motor( 95 ,100 );
				
			}else if(LR_cnt < 1000){
				handle( -110 );
       			motor( 100 ,100 );
					
			}else{
				handle( -110 );
       			motor( 100 ,100 );
			}
	
		}else if(LR_cnt < 400 ){ //�J�[�u�O�� 
		
			handle( -110 );
       		motor( 100 ,100 );
			
		
		}else if(LR_cnt < 800){ //�J�[�u����
		
			handle( -110 );
        	motor( 100 ,100 );
			
			
		}else{//�J�[�u�㔼
		
			handle( -110 );
        	motor( 100 ,100 );
			
		}
		
		
		 /* ���֑�Ȃ��̏I���̃`�F�b�N */		
        if( (sensor_inp(MASK4_4) == 0x30) || (sensor_inp(MASK4_4) == 0x20) ) {//0011 0000  //0010 0000 
		//if( (sensor_inp(MASK4_4) == 0x30) ) {//0011 0000  
	//	if( (sensor_inp(MASK4_4) == 0x18) || (sensor_inp(MASK4_4) == 0x10) ) {//0001 1000  //001 0000  

				pattern = 11;
			
				S_cnt = 0;
		
        }

        break;

    case 21:
        /* �P�{�ڂ̃N���X���C�����o���̏��� */
        led_out( 0x3 );
        handle( 0 );
		
		if(S_cnt > 800){
        	motor( -100 ,-100 );
			
		}else if(S_cnt > 400){
			motor( -50 ,-50 );
			
		}else if(S_cnt > 200){
			motor( -20 ,-20 );
		}else{
			motor( 0 ,0 );	
		}
		
        pattern = 22;
        cnt1 = 0;
        break;

    case 22:
        /* �Q�{�ڂ�ǂݔ�΂� */
		
	
		if( cnt1 > 100 ) {
	       pattern = 23;
	       cnt1 = 0;
	    }
		
        break;

    case 23:
        /* �N���X���C����̃g���[�X�A�N�����N���o */
        if( check_leftline() ){// || ((sensor_inp(MASK4_4)&0xc0) == 0xc0)) {
            /* ���N�����N�Ɣ��f�����N�����N�N���A������ */
            led_out( 0x1 );
            handle( -150 );
            motor( -100 ,100 );
            pattern = 31;
            cnt1 = 0;
            break;
        }
        if( check_rightline() ){// || ((sensor_inp(MASK4_4)&0x03) == 0x03)) {
            /* �E�N�����N�Ɣ��f���E�N�����N�N���A������ */
            led_out( 0x2 );
            handle( 150 );
            motor( 100 ,-100 );
            pattern = 41;
            cnt1 = 0;
            break;
        }

        if(cnt1 > 90){
			M = 90;
		}else if(cnt1 > 80){
			M = 100;
		}else if(cnt1 > 70){
			M = 90;
		}else if(cnt1 > 60){
			M = 80;
		}else if(cnt1 > 50){
			M = 70;
		}else if(cnt1 > 40){
			M = 60;
		}else{
			M = 50;
		}
		
		switch( sensor_inp(MASK4_4) ) {
            case 0x18: //0001 1000
                /* �Z���^���܂����� */

				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x08://0000 1000
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 8 );
                motor( M ,M );
                break;
				
			case 0x0C://0000 1100
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 10 );
                motor( M ,M );
                break;
				
            case 0x04://0000 0100
                /* �����ɍ���聨�E�֔��Ȃ� */
   
				handle( 15 );
                motor( M ,M-10 );
                break;

            case 0x06://0000 0110
                /* ��������聨�E�֏��Ȃ� */
                handle( 20 );
                motor( M ,M-40 );
				
                break;

			case 0x07://0000 0111
                /* ��������聨�E�֏��Ȃ� */
                handle( 30 );
                motor( M ,M-40 );
                break;

            case 0x03://0000 0011
                /* �傫������聨�E�֑�Ȃ� */
                handle( 30 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x10://0001 0000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -8 );
                motor( M ,M );
                break;
				
				
			case 0x30://0011 0000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -10 );
                motor( M ,M );
                break;
				
            case 0x20://0010 0000
                /* �����ɉE��聨���֔��Ȃ� */
      	
				handle( -15 );
                motor( M-10 ,M );
                break;

            case 0x60://0110 0000
                /* �����E��聨���֏��Ȃ� */
				handle( -20 );
                motor( M-40 ,M );
                break;
			case 0xe0://1110 0000
                /* �����E��聨���֏��Ȃ� */
				handle( -30 );
                motor( M-40 ,M );
                break;

            case 0xc0://1100 0000
                /* �傫���E��聨���֑�Ȃ� */
                handle( -30 );
                motor( M-40 ,M );
                break;
			
        }
  
        break;


    case 31:
		if( sensor_inp(MASK4_4)& 0x38 != 0x00 ) { //xx11 1xxx �O���
			handle( -180 );
        	
			if(cnt1 > 100){
				motor( 0 ,100 );	
			}else{
				motor( -100 ,100 );	
			}
			
		}else{
			handle( -160 );
        	
			if(cnt1 > 100){
				motor( 0 ,100 );	
			}else{
				motor( -100 ,100 );	
			}
		}
			
        /* ���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if(cnt1 > 50 &&  (sensor_inp(MASK3_3) == 0xC0 || sensor_inp(MASK3_3) == 0x40 || sensor_inp(MASK3_3) == 0x80)){ //1100 0000  //0100 0000  //1000 0000
            
            pattern = 32;
            cnt1 = 0;
        }
		
	
        break;
        
	case 32:
		if( sensor_inp(MASK4_4)& 0x04 != 0x00 ) { //xxxx x1xx �O���
			handle( -150 );
        	motor(  90 ,100 );
		}else{
			handle( -130 );
        	motor( 90 ,100 );
		}
		
        /* ���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if(cnt1 > 5 && ( sensor_inp(MASK3_3) == 0x60 ||  sensor_inp(MASK3_3) == 0x20 )){ //0110 0000 //0010 0000
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }

        break;
		
    case 41:
        if( sensor_inp(MASK4_4)& 0x1C != 0x00 ) { //xxx1 11xx �O���
			handle( 180 );
			
        	if(cnt1 > 100){
				motor( 100 ,0 );	
			}else{
				motor( 100 ,-100 );	
			}
		}else{
			handle( 160 );
			
			if(cnt1 > 100){
				motor( 100 ,0 );	
			}else{
				motor( 100 ,-100 );	
			}
        	
		}
			
        /* �E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if(cnt1 > 50 && ( sensor_inp(MASK3_3) == 0x03 || sensor_inp(MASK3_3) == 0x02 || sensor_inp(MASK3_3) == 0x01) ){ //0000 0011  //0000 0010 //0000 0001
            
            pattern = 42;
            cnt1 = 0;
        }
		

        break;

    case 42:
		if( sensor_inp(MASK4_4)& 0x20 != 0x00 ) { //xx1x xxxx �O���
        	handle( 150 );
        	motor( 100 , 90 );
		}else{
			handle( 130 );
        	motor( 100 ,90 );
		}
		
        /* �E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if(cnt1 > 5 &&  ( sensor_inp(MASK3_3) == 0x06 || sensor_inp(MASK3_3) == 0x04) ){ //0000 0110 //0000 0100
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }

        break;


    case 51:
        /* �P�{�ڂ̉E�n�[�t���C�����o���̏��� */
        led_out( 0x2 );
        handle( 0 );
        motor( 0 ,0 );
        pattern = 52;
        cnt1 = 0;
		
		if( check_crossline() || check_leftline() ){// || (sensor_inp(MASK4_4)&0x80) == 0x80) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
        break;

    case 52:
        /* �Q�{�ڂ�ǂݔ�΂� */
        if( cnt1 > 30 ) {
            pattern = 53;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_leftline()){// || (sensor_inp(MASK4_4)&0x80) == 0x80) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
       
        break;

    case 53:
        /* �E�n�[�t���C����̃g���[�X�A���[���`�F���W */
        if( sensor_inp(MASK4_4) == 0x00 ) {
#ifdef WallOn
            handle( 130 );
            motor( 100 ,0 );
#else
			handle( 130 );
			motor( 100 ,0 );
#endif
            pattern = 54;
            cnt1 = 0;
            break;
        }

#ifdef WallOn
		M = 90;
#else
		M = 100;
#endif
		

        switch( sensor_inp(MASK4_4) ) {
            case 0x30: //0011 0000
                /* �Z���^���܂����� */
				
				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x10://0001 0000
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 15 );
                motor( M ,M );
                break;
			
			case 0x18://0001 1000
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 20 );
                motor( M ,M );
                break;
					
            case 0x08://0000 1000
                /* �����ɍ���聨�E�֔��Ȃ� */
   
				handle( 25 );
                motor( M ,M-10 );
                break;

            case 0x0c://0000 1100
                /* ��������聨�E�֏��Ȃ� */
                handle( 30 );
                motor( M ,M-40 );
				
                break;
			
			case 0x0e://0000 1110
                /* ��������聨�E�֏��Ȃ� */
                handle( 35 );
                motor( M ,M-40 );
                break;

            case 0x06://0000 0110
                /* �傫������聨�E�֑�Ȃ� */
                handle( 40 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x20://0010 0000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -15 );
                motor( M ,M );
                break;
			
			case 0x60://0110 0000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -20 );
                motor( M ,M );
                break;
					
            case 0x40://0100 0000
                /* �����ɉE��聨���֔��Ȃ� */
      	
				handle( -25 );
                motor( M-10 ,M );
                break;

            case 0xc0://1100 0000
                /* �����E��聨���֏��Ȃ� */
				handle( -30 );
                motor( M-40 ,M );
                break;
		/*	case 0xe0://1100 0000
                // �����E��聨���֏��Ȃ� 
				handle( -30 );
                motor( M-40 ,M );
                break;
*/
            case 0x80://1000 0000
                /* �傫���E��聨���֑�Ȃ� */
                handle( -35 );
                motor( M-40 ,M );
                break;
        }
        break;

    case 54:
/*
#ifdef WallOn
 
#else
			handle( 130 );
			
			if(cnt1 < 50){
				motor( 100 ,-30 );
			}else{
            	motor( 100 ,100 );
			}
#endif
*/

/*		
		if(cnt1 > 120){
			handle( 0 );
			motor( 50 ,100 );
		}
*/
        /* �E���[���`�F���W�I���̃`�F�b�N */
        //if( sensor_inp( MASK4_4 ) == 0x01 || sensor_inp( MASK4_4 ) == 0x03 ) { //0000 0001  //0000 0011
		if(cnt1 > 5 && sensor_inp( MASK4_4 )&0x01 != 0x00 ) { //xxxx xxx1
		//if(cnt1 > 100 && sensor_inp( MASK4_4 )&0x02 != 0x00 ) { //xxxx xx1x
#ifdef WallOn
            handle( -100 );
            motor( 10 ,100 );
#else
            handle( -130 );
            motor( -50 ,100 );
#endif
		     
            pattern = 55;
            cnt1 = 0;
        }
        break;
			
    case 55:
        /* �E���[���`�F���W�I���̃`�F�b�N */
        //if( sensor_inp( MASK4_4 ) == 0x60 ) { //0110 0000
		//if(cnt1 > 100 && sensor_inp( MASK4_4 )&0x60 != 0x00 ) { //x11x xxxx
		if(cnt1 > 5 && sensor_inp( MASK4_4 )&0xc0 != 0x00 ) { //11xx xxxx
            led_out( 0x0 );

#ifdef WallOn
			handle( -100 );
            motor( 10 ,100 );
#else
			handle( -100 );
            motor( 10 ,100 );
#endif			

          
            pattern = 56;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;
		
	 case 56:
        /* �E���[���`�F���W�I���̃`�F�b�N */
        //if( sensor_inp( MASK4_4 ) == 0x60 ) { //0110 0000
		if( sensor_inp( MASK4_4 )&0x0c != 0x00 ) { //xxxx 11xx
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;
			
	
    case 61:
        /* �P�{�ڂ̍��n�[�t���C�����o���̏��� */
        led_out( 0x1 );
        handle( 0 );
        motor( 0 ,0 );
        pattern = 62;
        cnt1 = 0;
		
		if( check_crossline() || check_rightline()){// || (sensor_inp(MASK4_4)&0x01) == 0x01) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
        break;

    case 62:
        /* �Q�{�ڂ�ǂݔ�΂� */
        if( cnt1 > 30 ) {
            pattern = 63;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_rightline()){//|| (sensor_inp(MASK4_4)&0x01) == 0x01) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
        break;

    case 63:
        /* ���n�[�t���C����̃g���[�X�A���[���`�F���W */
        if(cnt1 > 100 &&  sensor_inp(MASK4_4) == 0x00 ) {

#ifdef WallOn
            handle( -80 );
            motor( 0 ,100 );
#else
            handle( -80 );
			motor( 0 ,100 );
#endif

            pattern = 64;
            cnt1 = 0;
            break;
        }
        
#ifdef WallOn
		M = 90;
#else
		M = 100;
#endif
		switch( sensor_inp(MASK4_4) ) {
            case 0x0c: //0000 1100
                /* �Z���^���܂����� */
				
				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x04://0000 0100
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 15 );
                motor( M ,M );
                break;
			
			case 0x06://0000 0110
                /* �����ɍ���聨�E�֔��Ȃ� */

				handle( 20 );
                motor( M ,M );
                break;
					
            case 0x02://0000 0010
                /* �����ɍ���聨�E�֔��Ȃ� */
   
				handle( 25 );
                motor( M ,M-10 );
                break;

            case 0x03://0000 0011
                /* ��������聨�E�֏��Ȃ� */
                handle( 30 );
                motor( M ,M-40 );
				
                break;
			
		/*	case 0x07://0000 0011
                // ��������聨�E�֏��Ȃ� 
                handle( 30 );
                motor( M ,M-40 );
                break;
*/
            case 0x01://0000 0001
                /* �傫������聨�E�֑�Ȃ� */
                handle( 35 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x08://0000 1000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -15 );
                motor( M ,M );
                break;
			
			case 0x18://0001 1000
                /* �����ɉE��聨���֔��Ȃ� */
	
				handle( -20 );
                motor( M ,M );
                break;
					
            case 0x10://0001 0000
                /* �����ɉE��聨���֔��Ȃ� */
      	
				handle( -25 );
                motor( M-10 ,M );
                break;

            case 0x30://0011 0000
                /* �����E��聨���֏��Ȃ� */
				handle( -30 );
                motor( M-40 ,M );
                break;
			case 0x70://0111 0000
                /* �����E��聨���֏��Ȃ� */
				handle( -35 );
                motor( M-40 ,M );
                break;

            case 0x60://0110 0000
                /* �傫���E��聨���֑�Ȃ� */
                handle( -40 );
                motor( M-40 ,M );
                break;
        }
        break;

	case 64:
 
 /*
#ifdef WallOn  
 
#else    
        handle( -80 );
			 
		if(cnt1 < 50){
			motor( -30 ,100 );
		}else{
            motor( 100 ,100 );
		}
#endif
*/	
/*		
		if(cnt1 > 120){
			handle( 0 );
			motor( 100 ,50 );
		}
*/		
        //if( sensor_inp( MASK4_4 ) == 0x80 || sensor_inp( MASK4_4 ) == 0xC0) { //1000 0000  //1100 0000
		if(cnt1 > 5 &&  sensor_inp( MASK4_4 )&0x80 != 0x00) { //1xxx xxxx
		//if(cnt1 > 100 &&  sensor_inp( MASK4_4 )&0x40 != 0x00) { //x1xx xxxx

#ifdef WallOn
			handle( 130 );
            motor( 100 ,-60 );
#else
			handle( 130 );
            motor( 100 ,-60 );
#endif
	  
            pattern = 65;
            cnt1 = 0;
        }
        break;
			
    case 65:
        /* �����[���`�F���W�I���̃`�F�b�N */
        //if( sensor_inp( MASK4_4 ) == 0x06 ) { //0000 0110
		//if(cnt1 > 100 &&  sensor_inp( MASK4_4 )&0x06 != 0x00) { //xxxx x11x
		if(cnt1 > 5 &&  sensor_inp( MASK4_4 )&0x03 != 0x00) { //xxxx xx11
            led_out( 0x0 );

#ifdef WallOn
			handle( 130 );
            motor( 100 ,-60 ); //�����F���͐؂�Ԃ��x���̂�-100�Ƃ���
#else
			handle( 130 );
            motor( 100 ,-60 ); 
#endif
			

       
            pattern = 66;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;
	
	 case 66:
        /* �����[���`�F���W�I���̃`�F�b�N */
        //if( sensor_inp( MASK4_4 ) == 0x06 ) { //0000 0110
		if( sensor_inp( MASK4_4 )&0x30 != 0x00) { //xx11 xxxx
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;

	case 99://���s�I��
		 motor( 0 ,0 );
		 log_start = 0;//���O�̕ۑ��I��
		 
		 if(pushsw_get()){//�X�C�b�`���������ƃ��O�o�̓��[�h
			 
			 log_num = 0;
			 printf( "pattern ,sensor, handle ,  Lmotor , Rmotor\n");
			 
			 while(log_num < LOG_MAX){
				printf( "%3d,0x%02x,%4d,%4d,%4d\n", log[log_num]
													,(unsigned char)log[log_num+1]&0x00ff
													,log[log_num+2]
													,log[log_num+3],log[log_num+4] );
				
				if(log[log_num] == 0 && log[log_num+1] == 0)break;
				log_num += 5;	 
			 }
		 }
		 
		 if(cnt1 < 10000){
	        switch( sensor_inp(MASK4_4) ) {
	            case 0x18: //0001 1000
	                /* �Z���^���܂����� */
				
					handle( 0 );
	               // if(cnt1 < 1000) motor( 100 ,100 );
	                break;
				///////////////////////////////////////////////////////////
			
				case 0x08://0000 1000
	                /* �����ɍ���聨�E�֔��Ȃ� */
		
					handle( 8 );
	               // if(cnt1 < 1000)motor( 100 ,100 );
	                break;
			
				case 0x0c://0000 1100
	                /* �����ɍ���聨�E�֔��Ȃ� */
		
					handle( 15 );
	               // if(cnt1 < 1000)motor( 100 ,100 );
	                break;
				
				
	            case 0x04://0000 0100
	                /* �����ɍ���聨�E�֔��Ȃ� */
      
					handle( 30 );
	               // if(cnt1 < 1000)motor( 100 ,90 );
	                break;

	            case 0x06://0000 0110
	                /* ��������聨�E�֏��Ȃ� */
	                handle( 40 );
	              //  if(cnt1 < 1000)motor( 100 ,90 );
				
	                break;
				
				case 0x02://0000 0010
	                /* ��������聨�E�֏��Ȃ� */
	                handle( 50 );
	              //  if(cnt1 < 1000)motor( 100 ,85 );
				
	                break;
				
				case 0x07://0000 0111
	                /* ��������聨�E�֏��Ȃ� */
	                handle( 60 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
			
	                break;

	            case 0x03://0000 0011
	                /* �傫������聨�E�֑�Ȃ� */
					handle( 65 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
	             
	                break;
			
				case 0x83://1000 0011
	                /* �傫������聨�E�֑�Ȃ� */
					handle( 65 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
	             
	                break;

				///////////////////////////////////////////////////////
			
				case 0x10://0001 0000
	                /* �����ɉE��聨���֔��Ȃ� */
			
					handle( -8 );
	              //  if(cnt1 < 1000)motor( 100 ,100 );
	                break;
			
				case 0x30://0011 0000
	                /* �����ɉE��聨���֔��Ȃ� */
			
					handle( -15 );
	             //   if(cnt1 < 1000)motor( 100 ,100 );
	                break;
				
	            case 0x20://0010 0000
	                /* �����ɉE��聨���֔��Ȃ� */
      		
					handle( -30 );
	             //   if(cnt1 < 1000)motor( 90 ,100 );
	                break;

	            case 0x60://0110 0000
	                /* �����E��聨���֏��Ȃ� */
					handle( -40 );
	             //   if(cnt1 < 1000)motor( 90 ,100 );
	                break;
			
				 case 0x40://0100 0000
	                /* �����E��聨���֏��Ȃ� */
					handle( -50 );
	             //   if(cnt1 < 1000)motor( 85 ,100 );
	                break;
			
			
				 case 0xe0://1110 0000
	                /* �����E��聨���֏��Ȃ� */
					handle( -60 );
	              //  if(cnt1 < 1000)motor( 80 ,100 );
		
	                break;

	            case 0xc0://1100 0000
	                /* �傫���E��聨���֑�Ȃ� */
					handle( -65 );
	             //   if(cnt1 < 1000)motor( 80 ,100 );
	        
	                break;
				
				case 0xc1://1100 0001
	                /* �傫���E��聨���֑�Ȃ� */
					handle( -65 );
	             //   if(cnt1 < 1000)motor( 80 ,100 );
	         
	                break;
				
	            default:
	                break;
	        }
		 }else{
			 //handle( 0 );
		 }
		
		break;

    default:
        /* �ǂ�ł��Ȃ��ꍇ�͑ҋ@��Ԃɖ߂� */
        pattern = 0;
        break;
    }
    }
}

/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void )
{
    int i;

    /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
    prc0  = 1;                          /* �v���e�N�g����               */
    cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
    cm05  = 0;                          /* XIN�N���b�N���U              */
    for(i=0; i<50; i++ );               /* ���肷��܂ŏ����҂�(��10ms) */
    ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
    prc0  = 0;                          /* �v���e�N�gON                 */

    /* �|�[�g�̓��o�͐ݒ� */
    prc2 = 1;                           /* PD0�̃v���e�N�g����          */
    pd0 = 0x00;                         /* 7-0:�Z���T���Ver.5          */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:���[�^�h���C�u���Ver.5  */
    pd3 = 0xff;                         /*                              */
    p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0xff;                         /*                              */
    pd6 = 0xff;                         /*                              */
    pd7 = 0xff;                         /*                              */
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3�`P1_0�̃v���A�b�vON     */

    /* �^�C�}RB�̐ݒ� */
    /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
    trbpre = 200-1;                     /* �v���X�P�[�����W�X�^         */
    trbpr  = 100-1;                     /* �v���C�}�����W�X�^           */
    trbic  = 0x07;                      /* ���荞�ݗD�惌�x���ݒ�       */
    trbcr  = 0x01;                      /* �J�E���g�J�n                 */

    /* �^�C�}RD ���Z�b�g����PWM���[�h�̐ݒ�*/
    /* PWM���� = 1 / 20[MHz]   * �J�E���g�\�[�X * (TRDGRA0+1)
               = 1 / (20*10^6) * 8              * 40000
               = 0.016[s] = 16[ms]
    */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
//    trdcr0  = 0x23;                     /* �\�[�X�J�E���g�̑I��:f8      */
	trdcr0  = 0x21;                     /* �\�[�X�J�E���g�̑I��:f2      */
    trdgra0 = trdgrc0 = PWM_CYCLE;          /* ����                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON���ݒ�           */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON���ݒ�           */
//	trdgrb1 = trdgrd1 = SERVO_CENTER;           /* P2_5�[�q��ON���ݒ�           */
    trdgrb1 = trdgrd1 = PWM_CYCLE+2;           /* P2_5�[�q��ON���ݒ�           */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRB(vect=24)
void intTRB( void )
{
    static int servo_cnt = 0,servo_angle = 0,cnt10ms = 0;

	if(servo_cnt == 0){
		servo_angle = angle_buf ;	
	}
	
	if(out_flag == 1)trdgrd1 = 0; //�R�[�X�A�E�g�̏ꍇ�̓T�[�{�o�͂��I�t�ɂ���
	else if(servo_angle >= (PWM_CYCLE)) trdgrd1 = PWM_CYCLE+2;
	else if(servo_angle > 0 )trdgrd1 = servo_angle;
	else trdgrd1 = 0;
	
		
	servo_angle -=  PWM_CYCLE;
	if(servo_angle <= 0)servo_angle = 0;
	

	servo_cnt++;
	if(servo_cnt >= SERVO_PWM_CYCLE){
		servo_cnt = 0; 		
	}
	
	//�i���Ӂj���̊֐��́A�������ɏ������L�ڂ��Ȃ����Ɓi�ϐ��錾�͂��Ԃ�\�j//////////////////////////
			
    cnt0++;
    cnt1++;
	run_time++;
	LR_cnt++;
	OUT_cnt++;
	
	if(pattern > 10 && run_time > 800){//���s�J�n�����0ms/s����̃X�^�[�g�Ȃ̂� XXXXms�̓J�E���g���Ȃ�
		S_cnt++;//�������s����
	}
	
	cnt10ms++;
	if(cnt10ms >= 10){
		cnt10ms = 0;
		if(log_start == 1){
			if(log_num + 5 < LOG_MAX){
				log[log_num++] = pattern;
				log[log_num++] = sensor_inp(MASK4_4)&0x00ff;
				log[log_num++] = log_buf_handle;
				log[log_num++] = log_buf_Lmotor;
				log[log_num++] = log_buf_Rmotor;
			}	
		}
	}

}

/************************************************************************/
/* �^�C�}�{��                                                           */
/* �����@ �^�C�}�l 1=1ms                                                */
/************************************************************************/
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
}

/************************************************************************/
/* �Z���T��Ԍ��o                                                       */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~p0;

    sensor &= mask;

    return sensor;
}

/************************************************************************/
/* �N���X���C�����o����                                                 */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{
     unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
	//1111 1111   //0111 1110 //1111 1110  //0111 1111
    if( b==0xff  || b==0x7e   || b==0xfe  || b==0x7f ) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* �E�n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_rightline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
	//0000 1111   //0001 1111 //0001 1110 //0001 1101 //0001 1011 //0000 1101 //0000 1011 //0001 1001 //0001 0001 //0000 1001 //0011 1111
    if( b==0x0f || b==0x1f  || b==0x1e   || b==0x1d  || b==0x1b  || b==0x0d  || b==0x0b  || b==0x19  || b==0x11  || b==0x09  || b==0x3f) {
		
	//0000 1111   //0001 1111
    //if( b==0x0f || b==0x1f ) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_leftline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
	//1111 0000  //1111 1000 //0111 1000 //1011 1000  //1101 1000  //1011 0000  //1101 0000  //1001 1000 //1000 1000  //1001 0000 //1111 1100
    if( b==0xf0  || b==0xf8  || b==0x78 || b==0xb8   || b==0xd8   || b==0xb0    || b==0xd0   || b==0x98  || b==0x88   || b==0x90  || b==0xfc) {
	
	//1111 0000  //1111 1000 
    //if( b==0xf0  || b==0xf8 ) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* �f�B�b�v�X�C�b�`�l�ǂݍ���                                           */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3�`P1_0�ǂݍ���           */

    return  sw;
}

/************************************************************************/
/* �v�b�V���X�C�b�`�l�ǂݍ���                                           */
/* �߂�l �X�C�b�`�l ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~p2;                          /* �X�C�b�`�̂���|�[�g�ǂݍ��� */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* �X�^�[�g�o�[���o�Z���T�ǂݍ���                                       */
/* �߂�l �Z���T�l ON(�o�[����):1 OFF(�Ȃ�):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = ~p0;                           /* �X�^�[�g�o�[�M���ǂݍ���     */
    b &= 0x01;

    return  b;
}

/************************************************************************/
/* LED����                                                              */
/* �����@�X�C�b�`�l LED2:bit1 LED3:bit0  "0":���� "1":�_��              */
/* ��)0x3��LED2:ON LED3:ON  0x2��LED2:ON LED3:OFF                       */
/************************************************************************/
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = p2 & 0x3f;
    p2 = data | led;
}

/************************************************************************/
/* ���[�^���x����                                                       */
/* �����@ �����[�^:-100�`100�A�E���[�^:-100�`100                        */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
    int    sw_data;
/*
    sw_data = dipsw_get() + 5;
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
*/
	log_buf_Lmotor = accele_l;
	log_buf_Rmotor = accele_r;
	
    /* �����[�^���� */
    if( accele_l >= 0 ) {
        p2 &= 0xfd;

        if(accele_l == 100) trdgrd0 = (long)( PWM_CYCLE + 2 );
        else trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        p2 |= 0x02;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* �E���[�^���� */
    if( accele_r >= 0 ) {
        p2 &= 0xf7;

        if(accele_r == 100)trdgrc1 = (long)( PWM_CYCLE + 2 );
        else trdgrc1 = (long)( PWM_CYCLE - 1 ) * accele_r / 100;

    } else {
        p2 |= 0x08;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* �T�[�{�n���h������                                                   */
/* �����@ �T�[�{����p�x�F-90�`90                                       */
/*        -90�ō���90�x�A0�ł܂������A90�ŉE��90�x��]                  */
/************************************************************************/
void handle( int angle )
{
    /* �T�[�{�����E�t�ɓ����ꍇ�́A�u-�v���u+�v�ɑւ��Ă������� */
    //trdgrd1 = SERVO_CENTER - angle * HANDLE_STEP;
    angle_buf = SERVO_CENTER - angle * HANDLE_STEP;
	
	//���O�̃T�C�Y�ɍ��킹��i�b��Ή��j
	if(127 < angle)angle = 127;
	if(angle < -128)angle = -128;
	log_buf_handle = angle;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
�������e

Ver.1.00 2012.05.07 �쐬(�Z���T���Ver.4��Ver.5�p�ɕύX)
Ver.1.01 2013.04.18 �N�����N�A���[���`�F���W��O�̉�����2�{����1�{�ɂȂ���
                    ���Ƃɂ��A�R�����g�̏C��(�v���O�����̕ύX�͂Ȃ�)

*/
