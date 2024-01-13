/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �}�C�R���J�[�g���[�X��{�v���O����(R8C/38A��)               */
/* �o�[�W����   Ver.1.02                                                    */
/* Date         2011.03.18                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/
/*
���̃v���O�����́A���L��ɑΉ����Ă��܂��B
�ERY_R8C38�{�[�h
�E���[�^�h���C�u���Ver.4
�E�Z���T���Ver.4.1
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */

/*======================================*/
/* �V���{����`                         */
/*======================================*/

/* �萔�ݒ� */
#define PWM_CYCLE       39999           /* ���[�^PWM�̎���              */
#define SERVO_CENTER    4170            /* �T�[�{�̃Z���^�l             */
#define HANDLE_STEP     22              /* 1�K���̒l                    */

#define RUN_TIME	30000	//���s����

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
void wait( unsigned long set_time );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p                  */
unsigned long   cnt1;                   /* main���Ŏg�p                 */
unsigned long   run_time = 0;
volatile unsigned long   wait_timer = 0;
int             pattern;                /* �p�^�[���ԍ�                 */

int S_flag = 0;
unsigned long S_cnt = 0;

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    int     i;

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */

    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    motor( 0, 0 );

    while( 1 ) {
		
	if(pattern > 10 && run_time > RUN_TIME){
		pattern = 99;
	}
	
    switch( pattern ) {

    /*****************************************************************
    �p�^�[���ɂ���
     0�F�X�C�b�`���͑҂�
     1�F�X�^�[�g�o�[���J�������`�F�b�N
    11�F�ʏ�g���[�X
    12�F�E�֑�Ȃ��̏I���̃`�F�b�N
    13�F���֑�Ȃ��̏I���̃`�F�b�N
    21�F�P�{�ڂ̃N���X���C�����o���̏���
    22�F�Q�{�ڂ�ǂݔ�΂�
    23�F�N���X���C����̃g���[�X�A�N�����N���o
    31�F���N�����N�N���A�����@���肷��܂ŏ����҂�
    32�F���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N
    41�F�E�N�����N�N���A�����@���肷��܂ŏ����҂�
    42�F�E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N
    51�F�P�{�ڂ̉E�n�[�t���C�����o���̏���
    52�F�Q�{�ڂ�ǂݔ�΂�
    53�F�E�n�[�t���C����̃g���[�X
    54�F�E���[���`�F���W�I���̃`�F�b�N
    61�F�P�{�ڂ̍��n�[�t���C�����o���̏���
    62�F�Q�{�ڂ�ǂݔ�΂�
    63�F���n�[�t���C����̃g���[�X
    64�F�����[���`�F���W�I���̃`�F�b�N
    *****************************************************************/

    case 0:
        /* �X�C�b�`���͑҂� */
        if( pushsw_get() ) {
            pattern = 1;
			
			wait(1000);
			
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

        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* �Z���^���܂����� */
				
				
				S_flag = 1;
				
				
                handle( 0 );
                motor( 100 ,100 );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x0C:
                /* �����ɍ���聨�E�֔��Ȃ� */
                handle( 3 );
                motor( 100 ,100 );
                break;
				
            case 0x04:
                /* �����ɍ���聨�E�֔��Ȃ� */
                handle( 5 );
                motor( 95 ,85 );
                break;

            case 0x06:
                /* ��������聨�E�֏��Ȃ� */
				
				if(S_flag == 1){
					S_flag = 0;
					S_cnt = 0;
				}	
				
				if(S_cnt < 20){
					handle( 12 );
                	motor( -50 ,-50 );
				
				}else{
                	handle( 12 );
                	motor( 90 ,40 );
				}
                break;

            case 0x07:
                /* �����炢����聨�E�֒��Ȃ� */
                handle( 15 );
                motor( 80 ,30 );
                break;

            case 0x03:
                /* �傫������聨�E�֑�Ȃ� */
                handle( 25 );
                motor( 80 ,10 );
                pattern = 12;
                break;

			///////////////////////////////////////////////////////
			
			case 0x30:
                /* �����ɉE��聨���֔��Ȃ� */
                handle( -3 );
                motor( 100 ,100 );
                break;
				
            case 0x20:
                /* �����ɉE��聨���֔��Ȃ� */
                handle( -5 );
                motor( 85 ,95 );
                break;

            case 0x60:
                /* �����E��聨���֏��Ȃ� */
				
				if(S_flag == 1){
					S_flag = 0;
					S_cnt = 0;
				}	
				
				if(S_cnt < 20){
					handle( -12 );
                	motor( -50 ,-50 );
				
				}else{
                	handle( -12 );
                	motor( 40 ,90 );
				}
			
                break;

            case 0xe0:
                /* �����炢�E��聨���֒��Ȃ� */
                handle( -15 );
                motor( 30 ,80 );
                break;

            case 0xc0:
                /* �傫���E��聨���֑�Ȃ� */
                handle( -25 );
                motor( 10 ,80 );
                pattern = 13;
                break;

            default:
                break;
        }
        break;

    case 12:
        /* �E�֑�Ȃ��̏I���̃`�F�b�N */
        if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
            pattern = 51;
            break;
        }
        if( check_leftline() ) {        /* ���n�[�t���C���`�F�b�N       */
            pattern = 61;
            break;
        }
        if( sensor_inp(MASK3_3) == 0x06 ) {
            pattern = 11;
        }
        break;

    case 13:
        /* ���֑�Ȃ��̏I���̃`�F�b�N */
        if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
            pattern = 51;
            break;
        }
        if( check_leftline() ) {        /* ���n�[�t���C���`�F�b�N       */
            pattern = 61;
            break;
        }
        if( sensor_inp(MASK3_3) == 0x60 ) {
            pattern = 11;
        }
        break;

    case 21:
        /* �P�{�ڂ̃N���X���C�����o���̏��� */
        led_out( 0x3 );
        handle( 0 );
        motor( -50 ,-50 );
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
        if( check_leftline() ) {
            /* ���N�����N�Ɣ��f�����N�����N�N���A������ */
            led_out( 0x1 );
            handle( -25 );
            motor( -80 ,100 );
            pattern = 31;
            cnt1 = 0;
            break;
        }
        if( check_rightline() ) {
            /* �E�N�����N�Ɣ��f���E�N�����N�N���A������ */
            led_out( 0x2 );
            handle( 25 );
            motor( 100 ,-80 );
            pattern = 41;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* �Z���^���܂����� */
                handle( 0 );
                motor( 40 ,40 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* ����聨�E�Ȃ� */
                handle( 8 );
                motor( 40 ,30 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* �E��聨���Ȃ� */
                handle( -8 );
                motor( 30 ,40 );
                break;
        }
        break;

    case 31:
        /* ���N�����N�N���A�����@���肷��܂ŏ����҂� */
        if( cnt1 > 200 ) {
            pattern = 32;
            cnt1 = 0;
        }
        break;

    case 32:
        /* ���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if( sensor_inp(MASK3_3) == 0xC0 ) {
            led_out( 0x0 );
			handle( -15 );
            motor( 50 ,80 );
			
            pattern = 33;
            cnt1 = 0;
        }
        break;

	case 33:
        /* ���N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if( sensor_inp(MASK3_3) == 0x60 ) {
            led_out( 0x0 );
            pattern = 11;
            cnt1 = 0;
        }
        break;
		
    case 41:
        /* �E�N�����N�N���A�����@���肷��܂ŏ����҂� */
        if( cnt1 > 200 ) {
            pattern = 42;
            cnt1 = 0;
        }
        break;

    case 42:
        /* �E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if( sensor_inp(MASK3_3) == 0x03 ) {
            led_out( 0x0 );
			handle( 15 );
            motor( 80 ,50 );
			
            pattern = 43;
            cnt1 = 0;
        }
        break;
	 case 43:
        /* �E�N�����N�N���A�����@�Ȃ��I���̃`�F�b�N */
        if( sensor_inp(MASK3_3) == 0x06 ) {
            led_out( 0x0 );
            pattern = 11;
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
        break;

    case 52:
        /* �Q�{�ڂ�ǂݔ�΂� */
        if( cnt1 > 100 ) {
            pattern = 53;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_leftline() ) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
       
        break;

    case 53:
        /* �E�n�[�t���C����̃g���[�X�A���[���`�F���W */
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( 15 );
            motor( 70 ,50 );
            pattern = 54;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* �Z���^���܂����� */
                handle( 0 );
                motor( 80 ,80 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* ����聨�E�Ȃ� */
                handle( 8 );
                motor( 80 ,50 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* �E��聨���Ȃ� */
                handle( -8 );
                motor( 50 ,80 );
                break;
            default:
                break;
        }
        break;

    case 54:
        /* �E���[���`�F���W�I���̃`�F�b�N */
        if( sensor_inp( MASK4_4 ) == 0x01 || sensor_inp( MASK4_4 ) == 0x03 ) {
            handle( -5 );
            motor( 50 ,50 );
          
			
            led_out( 0x0 );
            pattern = 55;
            cnt1 = 0;
        }
        break;
			
    case 55:
        /* �E���[���`�F���W�I���̃`�F�b�N */
        if( sensor_inp( MASK4_4 ) == 0x60 ) {
            led_out( 0x0 );
            pattern = 11;
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
        break;

    case 62:
        /* �Q�{�ڂ�ǂݔ�΂� */
        if( cnt1 > 100 ) {
            pattern = 63;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_rightline()) {       // �N���X���C���`�F�b�N         
            pattern = 21;
            break;
        }
        break;

    case 63:
        /* ���n�[�t���C����̃g���[�X�A���[���`�F���W */
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( -15 );
            motor( 50 ,70 );
            pattern = 64;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* �Z���^���܂����� */
                handle( 0 );
                motor( 80 ,80 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* ����聨�E�Ȃ� */
                handle( 8 );
                motor( 80 ,50 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* �E��聨���Ȃ� */
                handle( -8 );
                motor( 50 ,80 );
                break;
            default:
                break;
        }
        break;

	case 64:
        
        if( sensor_inp( MASK4_4 ) == 0x80 || sensor_inp( MASK4_4 ) == 0xC0) {
			handle( 5 );
            motor( 50 ,50 );
          
			
            led_out( 0x0 );
            pattern = 65;
            cnt1 = 0;
        }
        break;
			
    case 65:
        /* �����[���`�F���W�I���̃`�F�b�N */
        if( sensor_inp( MASK4_4 ) == 0x06 ) {
            led_out( 0x0 );
            pattern = 11;
            cnt1 = 0;
        }
        break;

	case 99://���s�I��
		 motor( 0 ,0 );
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
    pd0 = 0x00;                         /* 7-0:�Z���T���Ver.4.1        */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:���[�^�h���C�u���Ver.4  */
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
    trdcr0  = 0x23;                     /* �\�[�X�J�E���g�̑I��:f8      */
    trdgra0 = trdgrc0 = PWM_CYCLE;      /* ����                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON���ݒ�           */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON���ݒ�           */
    trdgrb1 = trdgrd1 = SERVO_CENTER;   /* P2_5�[�q��ON���ݒ�           */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRB(vect=24)
void intTRB( void )
{
    cnt0++;
    cnt1++;
	run_time++;
	wait_timer++;
	S_cnt++;
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
    sensor  &= 0xef;
    if( sensor & 0x08 ) sensor |= 0x10;

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
    b = sensor_inp(MASK3_3);
    if( b==0xe7 || b==0x66 || b==0xe6 || b==0x67) {
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
    if( b==0x1f || b==0x1e  ) {
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
    if( b==0xf8 ||  b==0x78  ) {
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
    b &= 0x10;
    b >>= 4;

    return  b;
}

/************************************************************************/
/* LED����                                                              */
/* �����@�X�C�b�`�l LED0:bit0 LED1:bit1  "0":���� "1":�_��              */
/* ��)0x3��LED1:ON LED0:ON  0x2��LED1:ON LED0:OFF                       */
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

   // sw_data = dipsw_get() + 5;
   // accele_l = accele_l * sw_data / 20;
   // accele_r = accele_r * sw_data / 20;

    /* �����[�^���� */
    if( accele_l >= 0 ) {
        p2 &= 0xfd;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        p2 |= 0x02;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* �E���[�^���� */
    if( accele_r >= 0 ) {
        p2 &= 0xf7;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * accele_r / 100;
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
    trdgrd1 = SERVO_CENTER - angle * HANDLE_STEP;
}


void wait( unsigned long set_time ){
	wait_timer = 0;
	
	while(wait_timer < set_time){
			
	}
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
�������e

Ver.1.00 2010.04.01 �쐬
Ver.1.01 2011.03.14 �^�C�}RD�̃��W�X�^�̐ݒ菇�̕ύX
Ver.1.02 2011.03.18 motor�֐��̈����̕ϐ����ύX
*/