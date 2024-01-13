/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �}�C�R���J�[�L�b�g�@����m�F�v���O����(R8C/38A��)           */
/* �o�[�W����   Ver.1.01                                                    */
/* Date         2011.03.18                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/

/*
�}�C�R���J�[�L�b�g�p�Z���T���Ver.4.1�A���[�^�h���C�u���Ver.4��
����m�F���s���܂��B
�}�C�R���{�[�h�̃f�B�b�v�X�C�b�`�ɂ�蓮��m�F������e��ύX���܂��B
   DipSW
bit3 2 1 0
   0 0 0 0 LED�̊m�F        LED��0.5�b�Ԋu�Ō��݂ɓ_��
   0 0 0 1 �߯�������̊m�F  OFF���FLED0�_�� ON���FLED1�_��
   0 0 1 0 �T�[�{�̊m�F     0�����E30������30���̌J��Ԃ�
   0 0 1 1 ���얳��
   0 1 0 0 �E���[�^�̊m�F   ���]���u���[�L�̌J��Ԃ�
   0 1 0 1                  �t�]���u���[�L�̌J��Ԃ�
   0 1 1 0 �����[�^�̊m�F   ���]���u���[�L�̌J��Ԃ�
   0 1 1 1                  �t�]���u���[�L�̌J��Ԃ�

   1 0 0 0 �Z���T�m�F       �Z���Tbit1,0��LED1,0�ɏo��
   1 0 0 1                  �Z���Tbit3,2��LED1,0�ɏo��
   1 0 1 0                  �Z���Tbit5,4��LED1,0�ɏo��
   1 0 1 1                  �Z���Tbit7,6��LED1,0�ɏo��

   1 1 0 0 ���i���̊m�F     PWM  50%�őO�i�A 2�b��X�g�b�v
   1 1 0 1 ���i���̊m�F     PWM  50%�őO�i�A 5�b��X�g�b�v
   1 1 1 0 ���i���̊m�F     PWM 100%�őO�i�A 2�b��X�g�b�v
   1 1 1 1 ���i���̊m�F     PWM 100%�őO�i�A 5�b��X�g�b�v
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
#define SERVO_CENTER    3750            /* �T�[�{�̃Z���^�l             */
#define HANDLE_STEP     22              /* 1�K���̒l                    */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
unsigned char sensor_inp_all( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void motor( int accele_l, int accele_r );
void handle( int angle );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p                  */
unsigned long   cnt1;                   /* main���Ŏg�p                 */

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* ���݃f�B�b�v�X�C�b�`�L��     */
    unsigned char   before_sw;          /* �O��f�B�b�v�X�C�b�`�L��     */
    unsigned char   c;                  /* ��Ɨp                       */
    int             i;                  /* ��Ɨp                       */

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */

    /* �ϐ������� */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    motor( 0, 0 );
    led_out( 0x0 );

    while( 1 ) {
    /* �f�B�b�v�X�C�b�`�ǂݍ��� */
    now_sw = dipsw_get();

    /* �O��̃X�C�b�`�l�Ɣ�r */
    if( before_sw != now_sw ) {
        /* �s��v�Ȃ�O��l�X�V�A�^�C�}�l�̃N���A */
        before_sw = now_sw;
        cnt1 = 0;
    }

    /* �f�B�b�v�X�C�b�`�̒l�ɂ�蓮��m�F���[�h�̑I�� */
    switch( now_sw ) {

        /* LED�̊m�F LED��0.5�b�Ԋu�Ō��݂ɓ_�� */
        case 0:
            if( cnt1 < 500 ) {
                led_out( 0x1 );
            } else if( cnt1 < 1000 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �v�b�V���X�C�b�`�̊m�F OFF���FLED0�_�� ON���FLED1�_�� */
        case 1:
            led_out( pushsw_get() + 1 );
            break;

        /* �T�[�{�̊m�F 0�����E30������30���̌J��Ԃ� */
        case 2:
            if( cnt1 < 1000 ) {
                handle( 0 );
            } else if( cnt1 < 2000 ) {
                handle( 30 );
            } else if( cnt1 < 3000 ) {
                handle( -30 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �������Ȃ� */
        case 3:
            break;

        /* �E���[�^�̊m�F ���]���u���[�L�̌J��Ԃ� */
        case 4:
            if( cnt1 < 1000 ) {
                motor( 0, 100 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �E���[�^�̊m�F �t�]���u���[�L�̌J��Ԃ� */
        case 5:
            if( cnt1 < 1000 ) {
                motor( 0, -100 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �����[�^�̊m�F ���]���u���[�L�̌J��Ԃ� */
        case 6:
            if( cnt1 < 1000 ) {
                motor( 100, 0 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �����[�^�̊m�F �t�]���u���[�L�̌J��Ԃ� */
        case 7:
            if( cnt1 < 1000 ) {
                motor( -100, 0 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �Z���T��̊m�F �Z���Tbit1,0��LED1,0�ɏo�� */
        case 8:
            c = sensor_inp_all( 0x03 );
            led_out( c );
            break;

        /* �Z���T��̊m�F �Z���Tbit3,2��LED1,0�ɏo�� */
        case 9:
            c = sensor_inp_all( 0x0c );
            c = c >> 2;
            led_out( c );
            break;

        /* �Z���T��̊m�F �Z���Tbit5,4��LED1,0�ɏo�� */
        case 10:
            c = sensor_inp_all( 0x30 );
            c = c >> 4;
            led_out( c );
            break;

        /* �Z���T��̊m�F �Z���Tbit7,6��LED1,0�ɏo�� */
        case 11:
            c = sensor_inp_all( 0xc0 );
            c = c >> 6;
            led_out( c );
            break;

        /* ���i���̊m�F PWM  50%�őO�i�A 2�b��X�g�b�v */
        case 12:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 4000 ) {
                motor( 50, 50 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* ���i���̊m�F PWM  50%�őO�i�A 5�b��X�g�b�v */
        case 13:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 7000 ) {
                motor( 50, 50 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* ���i���̊m�F PWM 100%�őO�i�A 2�b��X�g�b�v */
        case 14:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 4000 ) {
                motor( 100, 100 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* ���i���̊m�F PWM 100%�őO�i�A 5�b��X�g�b�v */
        case 15:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 7000 ) {
                motor( 100, 100 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* �ǂ�ł��Ȃ��Ȃ� */
        default:
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
}

/************************************************************************/
/* �Z���T��Ԍ��o(�X�^�[�g�o�[�Z���T���܂߂��ׂẴZ���T)               */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp_all( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~p0;
    sensor &= mask;

    return sensor;
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

    sw_data = dipsw_get() + 5;
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

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

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
�������e

Ver.1.00 2010.04.01 �쐬
Ver.1.01 2011.03.14 �^�C�}RD�̃��W�X�^�̐ݒ菇�̕ύX
Ver.1.02 2011.03.18 motor�֐��̈����̕ϐ����ύX
*/
