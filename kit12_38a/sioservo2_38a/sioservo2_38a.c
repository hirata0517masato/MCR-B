/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �p�\�R������T�[�{�̍ő�؂�p�𒲂ׂ�(R8C/38A��)           */
/* �o�[�W����   Ver.1.00                                                    */
/* Date         2012.05.07                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/
/*
�}�C�R���J�[�̎��ۂ̍ő�؂�p���m���߂�`�F�b�N�v���O�����ł��B
���L�̂P�L�[���������Ƃɐ����̂悤�ȓ��������܂��B

'Z'�L�[     �F�T�[�{�I�t�Z�b�g�l�|1
'X'�L�[     �F�T�[�{�I�t�Z�b�g�l�{1
'A'�L�[     �F�T�[�{�I�t�Z�b�g�l�|3
'S'�L�[     �F�T�[�{�I�t�Z�b�g�l�{3

�ڑ��́A
�E�`���P�[�u�������p�\�R��(�ʐM�\�t�g��TeraTermPro���n�C�p�[�^�[�~�i��)
�EP2�����[�^�h���C�u���Ver.5���T�[�{
�ł��B
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
#define SERVO_CENTER    3750            /* �T�[�{�̃Z���^�l             */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
int     servo_angle;                    /* �T�[�{�p�x                   */

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    int     i, ret;
    char    c;

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */

    servo_angle = 0;
    printf(
        "Servo Angle Check Soft\n"
        "'Z' key   : Angle Value -1\n"
        "'X' key   : Angle Value +1\n"
        "\n"
        "'A' key   : Angle Value -3\n"
        "'S' key   : Angle Value +3\n"
        "\n"
    );
    printf( "%3d\r", servo_angle );

    while( 1 ) {
        trdgrd1 = SERVO_CENTER - servo_angle * 22;

        i = get_uart0( &c );
        if( i == 1 ) {
            switch( c ) {
            case 'Z':
            case 'z':
                servo_angle--;
                if( servo_angle < -90 ) servo_angle = -90;
                printf( "%3d\r", servo_angle );
                break;

            case 'X':
            case 'x':
                servo_angle++;
                if( servo_angle > 90 ) servo_angle = 90;
                printf( "%3d\r", servo_angle );
                break;

            case 'A':
            case 'a':
                servo_angle -= 3;
                if( servo_angle < -90 ) servo_angle = -90;
                printf( "%3d\r", servo_angle );
                break;

            case 'S':
            case 's':
                servo_angle += 3;
                if( servo_angle > 90 ) servo_angle = 90;
                printf( "%3d\r", servo_angle );
                break;

            default:
                break;
            }
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
    trdgra0 = trdgrc0 = 39999;          /* ����                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON���ݒ�           */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON���ݒ�           */
    trdgrb1 = trdgrd1 = SERVO_CENTER;   /* P2_5�[�q��ON���ݒ�           */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
