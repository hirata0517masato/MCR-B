/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �p�\�R������T�[�{�̃Z���^����(R8C/38A��)                   */
/* �o�[�W����   Ver.1.01                                                    */
/* Date         2011.03.14                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/
/*
�}�C�R���J�[�̃T�[�{�������L�[�ōs���`�F�b�N�v���O�����ł��B
���L�̂P�L�[���������Ƃɐ����̂悤�ȓ��������܂��B

'Z'�L�[     �F�T�[�{�I�t�Z�b�g�l�{�P
'X'�L�[     �F�T�[�{�I�t�Z�b�g�l�|�P
'A'�L�[     �F�T�[�{�I�t�Z�b�g�l�{�P�O
'S'�L�[     �F�T�[�{�I�t�Z�b�g�l�|�P�O

�ڑ��́A
�E�`���P�[�u�������p�\�R��(�ʐM�\�t�g��TeraTermPro���n�C�p�[�^�[�~�i��)
�EP2�����[�^�h���C�u���Ver.4���T�[�{
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

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned int    servo_offset;           /* �T�[�{�I�t�Z�b�g             */

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

    servo_offset = 3750;
    printf(
        "Servo Center Adjustment Soft\n"
        "'Z' key   : Center Value +1\n"
        "'X' key   : Center Value -1\n"
        "\n"
        "'A' key   : Center Value +10\n"
        "'S' key   : Center Value -10\n"
        "\n"
    );
    printf( "%5d\r", servo_offset );

    while( 1 ) {
        trdgrd1 = servo_offset;

        i = get_uart0( &c );
        if( i == 1 ) {
            switch( c ) {
            case 'Z':
            case 'z':
                servo_offset++;
                if( servo_offset > 10000 ) servo_offset = 10000;
                printf( "%5d\r", servo_offset );
                break;

            case 'A':
            case 'a':
                servo_offset += 10;
                if( servo_offset > 10000 ) servo_offset = 10000;
                printf( "%5d\r", servo_offset );
                break;

            case 'X':
            case 'x':
                servo_offset--;
                if( servo_offset < 1000 ) servo_offset = 1000;
                printf( "%5d\r", servo_offset );
                break;

            case 'S':
            case 's':
                servo_offset -= 10;
                if( servo_offset < 1000 ) servo_offset = 1000;
                printf( "%5d\r", servo_offset );
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
    trdgrb1 = trdgrd1 = 3750;           /* P2_5�[�q��ON���ݒ�           */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/