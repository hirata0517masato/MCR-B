/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     パソコンからサーボのセンタ調整(R8C/38A版)                   */
/* バージョン   Ver.1.01                                                    */
/* Date         2011.03.14                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/
/*
マイコンカーのサーボ調整をキーで行うチェックプログラムです。
下記の１キーを押すごとに説明のような動きをします。

'Z'キー     ：サーボオフセット値＋１
'X'キー     ：サーボオフセット値－１
'A'キー     ：サーボオフセット値＋１０
'S'キー     ：サーボオフセット値－１０

接続は、
・伝送ケーブル←→パソコン(通信ソフトはTeraTermProかハイパーターミナル)
・P2←モータドライブ基板Ver.4←サーボ
です。
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "printf_lib.h"                 /* printf使用ライブラリ         */

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* 定数設定 */
#define PWM_CYCLE       2499           /* モータPWMの周期   初期値39999  */
#define SERVO_CENTER    4170            /* サーボのセンタ値             */
#define HANDLE_STEP     22              /* 1゜分の値                    */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
void handle( int angle );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned int    servo_offset;           /* サーボオフセット             */
int angle_buf = 0;


/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    int     i, ret;
    char    c;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
    asm(" fset I ");                    /* 全体の割り込み許可           */
	handle( SERVO_CENTER );
	
	
    servo_offset = SERVO_CENTER;
	
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
		handle( servo_offset );
		
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
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void )
{
    int i;

    /* クロックをXINクロック(20MHz)に変更 */
    prc0  = 1;                          /* プロテクト解除               */
    cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
    cm05  = 0;                          /* XINクロック発振              */
    for(i=0; i<50; i++ );               /* 安定するまで少し待つ(約10ms) */
    ocd2  = 0;                          /* システムクロックをXINにする  */
    prc0  = 0;                          /* プロテクトON                 */

    /* ポートの入出力設定 */
    prc2 = 1;                           /* PD0のプロテクト解除          */
    pd0 = 0x00;                         /* 7-0:センサ基板Ver.4.1        */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:モータドライブ基板Ver.4  */
    pd3 = 0xff;                         /*                              */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0xff;                         /*                              */
    pd6 = 0xff;                         /*                              */
    pd7 = 0xff;                         /*                              */
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3～P1_0のプルアップON     */

	/* タイマRBの設定 */
    /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* 動作モード、分周比設定       */
    trbpre = 200-1;                     /* プリスケーラレジスタ         */
    trbpr  = 100-1;                     /* プライマリレジスタ           */
    trbic  = 0x07;                      /* 割り込み優先レベル設定       */
    trbcr  = 0x01;                      /* カウント開始                 */
	
    /* タイマRD リセット同期PWMモードの設定*/
    /* PWM周期 = 1 / 20[MHz]   * カウントソース * (TRDGRA0+1)
               = 1 / (20*10^6) * 8              * 40000
               = 0.016[s] = 16[ms]
    */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x23;                     /* ソースカウントの選択:f8      */
    trdgra0 = trdgrc0 = PWM_CYCLE;          /* 周期                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅設定           */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅設定           */
    trdgrb1 = trdgrd1 = PWM_CYCLE/2;           /* P2_5端子のON幅設定           */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
}


/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRB(vect=24)
void intTRB( void )
{
	static int servo_cnt = 0,servo_angle = 0;
	
  
	
	
	if(servo_cnt == 0){
		servo_angle = angle_buf ;	
	}
	
	if(servo_angle >= (PWM_CYCLE+1)) trdgrd1 = PWM_CYCLE+1;
	else trdgrd1 = servo_angle;
			
	servo_angle -=  PWM_CYCLE+1;
	if(servo_angle <= 0)servo_angle = 0;
	
	servo_cnt++;
	if(servo_cnt >= 12)servo_cnt = 0;
}


/************************************************************************/
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90～90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void handle( int angle )
{
    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
    //trdgrd1 = SERVO_CENTER - angle * HANDLE_STEP;
	angle_buf = angle; //SERVO_CENTER - angle * HANDLE_STEP;
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/
