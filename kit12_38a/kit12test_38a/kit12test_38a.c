/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     マイコンカーキットVer.5.1 動作確認プログラム(R8C/38A版)     */
/* バージョン   Ver.1.00                                                    */
/* Date         2012.05.07                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*
マイコンカーキット用センサ基板Ver.5、モータドライブ基板Ver.5の
動作確認を行います。
マイコンボードのディップスイッチにより動作確認する内容を変更します。
   DipSW
bit3 2 1 0
   0 0 0 0 LEDの確認        LEDが0.5秒間隔で交互に点灯
   0 0 0 1 ﾌﾟｯｼｭｽｲｯﾁの確認  OFF時：LED0点灯 ON時：LED1点灯
   0 0 1 0 サーボの確認     0°→右30°→左30°の繰り返し
   0 0 1 1 動作無し
   0 1 0 0 右モータの確認   正転→ブレーキの繰り返し
   0 1 0 1                  逆転→ブレーキの繰り返し
   0 1 1 0 左モータの確認   正転→ブレーキの繰り返し
   0 1 1 1                  逆転→ブレーキの繰り返し

   1 0 0 0 センサ確認       センサbit1,0をLED1,0に出力
   1 0 0 1                  センサbit3,2をLED1,0に出力
   1 0 1 0                  センサbit5,4をLED1,0に出力
   1 0 1 1                  センサbit7,6をLED1,0に出力

   1 1 0 0 直進性の確認     PWM  50%で前進、 2秒後ストップ
   1 1 0 1 直進性の確認     PWM  50%で前進、 5秒後ストップ
   1 1 1 0 直進性の確認     PWM 100%で前進、 2秒後ストップ
   1 1 1 1 直進性の確認     PWM 100%で前進、 5秒後ストップ
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* 定数設定 */

//#define PWM_CYCLE       39999           /* モータPWMの周期              */
//#define SERVO_CENTER    4170 //3750           /* サーボのセンタ値             */

#define PWM_CYCLE       	9999           /* 駆動モータPWMの周期       1ms       								*/
#define SERVO_PWM_CYCLE     10//16			   /* サーボモータPWMの周期     PWM_CYCLE * SERVO_PWM_CYCLE (ms)         */
#define HANDLE_STEP     14//22              /* 1゜分の値                    */


#define SERVO_CENTER    	16730          /* サーボのセンタ値             */


/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char sensor_inp_all( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void motor( int accele_l, int accele_r );
void handle( int angle );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用                  */
unsigned long   cnt1;                   /* main内で使用                 */

int angle_buf = 0;

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* 現在ディップスイッチ記憶     */
    unsigned char   before_sw;          /* 前回ディップスイッチ記憶     */
    unsigned char   c;                  /* 作業用                       */
    int             i;                  /* 作業用                       */

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    asm(" fset I ");                    /* 全体の割り込み許可           */

    /* 変数初期化 */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* マイコンカーの状態初期化 */
    handle( 0 );
    motor( 0, 0 );
    led_out( 0x0 );

    while( 1 ) {
    /* ディップスイッチ読み込み */
    now_sw = dipsw_get();

    /* 前回のスイッチ値と比較 */
    if( before_sw != now_sw ) {
        /* 不一致なら前回値更新、タイマ値のクリア */
        before_sw = now_sw;
        cnt1 = 0;
    }

    /* ディップスイッチの値により動作確認モードの選択 */
    switch( now_sw ) {

        /* LEDの確認 LEDが0.5秒間隔で交互に点灯 */
        case 0:
            if( cnt1 < 500 ) {
                led_out( 0x1 );
            } else if( cnt1 < 1000 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        /* プッシュスイッチの確認 OFF時：LED0点灯 ON時：LED1点灯 */
        case 1:
            led_out( pushsw_get() + 1 );
            break;

        /* サーボの確認 0°→右30°→左30°の繰り返し */
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

        /* 何もしない */
        case 3:
            break;

        /* 右モータの確認 正転→ブレーキの繰り返し */
        case 4:
            if( cnt1 < 1000 ) {
                motor( 0, 100 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 右モータの確認 逆転→ブレーキの繰り返し */
        case 5:
            if( cnt1 < 1000 ) {
                motor( 0, -100 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 左モータの確認 正転→ブレーキの繰り返し */
        case 6:
            if( cnt1 < 1000 ) {
                motor( 100, 0 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 左モータの確認 逆転→ブレーキの繰り返し */
        case 7:
            if( cnt1 < 1000 ) {
                motor( -100, 0 );
            } else if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* センサ基板の確認 センサbit1,0をLED1,0に出力 */
        case 8:
            c = sensor_inp_all( 0x03 );
            led_out( c );
            break;

        /* センサ基板の確認 センサbit3,2をLED1,0に出力 */
        case 9:
            c = sensor_inp_all( 0x0c );
            c = c >> 2;
            led_out( c );
            break;

        /* センサ基板の確認 センサbit5,4をLED1,0に出力 */
        case 10:
            c = sensor_inp_all( 0x30 );
            c = c >> 4;
            led_out( c );
            break;

        /* センサ基板の確認 センサbit7,6をLED1,0に出力 */
        case 11:
            c = sensor_inp_all( 0xc0 );
            c = c >> 6;
            led_out( c );
            break;

        /* 直進性の確認 PWM  50%で前進、 2秒後ストップ */
        case 12:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 4000 ) {
                motor( 50, 50 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* 直進性の確認 PWM  50%で前進、 5秒後ストップ */
        case 13:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 7000 ) {
                motor( 50, 50 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* 直進性の確認 PWM 100%で前進、 2秒後ストップ */
        case 14:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 4000 ) {
                motor( 100, 100 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* 直進性の確認 PWM 100%で前進、 5秒後ストップ */
        case 15:
            if( cnt1 < 2000 ) {
                motor( 0, 0 );
            } else if( cnt1 < 7000 ) {
                motor( 100, 100 );
            } else {
                motor( 0, 0 );
            }
            break;

        /* どれでもないなら */
        default:
            break;
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
    pd0 = 0x00;                         /* 7-0:センサ基板Ver.5          */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    pd2 = 0xfe;                         /* 7-0:モータドライブ基板Ver.5  */
    pd3 = 0xff;                         /*                              */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0xff;                         /*                              */
    pd6 = 0xff;                         /*                              */
    pd7 = 0xff;                         /*                              */
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3〜P1_0のプルアップON     */

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
//    trdcr0  = 0x23;                     /* ソースカウントの選択:f8      */
	trdcr0  = 0x21;                     /* ソースカウントの選択:f2      */
    trdgra0 = trdgrc0 = PWM_CYCLE;          /* 周期                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅設定           */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅設定           */
//	trdgrb1 = trdgrd1 = SERVO_CENTER;           /* P2_5端子のON幅設定           */
    trdgrb1 = trdgrd1 = PWM_CYCLE+2;           /* P2_5端子のON幅設定           */
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
	
	if(servo_angle >= (PWM_CYCLE)) trdgrd1 = PWM_CYCLE+2;
	else if(servo_angle > 0 )trdgrd1 = servo_angle;
	else {
		 trdgrd1 = 0;
	}	
	servo_angle -=  PWM_CYCLE;
	if(servo_angle <= 0)servo_angle = 0;
	

	servo_cnt++;
	if(servo_cnt >= SERVO_PWM_CYCLE){
		servo_cnt = 0; 		
	}
	
	//（注意）この関数は、これより上に処理を記載しないこと（変数宣言はたぶん可能）//////////////////////////
	
	cnt0++;
    cnt1++;
	
}

/************************************************************************/
/* センサ状態検出(スタートバーセンサを含めすべてのセンサ)               */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
/************************************************************************/
unsigned char sensor_inp_all( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~p0;
    sensor &= mask;

    return sensor;
}

/************************************************************************/
/* ディップスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 0〜15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3〜P1_0読み込み           */

    return  sw;
}

/************************************************************************/
/* プッシュスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~p2;                          /* スイッチのあるポート読み込み */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* LED制御                                                              */
/* 引数　スイッチ値 LED0:bit0 LED1:bit1  "0":消灯 "1":点灯              */
/* 例)0x3→LED1:ON LED0:ON  0x2→LED1:ON LED0:OFF                       */
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
/* モータ速度制御                                                       */
/* 引数　 左モータ:-100〜100、右モータ:-100〜100                        */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
    int    sw_data;

    sw_data = dipsw_get() + 5;
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* 左モータ制御 */
    if( accele_l >= 0 ) {
        p2 &= 0xfd;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        p2 |= 0x02;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* 右モータ制御 */
    if( accele_r >= 0 ) {
        p2 &= 0xf7;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        p2 |= 0x08;
        trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90〜90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void handle( int angle )
{
    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
//    trdgrd1 = SERVO_CENTER - angle * HANDLE_STEP;
	
	angle_buf = SERVO_CENTER - angle * HANDLE_STEP; 

}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂内容

Ver.1.00 2012.05.07 作成(マイコンカーキットVer.5からコメント変更)
*/
