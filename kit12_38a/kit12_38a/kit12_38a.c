/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     マイコンカーキットVer.5.1 トレース基本プログラム(R8C/38A版) */
/* バージョン   Ver.1.01                                                    */
/* Date         2013.04.18                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/
/*
このプログラムは、下記基板に対応しています。
・RY_R8C38ボード
・モータドライブ基板Ver.5
・センサ基板Ver.5
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
#define PWM_CYCLE       	9999           /* 駆動モータPWMの周期       1ms       								*/
#define SERVO_PWM_CYCLE     10//16			   /* サーボモータPWMの周期     PWM_CYCLE * SERVO_PWM_CYCLE (ms)         */
#define HANDLE_STEP     	14//22              /* 1゜分の値                    */
#define SERVO_CENTER    	16760          /* サーボのセンタ値             */


#define RUN_TIME	40000	//走行時間

//#define WallOn //レーンチェンジの壁があるときに有効化すること



/* マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効) */
#define MASK2_2         0x66            /* ×○○××○○×             */
#define MASK2_0         0x60            /* ×○○×××××             */
#define MASK0_2         0x06            /* ×××××○○×             */
#define MASK3_3         0xe7            /* ○○○××○○○             */
#define MASK0_3         0x07            /* ×××××○○○             */
#define MASK3_0         0xe0            /* ○○○×××××             */
#define MASK4_0         0xf0            /* ○○○○××××             */
#define MASK0_4         0x0f            /* ××××○○○○             */
#define MASK4_4         0xff            /* ○○○○○○○○             */

/*======================================*/
/* プロトタイプ宣言                     */
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
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用                  */
unsigned long   cnt1;                   /* main内で使用                 */
unsigned long   run_time = 0;
int             pattern;                /* パターン番号                 */

int angle_buf = 0;

unsigned long S_cnt = 0,LR_cnt = 0;

int s_motor = 0; //直線用モータ出力

unsigned long OUT_cnt = 0;

unsigned char out_flag = 0; //コースアウトした場合は１


#define LOG_MAX 4025
signed char log[LOG_MAX] = {};
int log_num = 0;
int log_start = 0;
signed char log_buf_Lmotor,log_buf_Rmotor;
signed char log_buf_handle;


/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
   	int     i;
	int M = 0;
	int Out_cnt = 0;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
	init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
    asm(" fset I ");                    /* 全体の割り込み許可           */

    /* マイコンカーの状態初期化 */
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
			
			if(Out_cnt > 500){// 500msクランク判定が続いたらコースアウト
				pattern = 99;
				out_flag = 1; //コースアウト
				cnt1 = 0;
			}
		}
	}else{
		OUT_cnt = 0;
		Out_cnt = 0;
	}
	
    switch( pattern ) {

    /*****************************************************************
    パターンについて
     0：スイッチ入力待ち
     1：スタートバーが開いたかチェック
    11：通常トレース
    12：右へ大曲げの終わりのチェック
    13：左へ大曲げの終わりのチェック
    21：クロスライン検出時の処理
    22：クロスラインを読み飛ばす
    23：クロスライン後のトレース、クランク検出
    31：左クランククリア処理　安定するまで少し待つ
    32：左クランククリア処理　曲げ終わりのチェック
    41：右クランククリア処理　安定するまで少し待つ
    42：右クランククリア処理　曲げ終わりのチェック
    51：右ハーフライン検出時の処理
    52：右ハーフラインを読み飛ばす
    53：右ハーフライン後のトレース、レーンチェンジ
    54：右レーンチェンジ終了のチェック
    61：左ハーフライン検出時の処理
    62：左ハーフラインを読み飛ばす
    63：左ハーフライン後のトレース、レーンチェンジ
    64：左レーンチェンジ終了のチェック
    *****************************************************************/

    case 0:
        /* スイッチ入力待ち */
        if( pushsw_get() ) {
            pattern = 1;

			timer(1000);/////////////////////////////////////////////////////////////////////////////////////////////////
		
            cnt1 = 0;
            break;
        }
        if( cnt1 < 100 ) {              /* LED点滅処理                  */
            led_out( 0x1 );
        } else if( cnt1 < 200 ) {
            led_out( 0x2 );
        } else {
            cnt1 = 0;
        }
        break;

    case 1:
        /* スタートバーが開いたかチェック */
        if( !startbar_get() ) {
            /* スタート！！ */
            led_out( 0x0 );
            pattern = 11;
            run_time = 0;
            cnt1 = 0;
			log_start = 1;  //ログ取得開始
            break;
        }
        if( cnt1 < 50 ) {              /* LED点滅処理                   */
            led_out( 0x1 );
        } else if( cnt1 < 100 ) {
            led_out( 0x2 );
        } else {
            cnt1 = 0;
        }
        break;

    case 11:
		led_out( 0 );
		
        /* 通常トレース */
        if( check_crossline() ) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // 右ハーフラインチェック       
            pattern = 51;
            break;
        }
        if( check_leftline() ) {        // 左ハーフラインチェック       
            pattern = 61;
            break;
        }

		
		if(S_cnt > 1000){//指定時間以上直線が続いたら減速する（エンコーダでの速度制御風） T600x5 で500くらい
			s_motor = 90;
		}else{
			s_motor = 100;
		}
		
        switch( sensor_inp(MASK4_4) ) {
            case 0x18: //0001 1000
                /* センタ→まっすぐ */
				
				handle( 0 );
                motor( s_motor ,s_motor );
                break;
			///////////////////////////////////////////////////////////右曲げ
			
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

			///////////////////////////////////////////////////////左曲げ
			
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

    case 12://右大曲
		
		led_out( 3 );
		
		
        if( check_crossline() ) {       // 大曲げ中もクロスラインチェック 
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // 右ハーフラインチェック       
            pattern = 51;
            break;
        }
	 
        if( check_leftline() ) {        // 左ハーフラインチェック       
            pattern = 61;
            break;
        }
		
		
		if(LR_cnt < 20){
			handle( 100 );
       		motor( 100 ,-100 );
			
		}else if((sensor_inp(MASK4_4)&0x20) != 0x00 ){ //xx1x xxxx 外寄りすぎ
		//if((sensor_inp(MASK4_4)&0x10) != 0x00 ){ //xxx1 xxxx 外寄りすぎ
			led_out( 1 );
			
			handle( 120 );
       		motor( 100 ,-100 );
		
			
		}else if((sensor_inp(MASK4_4)&0x40) != 0x00 ){ //x1xx xxxx 外寄りすぎ
		//}else if((sensor_inp(MASK4_4)&0x20) != 0x00 ){ //xx1x xxxx 外寄りすぎ
		
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
			
		}else if((sensor_inp(MASK4_4)&0x80) != 0x00 ){ //1xxx xxxx 外寄りすぎ
		//}else if((sensor_inp(MASK4_4)&0x40) != 0x00 ){ //x1xx xxxx 外寄りすぎ
		
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
			

		}else if(LR_cnt < 400 ){ //カーブ前半 
		
			handle( 110 );
       		motor( 100 ,100 );
			
		
		}else if(LR_cnt < 800 ){ //カーブ中間
		
			handle( 110 );
        	motor( 100 ,100 );
			
		
		}else{//カーブ後半
		
			handle( 110 );
        	motor( 100 ,100 );
			
		}
		
				
		/* 右へ大曲げの終わりのチェック */
        if( sensor_inp(MASK4_4) == 0x0c || sensor_inp(MASK4_4) == 0x04 ) {//0000 1100  //0000 0100
		//if( (sensor_inp(MASK4_4) == 0x0c )) {//0000 1100
			

				pattern = 11;
			
				S_cnt = 0;
	
        }

        break;

    case 13: //左大曲
	
		led_out( 3 );
	
		
        if( check_crossline() ) {       // 大曲げ中もクロスラインチェック 
            pattern = 21;
            break;
        }
        if( check_rightline() ) {       // 右ハーフラインチェック       
            pattern = 51;
            break;
        }
		
        if( check_leftline() ) {        // 左ハーフラインチェック       
            pattern = 61;
            break;
        }
	   		
		if(LR_cnt < 20){
			
			handle( -100 );
       		motor( -100 ,100 );
			
		}else if((sensor_inp(MASK4_4)&0x04) != 0x00 ){ //xxxx x1xx 外寄りすぎ
		//if((sensor_inp(MASK4_4)&0x08) != 0x00 ){ //xxxx 1xxx 外寄りすぎ
		
			led_out( 1 );
			
			handle( -120 );
       		motor( -100 ,100 );
				
		}else if((sensor_inp(MASK4_4)&0x02) != 0x00 ){ //xxxx xx1x 外寄りすぎ
		//}else if((sensor_inp(MASK4_4)&0x04) != 0x00 ){ //xxxx x1xx 外寄りすぎ
		
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
			
		}else if((sensor_inp(MASK4_4)&0x01) != 0x00 ){ //xxxx xxx1 外寄りすぎ
		//}else if((sensor_inp(MASK4_4)&0x02) != 0x00 ){ //xxxx xx1x 外寄りすぎ
		
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
	
		}else if(LR_cnt < 400 ){ //カーブ前半 
		
			handle( -110 );
       		motor( 100 ,100 );
			
		
		}else if(LR_cnt < 800){ //カーブ中間
		
			handle( -110 );
        	motor( 100 ,100 );
			
			
		}else{//カーブ後半
		
			handle( -110 );
        	motor( 100 ,100 );
			
		}
		
		
		 /* 左へ大曲げの終わりのチェック */		
        if( (sensor_inp(MASK4_4) == 0x30) || (sensor_inp(MASK4_4) == 0x20) ) {//0011 0000  //0010 0000 
		//if( (sensor_inp(MASK4_4) == 0x30) ) {//0011 0000  
	//	if( (sensor_inp(MASK4_4) == 0x18) || (sensor_inp(MASK4_4) == 0x10) ) {//0001 1000  //001 0000  

				pattern = 11;
			
				S_cnt = 0;
		
        }

        break;

    case 21:
        /* １本目のクロスライン検出時の処理 */
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
        /* ２本目を読み飛ばす */
		
	
		if( cnt1 > 100 ) {
	       pattern = 23;
	       cnt1 = 0;
	    }
		
        break;

    case 23:
        /* クロスライン後のトレース、クランク検出 */
        if( check_leftline() ){// || ((sensor_inp(MASK4_4)&0xc0) == 0xc0)) {
            /* 左クランクと判断→左クランククリア処理へ */
            led_out( 0x1 );
            handle( -150 );
            motor( -100 ,100 );
            pattern = 31;
            cnt1 = 0;
            break;
        }
        if( check_rightline() ){// || ((sensor_inp(MASK4_4)&0x03) == 0x03)) {
            /* 右クランクと判断→右クランククリア処理へ */
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
                /* センタ→まっすぐ */

				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x08://0000 1000
                /* 微妙に左寄り→右へ微曲げ */

				handle( 8 );
                motor( M ,M );
                break;
				
			case 0x0C://0000 1100
                /* 微妙に左寄り→右へ微曲げ */

				handle( 10 );
                motor( M ,M );
                break;
				
            case 0x04://0000 0100
                /* 微妙に左寄り→右へ微曲げ */
   
				handle( 15 );
                motor( M ,M-10 );
                break;

            case 0x06://0000 0110
                /* 少し左寄り→右へ小曲げ */
                handle( 20 );
                motor( M ,M-40 );
				
                break;

			case 0x07://0000 0111
                /* 少し左寄り→右へ小曲げ */
                handle( 30 );
                motor( M ,M-40 );
                break;

            case 0x03://0000 0011
                /* 大きく左寄り→右へ大曲げ */
                handle( 30 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x10://0001 0000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -8 );
                motor( M ,M );
                break;
				
				
			case 0x30://0011 0000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -10 );
                motor( M ,M );
                break;
				
            case 0x20://0010 0000
                /* 微妙に右寄り→左へ微曲げ */
      	
				handle( -15 );
                motor( M-10 ,M );
                break;

            case 0x60://0110 0000
                /* 少し右寄り→左へ小曲げ */
				handle( -20 );
                motor( M-40 ,M );
                break;
			case 0xe0://1110 0000
                /* 少し右寄り→左へ小曲げ */
				handle( -30 );
                motor( M-40 ,M );
                break;

            case 0xc0://1100 0000
                /* 大きく右寄り→左へ大曲げ */
                handle( -30 );
                motor( M-40 ,M );
                break;
			
        }
  
        break;


    case 31:
		if( sensor_inp(MASK4_4)& 0x38 != 0x00 ) { //xx11 1xxx 外より
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
			
        /* 左クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 50 &&  (sensor_inp(MASK3_3) == 0xC0 || sensor_inp(MASK3_3) == 0x40 || sensor_inp(MASK3_3) == 0x80)){ //1100 0000  //0100 0000  //1000 0000
            
            pattern = 32;
            cnt1 = 0;
        }
		
	
        break;
        
	case 32:
		if( sensor_inp(MASK4_4)& 0x04 != 0x00 ) { //xxxx x1xx 外より
			handle( -150 );
        	motor(  90 ,100 );
		}else{
			handle( -130 );
        	motor( 90 ,100 );
		}
		
        /* 左クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 && ( sensor_inp(MASK3_3) == 0x60 ||  sensor_inp(MASK3_3) == 0x20 )){ //0110 0000 //0010 0000
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }

        break;
		
    case 41:
        if( sensor_inp(MASK4_4)& 0x1C != 0x00 ) { //xxx1 11xx 外より
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
			
        /* 右クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 50 && ( sensor_inp(MASK3_3) == 0x03 || sensor_inp(MASK3_3) == 0x02 || sensor_inp(MASK3_3) == 0x01) ){ //0000 0011  //0000 0010 //0000 0001
            
            pattern = 42;
            cnt1 = 0;
        }
		

        break;

    case 42:
		if( sensor_inp(MASK4_4)& 0x20 != 0x00 ) { //xx1x xxxx 外より
        	handle( 150 );
        	motor( 100 , 90 );
		}else{
			handle( 130 );
        	motor( 100 ,90 );
		}
		
        /* 右クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 &&  ( sensor_inp(MASK3_3) == 0x06 || sensor_inp(MASK3_3) == 0x04) ){ //0000 0110 //0000 0100
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }

        break;


    case 51:
        /* １本目の右ハーフライン検出時の処理 */
        led_out( 0x2 );
        handle( 0 );
        motor( 0 ,0 );
        pattern = 52;
        cnt1 = 0;
		
		if( check_crossline() || check_leftline() ){// || (sensor_inp(MASK4_4)&0x80) == 0x80) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
        break;

    case 52:
        /* ２本目を読み飛ばす */
        if( cnt1 > 30 ) {
            pattern = 53;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_leftline()){// || (sensor_inp(MASK4_4)&0x80) == 0x80) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
       
        break;

    case 53:
        /* 右ハーフライン後のトレース、レーンチェンジ */
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
                /* センタ→まっすぐ */
				
				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x10://0001 0000
                /* 微妙に左寄り→右へ微曲げ */

				handle( 15 );
                motor( M ,M );
                break;
			
			case 0x18://0001 1000
                /* 微妙に左寄り→右へ微曲げ */

				handle( 20 );
                motor( M ,M );
                break;
					
            case 0x08://0000 1000
                /* 微妙に左寄り→右へ微曲げ */
   
				handle( 25 );
                motor( M ,M-10 );
                break;

            case 0x0c://0000 1100
                /* 少し左寄り→右へ小曲げ */
                handle( 30 );
                motor( M ,M-40 );
				
                break;
			
			case 0x0e://0000 1110
                /* 少し左寄り→右へ小曲げ */
                handle( 35 );
                motor( M ,M-40 );
                break;

            case 0x06://0000 0110
                /* 大きく左寄り→右へ大曲げ */
                handle( 40 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x20://0010 0000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -15 );
                motor( M ,M );
                break;
			
			case 0x60://0110 0000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -20 );
                motor( M ,M );
                break;
					
            case 0x40://0100 0000
                /* 微妙に右寄り→左へ微曲げ */
      	
				handle( -25 );
                motor( M-10 ,M );
                break;

            case 0xc0://1100 0000
                /* 少し右寄り→左へ小曲げ */
				handle( -30 );
                motor( M-40 ,M );
                break;
		/*	case 0xe0://1100 0000
                // 少し右寄り→左へ小曲げ 
				handle( -30 );
                motor( M-40 ,M );
                break;
*/
            case 0x80://1000 0000
                /* 大きく右寄り→左へ大曲げ */
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
        /* 右レーンチェンジ終了のチェック */
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
        /* 右レーンチェンジ終了のチェック */
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
        /* 右レーンチェンジ終了のチェック */
        //if( sensor_inp( MASK4_4 ) == 0x60 ) { //0110 0000
		if( sensor_inp( MASK4_4 )&0x0c != 0x00 ) { //xxxx 11xx
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;
			
	
    case 61:
        /* １本目の左ハーフライン検出時の処理 */
        led_out( 0x1 );
        handle( 0 );
        motor( 0 ,0 );
        pattern = 62;
        cnt1 = 0;
		
		if( check_crossline() || check_rightline()){// || (sensor_inp(MASK4_4)&0x01) == 0x01) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
        break;

    case 62:
        /* ２本目を読み飛ばす */
        if( cnt1 > 30 ) {
            pattern = 63;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_rightline()){//|| (sensor_inp(MASK4_4)&0x01) == 0x01) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
        break;

    case 63:
        /* 左ハーフライン後のトレース、レーンチェンジ */
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
                /* センタ→まっすぐ */
				
				handle( 0 );
                motor( M ,M );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x04://0000 0100
                /* 微妙に左寄り→右へ微曲げ */

				handle( 15 );
                motor( M ,M );
                break;
			
			case 0x06://0000 0110
                /* 微妙に左寄り→右へ微曲げ */

				handle( 20 );
                motor( M ,M );
                break;
					
            case 0x02://0000 0010
                /* 微妙に左寄り→右へ微曲げ */
   
				handle( 25 );
                motor( M ,M-10 );
                break;

            case 0x03://0000 0011
                /* 少し左寄り→右へ小曲げ */
                handle( 30 );
                motor( M ,M-40 );
				
                break;
			
		/*	case 0x07://0000 0011
                // 少し左寄り→右へ小曲げ 
                handle( 30 );
                motor( M ,M-40 );
                break;
*/
            case 0x01://0000 0001
                /* 大きく左寄り→右へ大曲げ */
                handle( 35 );
                motor( M ,M-40 );
                break;
				
			///////////////////////////////////////////////////////
			
			case 0x08://0000 1000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -15 );
                motor( M ,M );
                break;
			
			case 0x18://0001 1000
                /* 微妙に右寄り→左へ微曲げ */
	
				handle( -20 );
                motor( M ,M );
                break;
					
            case 0x10://0001 0000
                /* 微妙に右寄り→左へ微曲げ */
      	
				handle( -25 );
                motor( M-10 ,M );
                break;

            case 0x30://0011 0000
                /* 少し右寄り→左へ小曲げ */
				handle( -30 );
                motor( M-40 ,M );
                break;
			case 0x70://0111 0000
                /* 少し右寄り→左へ小曲げ */
				handle( -35 );
                motor( M-40 ,M );
                break;

            case 0x60://0110 0000
                /* 大きく右寄り→左へ大曲げ */
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
        /* 左レーンチェンジ終了のチェック */
        //if( sensor_inp( MASK4_4 ) == 0x06 ) { //0000 0110
		//if(cnt1 > 100 &&  sensor_inp( MASK4_4 )&0x06 != 0x00) { //xxxx x11x
		if(cnt1 > 5 &&  sensor_inp( MASK4_4 )&0x03 != 0x00) { //xxxx xx11
            led_out( 0x0 );

#ifdef WallOn
			handle( 130 );
            motor( 100 ,-60 ); //メモ：左は切り返し遅いので-100とする
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
        /* 左レーンチェンジ終了のチェック */
        //if( sensor_inp( MASK4_4 ) == 0x06 ) { //0000 0110
		if( sensor_inp( MASK4_4 )&0x30 != 0x00) { //xx11 xxxx
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;

	case 99://走行終了
		 motor( 0 ,0 );
		 log_start = 0;//ログの保存終了
		 
		 if(pushsw_get()){//スイッチが押されるとログ出力モード
			 
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
	                /* センタ→まっすぐ */
				
					handle( 0 );
	               // if(cnt1 < 1000) motor( 100 ,100 );
	                break;
				///////////////////////////////////////////////////////////
			
				case 0x08://0000 1000
	                /* 微妙に左寄り→右へ微曲げ */
		
					handle( 8 );
	               // if(cnt1 < 1000)motor( 100 ,100 );
	                break;
			
				case 0x0c://0000 1100
	                /* 微妙に左寄り→右へ微曲げ */
		
					handle( 15 );
	               // if(cnt1 < 1000)motor( 100 ,100 );
	                break;
				
				
	            case 0x04://0000 0100
	                /* 微妙に左寄り→右へ微曲げ */
      
					handle( 30 );
	               // if(cnt1 < 1000)motor( 100 ,90 );
	                break;

	            case 0x06://0000 0110
	                /* 少し左寄り→右へ小曲げ */
	                handle( 40 );
	              //  if(cnt1 < 1000)motor( 100 ,90 );
				
	                break;
				
				case 0x02://0000 0010
	                /* 少し左寄り→右へ小曲げ */
	                handle( 50 );
	              //  if(cnt1 < 1000)motor( 100 ,85 );
				
	                break;
				
				case 0x07://0000 0111
	                /* 少し左寄り→右へ小曲げ */
	                handle( 60 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
			
	                break;

	            case 0x03://0000 0011
	                /* 大きく左寄り→右へ大曲げ */
					handle( 65 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
	             
	                break;
			
				case 0x83://1000 0011
	                /* 大きく左寄り→右へ大曲げ */
					handle( 65 );
	              //  if(cnt1 < 1000)motor( 100 ,80 );
	             
	                break;

				///////////////////////////////////////////////////////
			
				case 0x10://0001 0000
	                /* 微妙に右寄り→左へ微曲げ */
			
					handle( -8 );
	              //  if(cnt1 < 1000)motor( 100 ,100 );
	                break;
			
				case 0x30://0011 0000
	                /* 微妙に右寄り→左へ微曲げ */
			
					handle( -15 );
	             //   if(cnt1 < 1000)motor( 100 ,100 );
	                break;
				
	            case 0x20://0010 0000
	                /* 微妙に右寄り→左へ微曲げ */
      		
					handle( -30 );
	             //   if(cnt1 < 1000)motor( 90 ,100 );
	                break;

	            case 0x60://0110 0000
	                /* 少し右寄り→左へ小曲げ */
					handle( -40 );
	             //   if(cnt1 < 1000)motor( 90 ,100 );
	                break;
			
				 case 0x40://0100 0000
	                /* 少し右寄り→左へ小曲げ */
					handle( -50 );
	             //   if(cnt1 < 1000)motor( 85 ,100 );
	                break;
			
			
				 case 0xe0://1110 0000
	                /* 少し右寄り→左へ小曲げ */
					handle( -60 );
	              //  if(cnt1 < 1000)motor( 80 ,100 );
		
	                break;

	            case 0xc0://1100 0000
	                /* 大きく右寄り→左へ大曲げ */
					handle( -65 );
	             //   if(cnt1 < 1000)motor( 80 ,100 );
	        
	                break;
				
				case 0xc1://1100 0001
	                /* 大きく右寄り→左へ大曲げ */
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
        /* どれでもない場合は待機状態に戻す */
        pattern = 0;
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
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:モータドライブ基板Ver.5  */
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
    static int servo_cnt = 0,servo_angle = 0,cnt10ms = 0;

	if(servo_cnt == 0){
		servo_angle = angle_buf ;	
	}
	
	if(out_flag == 1)trdgrd1 = 0; //コースアウトの場合はサーボ出力をオフにする
	else if(servo_angle >= (PWM_CYCLE)) trdgrd1 = PWM_CYCLE+2;
	else if(servo_angle > 0 )trdgrd1 = servo_angle;
	else trdgrd1 = 0;
	
		
	servo_angle -=  PWM_CYCLE;
	if(servo_angle <= 0)servo_angle = 0;
	

	servo_cnt++;
	if(servo_cnt >= SERVO_PWM_CYCLE){
		servo_cnt = 0; 		
	}
	
	//（注意）この関数は、これより上に処理を記載しないこと（変数宣言はたぶん可能）//////////////////////////
			
    cnt0++;
    cnt1++;
	run_time++;
	LR_cnt++;
	OUT_cnt++;
	
	if(pattern > 10 && run_time > 800){//走行開始直後は0ms/sからのスタートなので XXXXmsはカウントしない
		S_cnt++;//直線走行時間
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
/* タイマ本体                                                           */
/* 引数　 タイマ値 1=1ms                                                */
/************************************************************************/
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
}

/************************************************************************/
/* センサ状態検出                                                       */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~p0;

    sensor &= mask;

    return sensor;
}

/************************************************************************/
/* クロスライン検出処理                                                 */
/* 戻り値 0:クロスラインなし 1:あり                                     */
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
/* 右ハーフライン検出処理                                               */
/* 戻り値 0:なし 1:あり                                                 */
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
/* 左ハーフライン検出処理                                               */
/* 戻り値 0:なし 1:あり                                                 */
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
/* ディップスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3～P1_0読み込み           */

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
/* スタートバー検出センサ読み込み                                       */
/* 戻り値 センサ値 ON(バーあり):1 OFF(なし):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = ~p0;                           /* スタートバー信号読み込み     */
    b &= 0x01;

    return  b;
}

/************************************************************************/
/* LED制御                                                              */
/* 引数　スイッチ値 LED2:bit1 LED3:bit0  "0":消灯 "1":点灯              */
/* 例)0x3→LED2:ON LED3:ON  0x2→LED2:ON LED3:OFF                       */
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
/* 引数　 左モータ:-100～100、右モータ:-100～100                        */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
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
	
    /* 左モータ制御 */
    if( accele_l >= 0 ) {
        p2 &= 0xfd;

        if(accele_l == 100) trdgrd0 = (long)( PWM_CYCLE + 2 );
        else trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        p2 |= 0x02;
        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* 右モータ制御 */
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
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90～90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void handle( int angle )
{
    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
    //trdgrd1 = SERVO_CENTER - angle * HANDLE_STEP;
    angle_buf = SERVO_CENTER - angle * HANDLE_STEP;
	
	//ログのサイズに合わせる（暫定対応）
	if(127 < angle)angle = 127;
	if(angle < -128)angle = -128;
	log_buf_handle = angle;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂内容

Ver.1.00 2012.05.07 作成(センサ基板Ver.4→Ver.5用に変更)
Ver.1.01 2013.04.18 クランク、レーンチェンジ手前の横線が2本から1本になった
                    ことによる、コメントの修正(プログラムの変更はなし)

*/
