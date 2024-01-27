/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     マイコンカートレース基本プログラム(R8C/38A版)               */
/* バージョン   Ver.1.02                                                    */
/* Date         2011.03.18                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/
/*
このプログラムは、下記基板に対応しています。
・RY_R8C38ボード
・モータドライブ基板Ver.4
・センサ基板Ver.4.1
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* 定数設定 */
#define PWM_CYCLE       2499           /* モータPWMの周期   初期値39999  */
#define SERVO_CENTER    4170            /* サーボのセンタ値             */
#define HANDLE_STEP     22              /* 1゜分の値                    */

#define RUN_TIME	21000	//走行時間

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
void handle2( int angle );
void wait( unsigned long set_time );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用                  */
unsigned long   cnt1;                   /* main内で使用                 */
unsigned long   run_time = 0;
int             pattern;                /* パターン番号                 */

int S_angle = 0;
int angle_buf = 0;

unsigned char sensor_old = 0;
unsigned long S_angle_cnt = 0;
unsigned long S_cnt = 0,LR_cnt = 0;

unsigned long OUT_cnt = 0;

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    int     i;
	
	int Out_cnt = 0;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    asm(" fset I ");                    /* 全体の割り込み許可           */

    /* マイコンカーの状態初期化 */
    handle( 0 );
	motor( 0, 0 );

	
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
    21：１本目のクロスライン検出時の処理
    22：２本目を読み飛ばす
    23：クロスライン後のトレース、クランク検出
    31：左クランククリア処理　安定するまで少し待つ
    32：左クランククリア処理　曲げ終わりのチェック
    41：右クランククリア処理　安定するまで少し待つ
    42：右クランククリア処理　曲げ終わりのチェック
    51：１本目の右ハーフライン検出時の処理
    52：２本目を読み飛ばす
    53：右ハーフライン後のトレース
    54：右レーンチェンジ終了のチェック
    61：１本目の左ハーフライン検出時の処理
    62：２本目を読み飛ばす
    63：左ハーフライン後のトレース
    64：左レーンチェンジ終了のチェック
    *****************************************************************/

    case 0:
        /* スイッチ入力待ち */
        if( pushsw_get() ) {
            pattern = 1;
			
			timer(1000);
			
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
			S_angle = 0;
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

		LR_cnt = 0;
		
        switch( sensor_inp(MASK4_4) ) {
            case 0x18: //000 1 000
                /* センタ→まっすぐ */
                //handle2( S_angle );
				
				handle( 0 );
                motor( 100 ,100 );
                break;
			///////////////////////////////////////////////////////////
			
			case 0x1C://000 1 100
                /* 微妙に左寄り→右へ微曲げ */
			/*	if(S_angle_cnt > 10){
					S_angle_cnt = 0;
					S_angle += 1;
				}
				
                handle2( S_angle );
			*/	
				handle( 3 );
                motor( 100 ,100 );
                break;
				
            case 0x04://000 0 100
                /* 微妙に左寄り→右へ微曲げ */
                //handle2( 8 * HANDLE_STEP + S_angle);
				
			/*	if(S_angle_cnt > 5){
					S_angle_cnt = 0;
					S_angle += 1;
				}
				
                handle2( S_angle );
			*/	
				handle( 5 );
                motor( 100 ,98 );
                break;

            case 0x06://000 0 110
                /* 少し左寄り→右へ小曲げ */
                handle( 10 );
                motor( 90 ,50 );
				
                break;
			
			case 0x07://000 0 111
                /* 少し左寄り→右へ小曲げ */
                handle( 20 );
                motor( 90 ,30 );
				pattern = 12;
                break;

            case 0x03://000 0 011
                /* 大きく左寄り→右へ大曲げ */
                pattern = 12;
                break;

			///////////////////////////////////////////////////////
			
			case 0x38://001 1 000
                /* 微妙に右寄り→左へ微曲げ */
			/*	if(S_angle_cnt > 10){
					S_angle_cnt = 0;
					S_angle -= 1;
				}
				
                handle2( S_angle );
			*/	
				handle( -3 );
                motor( 100 ,100 );
                break;
				
            case 0x20://001 0 000
                /* 微妙に右寄り→左へ微曲げ */
                //handle2( -5 * HANDLE_STEP + S_angle);
				
			/*	if(S_angle_cnt > 5){
					S_angle_cnt = 0;
					S_angle -= 1;
				}
				
                handle2( S_angle );
		*/		
				handle( -5 );
                motor( 98 ,100 );
                break;

            case 0x60://011 0 000
                /* 少し右寄り→左へ小曲げ */
				handle( -10 );
                motor( 50 ,90 );
                break;
			
			 case 0xe0://111 0 000
                /* 少し右寄り→左へ小曲げ */
				handle( -20 );
                motor( 30 ,90 );
				pattern = 13;
                break;

            case 0xc0://110 0 000
                /* 大きく右寄り→左へ大曲げ */
                pattern = 13;
                break;

            default:
                break;
        }
        break;

    case 12:
		
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
		
		if(S_cnt > 1000){	//一定時間直線が続いた後のカーブ　＝　ブレーキ
		
			handle( 30 );
        	motor( -50 ,-50 );
			
			if(LR_cnt > 400){ //ブレーキ終了時間
				S_cnt = 0;	
			}
		}else if(S_cnt > 500){	//一定時間直線が続いた後のカーブ　＝　ブレーキ
		
			handle( 30 );
        	motor( -20 ,-20 );
			
			if(LR_cnt > 300){ //ブレーキ終了時間
				S_cnt = 0;	
			}
		}else if(sensor_inp(MASK4_4)&0x18 != 0x00 ){ //xxx 1 xxx 外寄りすぎ
		
			handle( 30 );
       		motor( 80 ,-20 );
			
			S_cnt = 0;
		}else if(LR_cnt < 500 || sensor_inp(MASK4_4)&0x20 != 0x00){ //カーブ前半 || xx1 x xxx
		
			handle( 30 );
       		motor( 80 ,15 );
			
			S_cnt = 0;
		
		}else{//カーブ後半
		
			handle( 30 );
        	motor( 100 ,90 );
			
			S_cnt = 0;
		}
		
				
		/* 右へ大曲げの終わりのチェック */
        if( sensor_inp(MASK4_4) == 0x06) {//000 0 110
		/*	if(sensor_old != 0x1C && sensor_old != 0x04){//外側に膨らんでる  //000 1 100  //000 0 100
				S_cnt = 9000;//ブレーキするため
				LR_cnt = 0;
				
			}else{
         */   	pattern = 11;
				S_cnt = 0;
		//	}
        }
		sensor_old = sensor_inp(MASK4_4);
        break;

    case 13:
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
		
		if(S_cnt > 1000){	//一定時間直線が続いた後のカーブ　＝　ブレーキ
		
			handle( -30 );
        	motor( -50 ,-50 );
			
			if(LR_cnt > 400){ //ブレーキ終了時間
				S_cnt = 0;	
			}
		}else if(S_cnt > 500){	//一定時間直線が続いた後のカーブ　＝　ブレーキ
		
			handle( -30 );
        	motor( -20 ,-20 );
			
			if(LR_cnt > 300){ //ブレーキ終了時間
				S_cnt = 0;	
			}
		}else if(sensor_inp(MASK4_4)&0x18 != 0x00 ){ //xxx 1 xxx 外寄りすぎ
		
			handle( -30 );
       		motor( -20 ,90 );
			
			S_cnt = 0;
		}else if(LR_cnt < 500 || sensor_inp(MASK4_4)&0x04 != 0x00 ){ //カーブ前半 || xxx x 1xx
		
			handle( -30 );
       		motor( 15 ,80 );
			
			S_cnt = 0;
		
		}else{//カーブ後半
		
			handle( -30 );
        	motor( 90 ,100 );
			
			S_cnt = 0;
		}
		
		
		 /* 左へ大曲げの終わりのチェック */		
        if( sensor_inp(MASK4_4) == 0x60 ) {//011 0 000
		/*	if(sensor_old != 0x31 && sensor_old != 0x20){//外側に膨らんでる  //001 1 000  //001 0 000
				S_cnt = 9000;//ブレーキするため
				LR_cnt = 0;
				
			}else{
          */  	pattern = 11;
				S_cnt = 0;
		//	}
        }
		sensor_old = sensor_inp(MASK4_4);
        break;

    case 21:
        /* １本目のクロスライン検出時の処理 */
        led_out( 0x3 );
        handle( 0 );
        motor( -50 ,-50 );
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
        if( check_leftline() ) {
            /* 左クランクと判断→左クランククリア処理へ */
            led_out( 0x1 );
            handle( -30 );
            motor( -100 ,100 );
            pattern = 31;
            cnt1 = 0;
            break;
        }
        if( check_rightline() ) {
            /* 右クランクと判断→右クランククリア処理へ */
            led_out( 0x2 );
            handle( 30 );
            motor( 100 ,-100 );
            pattern = 41;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* センタ→まっすぐ */
                handle( 0 );
                motor( 70 ,70 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* 左寄り→右曲げ */
                handle( 8 );
                motor( 70 ,50 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* 右寄り→左曲げ */
                handle( -8 );
                motor( 50 ,70 );
                break;
        }
        break;


    case 31:
		if( sensor_inp(MASK4_4)& 0x38 != 0x00 ) { //xx1 1 xxx 外より
			handle( -30 );
        	motor( -100 ,100 );
			
		}else if(sensor_inp(MASK4_4)& 0xC0 != 0x00 && (sensor_old & 0x20) != 0x00){//11x x xxx 外より
			handle( -30 );
        	motor( -100 ,100 );
			
		}else{
			handle( -25 );
        	motor( -50 ,100 );
		}
			
        /* 左クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 &&  sensor_inp(MASK3_3) == 0xC0  && (sensor_old & 0x18) == 0x00) {
            led_out( 0x0 );
			
            pattern = 32;
            cnt1 = 0;
        }
		
		sensor_old = sensor_inp(MASK4_4);
        break;

	case 32:
		if( sensor_inp(MASK4_4)& 0x04 != 0x00 ) { //xxx x 1xx 外より
			handle( -20 );
        	motor(  0 ,100 );
		}else{
			handle( -15 );
        	motor( 50 ,100 );
		}
		
        /* 左クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 && sensor_inp(MASK3_3) == 0x60 && (sensor_old & 0x18) == 0x00) {
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
		sensor_old = sensor_inp(MASK4_4);
        break;
		
    case 41:
        if( sensor_inp(MASK4_4)& 0x1C != 0x00 ) { //xxx 1 1xx 外より
			handle( 30 );
        	motor( 100 ,-100 );
			
		}else if(sensor_inp(MASK4_4)& 0x03 != 0x00 && (sensor_old & 0x04) != 0x00){//xxx x x11 外より
			handle( 30 );
        	motor( 100 ,-100 );
			
		}else{
			handle( 25 );
        	motor( 100 ,-50 );
		}
			
        /* 右クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 &&  sensor_inp(MASK3_3) == 0x03 && (sensor_old & 0x18) == 0x00) {
            led_out( 0x0 );
			
            pattern = 42;
            cnt1 = 0;
        }
		
		sensor_old = sensor_inp(MASK4_4);
        break;

    case 42:
		if( sensor_inp(MASK4_4)& 0x20 != 0x00 ) { //xx1 x xxx 外より
        	handle( 20 );
        	motor( 100 , 0 );
		}else{
			handle( 15 );
        	motor( 100 ,50 );
		}
		
        /* 右クランククリア処理　曲げ終わりのチェック */
        if(cnt1 > 5 && sensor_inp(MASK3_3) == 0x06 && (sensor_old & 0x18) == 0x00) {
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
		sensor_old = sensor_inp(MASK4_4);
        break;


    case 51:
        /* １本目の右ハーフライン検出時の処理 */
        led_out( 0x2 );
        handle( 0 );
        motor( 0 ,0 );
        pattern = 52;
        cnt1 = 0;
        break;

    case 52:
        /* ２本目を読み飛ばす */
        if( cnt1 > 100 ) {
            pattern = 53;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_leftline() ) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
       
        break;

    case 53:
        /* 右ハーフライン後のトレース、レーンチェンジ */
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( 30 );
            motor( 70 ,30 );
            pattern = 54;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* センタ→まっすぐ */
                handle( 0 );
                motor( 80 ,80 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* 左寄り→右曲げ */
                handle( 8 );
                motor( 80 ,50 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* 右寄り→左曲げ */
                handle( -8 );
                motor( 50 ,80 );
                break;
            default:
                break;
        }
        break;

    case 54:
        /* 右レーンチェンジ終了のチェック */
        if( sensor_inp( MASK4_4 ) == 0x01 || sensor_inp( MASK4_4 ) == 0x03 ) {
            handle( -5 );
            motor( 50 ,50 );
          
			
            led_out( 0x0 );
            pattern = 55;
            cnt1 = 0;
        }
        break;
			
    case 55:
        /* 右レーンチェンジ終了のチェック */
        if( sensor_inp( MASK4_4 ) == 0x60 ) {
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
        break;

    case 62:
        /* ２本目を読み飛ばす */
        if( cnt1 > 100 ) {
            pattern = 63;
            cnt1 = 0;
        }
		
		if( check_crossline() || check_rightline()) {       // クロスラインチェック         
            pattern = 21;
            break;
        }
        break;

    case 63:
        /* 左ハーフライン後のトレース、レーンチェンジ */
        if( sensor_inp(MASK4_4) == 0x00 ) {
            handle( -30 );
            motor( 30 ,70 );
            pattern = 64;
            cnt1 = 0;
            break;
        }
        switch( sensor_inp(MASK3_3) ) {
            case 0x00:
                /* センタ→まっすぐ */
                handle( 0 );
                motor( 80 ,80 );
                break;
            case 0x04:
            case 0x06:
            case 0x07:
            case 0x03:
                /* 左寄り→右曲げ */
                handle( 8 );
                motor( 80 ,50 );
                break;
            case 0x20:
            case 0x60:
            case 0xe0:
            case 0xc0:
                /* 右寄り→左曲げ */
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
        /* 左レーンチェンジ終了のチェック */
        if( sensor_inp( MASK4_4 ) == 0x06 ) {
            led_out( 0x0 );
            pattern = 11;
			S_cnt = 0;
            cnt1 = 0;
        }
        break;

	case 99://走行終了
		 motor( 0 ,0 );
		 
		 if(cnt1 < 500){
			 switch( sensor_inp(MASK3_3) ) {
	            case 0x00:
	                /* センタ→まっすぐ */
	                handle( 0 );
              
	                break;
	            case 0x04:
	            case 0x06:
	            case 0x07:
	            case 0x03:
	                /* 左寄り→右曲げ */
	                handle( 8 );
               
	                break;
	            case 0x20:
	            case 0x60:
	            case 0xe0:
	            case 0xc0:
	                /* 右寄り→左曲げ */
	                handle( -8 );
               
	                break;
	        }
		 }else{
			 handle( 0 );
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
    trdgra0 = trdgrc0 = PWM_CYCLE;      /* 周期                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅設定           */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅設定           */
    trdgrb1 = trdgrd1 = PWM_CYCLE/2;   /* P2_5端子のON幅設定           */
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
		
    cnt0++;
    cnt1++;
	run_time++;
	S_cnt++;
	LR_cnt++;
	S_angle_cnt++;
	OUT_cnt++;
	
	
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
    sensor  &= 0xef;
    if( sensor & 0x08 ) sensor |= 0x10;

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
	//111 1 111     //011 1 110
    if( b==0xff  || b==0x7e) {
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
	//000 1 111  //000 1 110 
    if( b==0x1f || b==0x1e   ) {
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
	//111 1 000   //011 1 000 
    if( b==0xf8 ||  b==0x78  ) {
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
    b &= 0x10;
    b >>= 4;

    return  b;
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
/* 引数　 左モータ:-100～100、右モータ:-100～100                        */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
    int    sw_data;

   // sw_data = dipsw_get() + 5;
   // accele_l = accele_l * sw_data / 20;
   // accele_r = accele_r * sw_data / 20;

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
}

void handle2( int angle )
{
    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
    //trdgrd1 = SERVO_CENTER - angle;
	angle_buf = SERVO_CENTER - angle;
}


/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂内容

Ver.1.00 2010.04.01 作成
Ver.1.01 2011.03.14 タイマRDのレジスタの設定順の変更
Ver.1.02 2011.03.18 motor関数の引数の変数名変更
*/
