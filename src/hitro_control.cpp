//subscriberとpublisherのテンプレ。

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <stdio.h>
#include <time.h>
#include <sensor_msgs/Joy.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

#define PI 3.141592
int s0 = 0;
int s1 = 0;
int s2 = 0;
int s3 = 0;
int s4 = 0;
float speed = 5;
float joy_axes[6] = {0,0,0,0,0,0};
float x = 0 ;
float y = 0 ;
float max = 0 ;

int b[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};


//////////////////////////////////////////////////////////


float joint_tip = 0;
float joint_plus = 0;


//逆運動学パラメータ
float angle1 = 0;
float angle2 = 0;
float angle1_ = 0;
float angle2_ = 0;

float arm_x = -0.126557 ;  //アームのx座標
float arm_y = -0.063921;  //アームのy座標
float arm_xpast = arm_x; //一つ前のアームの位置
float arm_ypast = arm_y; //一つ前のアームの位置
float arm_x2 = arm_x ;  // アームのx座標　座標変換後
float arm_y2 = arm_y;  // アームのy座標　座標変換後
float arm_x2past = arm_x; //一つ前のアームの位置
float arm_y2past = arm_y; //一つ前のアームの位置
float A = PI ; //3リンクの３つ目の式の条件 　（シータ１　＋　シータ２　＋　シータ３　＝　A　）

//float l1 = 0.225;   //リンク１の長さ
//float l2 = 0.152;    //リンク2の長さ
//float l3 = 0.188 ;  // リンクの長さ

float l1 = 0.2215;
float l2 = 0.232;
float l3 = 0.188 ;  // リンクの長さ

float goal_pos ;
float current_pos ;	//モーター１７番（sakeの手）のデータの取得する　　関数外で定義したからグローバル変数のはず
float error ;
float load ;

/////////////////////////////////////////////////////////////


class Hitro_control		//classの定義
{
	public:
		Hitro_control();
		void timerCallback(const ros::TimerEvent&);//時間指定でイベント実行   //t秒間隔でtimerCallbackを実行するcakkbackの定義
		void TopicCallback(const sensor_msgs::Joy &joy_msg);		//TopicCallbackを定義
		void flipper_left_rear_callback(const std_msgs::Float64ConstPtr &msg);
		void flipper_right_rear_callback(const std_msgs::Float64ConstPtr &msg);
		void flipper_left_front_callback(const std_msgs::Float64ConstPtr &msg);
		void flipper_right_front_callback(const std_msgs::Float64ConstPtr &msg);

		void joint_callback17(const dynamixel_msgs::JointState& pos);



	private:
		ros::Publisher pub[2];
		ros::Subscriber sub[4];
		ros::Timer timer[1];
 		//ros::Subscriber scan_sub;

		ros::Subscriber joint_callback17_sub;
		//ros::Subscriber callback17_sub;

 		ros::Subscriber joy_sub;
		ros::Publisher dyn_pub11;
		ros::Publisher dyn_pub12;
		ros::Publisher flipper[4];
		ros::NodeHandle nh;
		
		float flipper_left_front_val;
		float flipper_right_front_val;
		float flipper_left_rear_val;
		float flipper_right_rear_val;

		std_msgs::Float64 test_topic,test_value_10times;
		double scale,alpha;

		std_msgs::Float64 pos[6];

		std_msgs::Float64 pos_dy[9];

		ros::Publisher dyn_pub[9];

};


Hitro_control::Hitro_control()//最初に一度だけ読まれる
{
  timer[0] = nh.createTimer(ros::Duration(0.1), &Hitro_control::timerCallback, this);//0.02秒間隔でTestclass::timerCallbackを実行
  //pub[0] = nh.advertise<std_msgs::Float64>("/test_value", 1); //"/test_value"という名前のトピックをstd_msgs::Float64型でパブリッシュ
  sub[0] = nh.subscribe("/flipper_left_rear", 1, &Hitro_control::flipper_left_rear_callback,this);
  sub[1] = nh.subscribe("/flipper_right_rear", 1, &Hitro_control::flipper_right_rear_callback,this);
  sub[2] = nh.subscribe("/flipper_left_front", 1, &Hitro_control::flipper_left_rear_callback,this);
  sub[3] = nh.subscribe("/flipper_right_front", 1, &Hitro_control::flipper_right_front_callback,this);
  

  test_topic.data=0;//初期値

  joint_callback17_sub = nh.subscribe("/tilt_controller17/state", 1, &Hitro_control::joint_callback17, this);
  //callback17_sub = nh.subscribe("/motor_states/pan_tilt_port", 1, &Testclass::callback17, this);


  joy_sub = nh.subscribe("joy", 1, &Hitro_control::TopicCallback, this);
  //scan_sub = nh.subscribe("scan" , 1 , &Testclass::TopicCallback, this);
  dyn_pub11 = nh.advertise<std_msgs::Float64>("/drive_right_speed", 1);
  dyn_pub12 = nh.advertise<std_msgs::Float64>("/drive_left_speed", 1);

	flipper[0] = nh.advertise<std_msgs::Float64>("/flipper_left_rear/speed", 1);
	flipper[1] = nh.advertise<std_msgs::Float64>("/flipper_right_rear/speed", 1);
	flipper[2] = nh.advertise<std_msgs::Float64>("/flipper_left_front/speed", 1);
	flipper[3] = nh.advertise<std_msgs::Float64>("/flipper_right_front/speed", 1);


	dyn_pub[0] = nh.advertise<std_msgs::Float64>("/tilt_controller10/command", 1);
	dyn_pub[1] = nh.advertise<std_msgs::Float64>("/tilt_controller11/command", 1);
	dyn_pub[2] = nh.advertise<std_msgs::Float64>("/tilt_controller12/command", 1);
	dyn_pub[3] = nh.advertise<std_msgs::Float64>("/tilt_controller13/command", 1);
	dyn_pub[4] = nh.advertise<std_msgs::Float64>("/tilt_controller14/command", 1);
	dyn_pub[5] = nh.advertise<std_msgs::Float64>("/tilt_controller15/command", 1);
	dyn_pub[6] = nh.advertise<std_msgs::Float64>("/tilt_controller16/command", 1);
	dyn_pub[7] = nh.advertise<std_msgs::Float64>("/tilt_controller17/command", 1);
	dyn_pub[8] = nh.advertise<std_msgs::Float64>("/tilt_controller18/command", 1);
}

void Hitro_control::joint_callback17(const dynamixel_msgs::JointState& pos)  //dynamixel17番のモーターの目標位置などを取得した時に動作するコールバック関数
{


	//goal_pos = pos.goal_pos ;  //　モーターの目標位置の取得
	//current_pos = pos.current_pos ; // モーターの現在位置の取得
	//error = pos.error ;  //　モーターの目標位置と現在位置の誤差値の取得
	load = pos.load ;  // モーターの負荷トルクの取得


	//ROS_INFO("//////////goal_pos:%f",goal_pos);
	//ROS_INFO("current_pos:%f",current_pos);
	
/*	if(goal_pos - current_pos  > 0.017)
	{ 
		ROS_INFO("error:%f",error);
		
	}
*/
	
}


//void Testclass::callback17(const dynamixel_msgs::MotorStateList& pos17)　 //dynamixel17番のモーターの目標位置などを取得した時に動作するコールバック関数
//{
/*
float goal_pos2;
float current_pos2;
float error2;
	goal_pos2 = pos.goal_pos ;
	current_pos2 = pos.current_pos ;
	error2 = pos.error ;
	ROS_INFO("//////////goal_pos:%f",goal_pos);
	ROS_INFO("current_pos:%f",current_pos);
	if(goal_pos - current_pos  > 0.02)
	{ 
		ROS_INFO("error:%f",error);

	}


}*/

void Hitro_control::flipper_left_rear_callback(const std_msgs::Float64ConstPtr & msg) {
    flipper_left_rear_val=msg->data;
}

void Hitro_control::flipper_right_rear_callback(const std_msgs::Float64ConstPtr & msg) {
    flipper_right_rear_val=msg->data;
}

void Hitro_control::flipper_left_front_callback(const std_msgs::Float64ConstPtr & msg) {
    flipper_left_front_val=msg->data;
}

void Hitro_control::flipper_right_front_callback(const std_msgs::Float64ConstPtr & msg) {
    flipper_right_front_val=msg->data;
}


void Hitro_control::TopicCallback(const sensor_msgs::Joy &joy_msg)//test_valueをsubscribeして10倍にするプログラム
{
 //test_value_10times.data=callback_value->data*10;
 //ROS_INFO("test_value_10times=%f",test_value_10times.data);//表示

 	pos[0].data = 0; //左前
	pos[1].data = 0; //右前
	
	joy_axes[1] = joy_msg.axes[1];//上下
	joy_axes[0] = -joy_msg.axes[0];//左右 (左が正の時はこちらを使う)(左が正の時は、右を正にするためにマイナスをかける)

	/*if (abs(joy_msg.axes[1]) > 0.05)
	{
		joy_axes[1] = joy_msg.axes[1];//上下
	}
	else
	{
		joy_axes[1] = 0;//上下
	}
	
	if (abs(joy_msg.axes[0]) > 0.05)
	{
		joy_axes[0] = -joy_msg.axes[0];//左右 (左が正の時はこちらを使う)(左が正の時は、右を正にするためにマイナスをかける)
	}
	else
	{
		joy_axes[0] = 0;
	}
	* */

	if ( joy_axes[1] != 0 && -0.15 < joy_axes[0] && joy_axes[0] < 0.15 )  s0 = 1 ; //前進または後退		//前進または後退のみの範囲を広げる
	else if ( joy_axes[0] != 0 && -0.15 < joy_axes[1] && joy_axes[1] < 0.15 )  s0 = 2 ; //その場右旋回	//その場旋回のみの範囲を広げる
	else 									  s0 = 0 ;

	if ( joy_axes[1] >= 0 && joy_axes[0] >= 0)		s1 = 1 ;		//右カーブ前進
	else						s1 = 0 ;

	if (joy_axes[1] >= 0 && joy_axes[0] < 0)			s2 = 1 ;		//左カーブ前進
	else						s2 = 0 ;

	if (joy_axes[1] < 0 && joy_axes[0] < 0)			s3 = 1 ;		//右カーブ後退??
	else						s3 = 0 ;

	if (joy_axes[1] < 0 && joy_axes[0] >= 0)			s4 = 1 ;		//左カーブ後退??

	else						s4 = 0 ;


	joy_axes[2] = joy_msg.axes[2];//右ジョイスティック横方向 ひだりが１　右が−１
	joy_axes[3] = joy_msg.axes[3];//右ジョイスティック縦方向　うえが１　下が−１
	joy_axes[4] = joy_msg.axes[4];//十字キー
	joy_axes[5] = joy_msg.axes[5];//十字キー

	if(joy_msg.buttons[0] == 1)//△ボタン 手首上げる
	{
		b[0] = 1;
		b[1] = 0;
		b[2] = 0;
		b[3] = 0;
		joy_axes[2]= 0;
		//ROS_INFO("sankaku");
	}
	else if(joy_msg.buttons[1] == 1)//○ボタン 手首回転 
	{
		b[0] = 0;
		b[1] = 1;
		b[2] = 0;
		b[3] = 0;
		joy_axes[2]= 0;
		//ROS_INFO("maru");
	}
	else if(joy_msg.buttons[2] == 1)//×ボタン 手首下ろす
	{
		b[0] = 0;
		b[1] = 0;
		b[2] = 1;
		b[3] = 0;
		joy_axes[2]= 0;
		//ROS_INFO("batsu");
	}
	else if(joy_msg.buttons[3] == 1)//□ボタン 手首逆回転
	{
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
		b[3] = 1;
		joy_axes[2]= 0;
		//ROS_INFO("sikaku");
	}
	else if(joy_axes[2] == 1)//右ジョイスティック
	{
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
		b[3] = 0;
		joy_axes[2]= 1;
		//ROS_INFO("sikaku");
	}

	else if(joy_axes[2] == -1)//右ジョイスティック
	{
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
		b[3] = 0;
		joy_axes[2]= -1;
		//ROS_INFO("sikaku");
	}
	else{
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
		b[3] = 0;
		joy_axes[2]= 0;
	}

	b[4] = joy_msg.buttons[4];//L2ボタン  　
	b[5] = joy_msg.buttons[5];//R2ボタン	フリッパーのBR　または	アーム開く R2ボタン
	b[6] = joy_msg.buttons[6];//L1ボタン
	b[7] = joy_msg.buttons[7];//R1ボタン      フリッパーのFR　または　アーム把持 R1ボタン 
	b[8] = joy_msg.buttons[8];//startボタン



	if(joy_msg.buttons[10] == 1)//左ジョイスティックボタン
	{
		b[9] = 0;
		b[10] = 1;  //アームモード
		b[11] = 0;
	}
	else if(joy_msg.buttons[11] == 1)//右ジョイスティックボタン
	{
		b[9] = 0;
		b[10] = 0;
		b[11] = 1;  
	}

	else if(joy_msg.buttons[9] == 1)//SELECT
	{
		b[9] = 1;  //SELECTボタンフリッパーモード
		b[10] = 0;
		b[11] = 0;
	}


}

void Hitro_control::timerCallback(const ros::TimerEvent&)//0.02秒間隔で実行されるプログラム。ここでは/test_value（test_topic）をpublishしている
{


	//SELECTボタンフリッパーモード
	if(b[9] == 1) //SELECTボタン

	{
		if (joy_axes[5] == 1)
		{
			pos[2].data = 1;
		}
		else if (joy_axes[5] == -1)
		{
			pos[2].data = -1;
		}
		else
		{
			pos[2].data = 0;
		}



		if (b[0] == 1)//BR
		{
			pos[3].data = 1;
		}
		else if (b[2] == 1)
		{
			pos[3].data = -1;
		}
		else
		{
			pos[3].data = 0;
		}



		if (b[6] == 1)//FL
		{
			pos[4].data = 1;
		}
		else if (b[4] == 1)
		{
			pos[4].data = -1;
		}
		else
		{
			pos[4].data = 0;
		}


		if (b[7] == 1)
		{
			pos[5].data = 1;
		}
		else if (b[5] == 1)
		{
			pos[5].data = -1;
		}
		else
		{
			pos[5].data = 0;
		}


		flipper[0].publish(pos[2]);
		flipper[1].publish(pos[3]);
		flipper[2].publish(pos[4]);
		flipper[3].publish(pos[5]);





	}
	
		//フリッパーモード同期
	if(b[11] == 1) //右ジョイスティックボタン

	{
		if (b[0] == 1)//BR
		{
			pos[3].data = 1;
			pos[2].data = 1;
		}
		else if (b[2] == 1)
		{
			pos[3].data = -1;
			pos[2].data = -1;
		}
		else
		{
			pos[3].data = 0;
			pos[2].data = 0;
		}

		if (b[7] == 1)
		{
			pos[5].data = 1;
			pos[4].data = 1;
		}
		else if (b[5] == 1)
		{
			pos[5].data = -1;
			pos[4].data = -1;
		}
		else
		{
			pos[5].data = 0;
			pos[4].data = 0;
		}

		flipper[0].publish(pos[2]);
		flipper[1].publish(pos[3]);
		flipper[2].publish(pos[4]);
		flipper[3].publish(pos[5]);





	}

/*
	//左ジョイスティックボタン  アームモード
	if(b[10] == 1)  //左ジョイスティックボタン
	{
		if(b[8] == 1)   //アームの位置の初期化
		{

		arm_x = -0.126557;  //アームのx座標
		arm_y = -0.063921;  //アームのy座標
		arm_x2 =  arm_x;  // アームのx座標　座標変換後
		arm_y2 = arm_y ;  // アームのy座標　座標変換後
		joint_plus = -0.250000;
		angle1 = -1.285572 ;
		angle2 = -2.868644 ;
		pos_dy[4].data = 0 ;
		pos_dy[6].data = 0 ;
		pos_dy[7].data = -0.3300 ;
		//joint_tip = 0.110638 ; 
		arm_x2past = arm_x; //一つ前のアームの位置
		arm_y2past = arm_y; //一つ前のアームの位置
		}




		//ROS_INFO("X:%f Y:%f\n",arm_x,arm_y);

		/////////////////////////////////////////////////////
		if(b[4] == 1)//下げる //L2ボタン
		{
			arm_y += 0.005*cos(angle1+angle2-joint_tip);  //　アームをカメラの向いているのの上下に動く
			arm_x -= 0.005*sin(angle1+angle2-joint_tip);
		}

		if(b[6] == 1)//上げる //L1ボタン
		{
			arm_y -= 0.005*cos(angle1+angle2-joint_tip);
			arm_x += 0.005*sin(angle1+angle2-joint_tip);
		}

		if(joy_axes[5] == 1)//前に //十字キー		//　アームのカメラに向いている方向に動く
		{
			arm_x += 0.005*cos(angle1+angle2-joint_tip);
			arm_y += 0.005*sin(angle1+angle2-joint_tip);
			
		}

		if(joy_axes[5] == -1)//後ろに //十字キー 		
		{
			arm_x -= 0.005*cos(angle1+angle2-joint_tip);
			arm_y -= 0.005*sin(angle1+angle2-joint_tip);
			
		}



		if(joy_axes[4] == 1)//前に   //十字キー　　//baseヨー回転
		{
			pos_dy[0].data += 0.05;  //10番のモーターに入力
		}

		if(joy_axes[4] == -1)//後ろに  //十字キー　　//baseヨー回転
		{
			pos_dy[0].data -= 0.05;  //10番のモーターに入力
		}



		if(b[0] == 1)//手首上げる △ボタン
		{
			joint_plus += 0.05;
		}

		if(b[2] == 1)//手首下ろす ×ボタン
		{
			joint_plus += -0.05;
		}
		
		float angle_before3 = angle2  - PI/2 + angle1 + joint_plus;

		if( isnan(  angle_before3) == false ) // NaNなら避けるを避ける 
		{
			joint_tip = angle_before3 ;
			pos_dy[3].data = -joint_tip;     //　手首ピッチ回転
			ROS_INFO("complete(angle3");
		}
		else ROS_INFO("error(angle3");

		if(b[1] == 1)//手首回転 ○ボタン
		{
			pos_dy[6].data += 0.1;  //14番のモーターに入力(手首ロール)
		}

		if(b[3] == 1)//手首回転 □ボタン
		{
			pos_dy[6].data -= 0.1; //14番のモーターに入力(手首ロール)
		}

		if(joy_axes[2] == 1)//手首ヨー回転  右ジョイスティック横方が左に入力
		{
			pos_dy[4].data += 0.1;
		}
		
		if(joy_axes[2] == -1)//手首ヨー回転  右ジョイスティック横方が右に入力
		{
			pos_dy[4].data -= 0.1;
		}


float pos_dy7_past = pos_dy[7].data ;
		if(b[7] == 1)//把持 R1ボタン
		{	
			if(load < 0.3 )
			{
				//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data -= 0.03; //17番のモーターに入力(手）
				printf("///////////////////////////////+0.03\n");
			}
/*			else if(load < 0.33)	
			{	//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data -= 0.005; //17番のモーターに入力(手）
				printf("//////////////////////////////////////////////////////+0.005\n");
			}
			else if(load < 0.39)	
			{	//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data -= 0.001; //17番のモーターに入力(手）
				printf("//////////////////////////////////////////////////////+0.001\n");
			}
			else	pos_dy[7].data = pos_dy7_past ; //17番のモーターに入力(手）
*//*
			else    pos_dy[7].data -= (0.4 - load ) * 0.1 ;    // 0.4付近にする  負荷トルクが0.41付近だと落ちる
		}

		if(b[5] == 1)//開く R2ボタン
		{	
			
			pos_dy[7].data += 0.03; //17番のモーターに入力(手）

/*
			if(load < 0.3 )
			{
				//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data += 0.03; //17番のモーターに入力(手）
				printf("///////////////////////////////-0.03\n");
			}
			else if(load < 0.33)	
			{	//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data += 0.005; //17番のモーターに入力(手）
				printf("//////////////////////////////////////////////////////-0.005\n");
			}
			else 	
			{	//pos_dy7_past = pos_dy[7].data ;
				pos_dy[7].data += 0.005; //17番のモーターに入力(手）
				printf("//////////////////////////////////////////////////////-0.001\n");
			}
*//*
		}
		
		//カメラ方向に座標変換
		//arm_x2 = cos( angle1+angle2-joint_tip )*arm_x  +  sin( angle1+angle2-joint_tip )*arm_y ;
		//arm_y2= -sin( angle1+angle2-joint_tip )*arm_x  +  cos( angle1+angle2-joint_tip )*arm_y ;


		//ROS_INFO("angle[0]:%f",pos[0].data);
		arm_x2 = arm_x ;
		arm_y2 = arm_y ;
		ROS_INFO("X2:%f Y2:%f\n",arm_x2,arm_y2);

		// 3リンクの逆運動学
		float angle_before2 = acos( ( arm_x2*arm_x2 + arm_y2*arm_y2 - l1*l1- l2*l2 +l3*l3 -2*arm_x2*l3*cos(A) -2*arm_y2*l3*sin(A)) /(2*l1*l2));
		float angle_before1 = atan( (arm_y2-l3*sin(A))/ (arm_x2-l3*cos(A)) ) - acos( ( l1*l1 + pow(arm_x2-l3*cos(A) , 2 ) + pow(arm_y2-l3*sin(A),2) - l2*l2) / (2*l1*sqrt(pow(arm_x2-l3*cos(A), 2 ) +pow(arm_y2-l3*sin(A),2) )  ) );

		// 3リンクの逆運動学
		if( isnan( angle_before1 ) == false ) // NaNなら避けるを避ける 
		{
			if( isnan( angle_before2 ) == false ) // NaNなら避けるを避ける
			{
				angle2 = angle_before2 ;
				angle1 = angle_before1 ;
				ROS_INFO("angle2 complete");
			}
			else  	ROS_INFO("error(angle1)");
		}
		else  
		{	ROS_INFO("error(angle2)");}
		// ２リンクの逆運動学
		//angle2 = acos( ( arm_x2*arm_x2 + arm_y2*arm_y2 - l1*l1 - l2*l2 ) / (2*l1*l2) ) ;
		//angle1 = atan( arm_y2 / arm_x2 ) - acos( ( l1*l1 + arm_x2*arm_x2 + arm_y2*arm_y2 - l2*l2) / (2*l1*sqrt(arm_x2*arm_x2 +arm_y2*arm_y2) ) );
		
		// ２リンクの逆運動学
		//angle1_ = acos((pow(l1,2)-pow(l2,2)+(pow(arm_x2,2)+pow(arm_y2,2)))/((2*l1)*sqrt(pow(arm_x2,2)+pow(arm_y2,2))));
		//angle1 = atan2(arm_y2,arm_x2)-angle1_;
		//angle2_ = acos((pow(l2,2)-pow(l1,2)+(pow(arm_x2,2)+pow(arm_y2,2)))/((2*l2)*sqrt(pow(arm_x2,2)+pow(arm_y2,2))));
		//angle2 = angle1_ + angle2_;

		pos_dy[1].data= angle1;   //11番のモーターに入力
		pos_dy[2].data = angle2;  //12番のモーターに入力

		////////////////////////////////////////////////////////////


			//pos_dy[1].data = -pos_dy[1].data;  
			pos_dy[2].data = -pos_dy[2].data;  
			


			pos_dy[5].data = -pos_dy[1].data;  //11番と15番のモーターは逆に動かす
			pos_dy[8].data = -pos_dy[2].data;  //12番と18番のモーターは逆に動かす

		ROS_INFO("angle[1]:%f",pos_dy[1].data);
		ROS_INFO("angle[2]:%f",pos_dy[2].data);
		ROS_INFO("angle[3]:%f",pos_dy[3].data);
		ROS_INFO("angle[4]:%f",pos_dy[4].data);	
		ROS_INFO("angle[5]:%f",pos_dy[5].data);
		ROS_INFO("angle[6]:%f",pos_dy[6].data);
		ROS_INFO("angle[7]:%f",pos_dy[7].data);
		//ROS_INFO("pos_dy7_past:%f",pos_dy7_past);
		//printf("//////////////////////////////pos_dy7_past:%f\n",pos_dy7_past);
		ROS_INFO("angle[8]:%f",pos_dy[8].data);
		ROS_INFO("angle[9]:%f",pos_dy[9].data);
		ROS_INFO("angle[0]:%f",pos_dy[0].data);
		ROS_INFO("joint_plus:%f",joint_plus);



		    	//指令の送信//カメラアーム
			for(int i=0;i<9;i++)  //i=0からi=8までループ
			{
				//ROS_INFO("kitani");
				dyn_pub[i].publish(pos_dy[i]);
			}

	}
	*/
	
	
	if(b[10] == 1)
	{
		

		/////////////////////////////////////////////////////
		if(b[4] == 1)//下げる //L2ボタン
		{
			arm_y += 0.005*cos(angle1+angle2-joint_tip);  //　アームをカメラの向いているのの上下に動く
			arm_x -= 0.005*sin(angle1+angle2-joint_tip);
		}

		if(b[6] == 1)//上げる //L1ボタン
		{
			arm_y -= 0.005*cos(angle1+angle2-joint_tip);
			arm_x += 0.005*sin(angle1+angle2-joint_tip);
		}

		if(joy_axes[5] == 1)//前に //十字キー		//　アームのカメラに向いている方向に動く
		{
			arm_x += 0.005*cos(angle1+angle2-joint_tip);
			arm_y += 0.005*sin(angle1+angle2-joint_tip);
			
		}

		if(joy_axes[5] == -1)//後ろに //十字キー 		
		{
			arm_x -= 0.005*cos(angle1+angle2-joint_tip);
			arm_y -= 0.005*sin(angle1+angle2-joint_tip);
			
		}


		if(joy_axes[4] == 1)//前に
		{
			pos_dy[0].data += 0.05;
		}

		if(joy_axes[4] == -1)//後ろに
		{
			pos_dy[0].data -= 0.05;
		}



		if(b[0] == 1)//手首上げる
		{
			joint_plus += 0.05;
		}

		if(b[2] == 1)//手首下ろす
		{
			joint_plus += -0.05;
		}


		joint_tip = angle2 - PI/2 + angle1 + joint_plus;
		pos_dy[3].data = -joint_tip;

		if(b[1] == 1)//手首回転
		{
			pos_dy[4].data += 0.1;
		}

		if(b[3] == 1)//手首回転
		{
			pos_dy[4].data -= 0.1;
		}


		if(b[7] == 1)//把持
		{
			pos_dy[7].data += 0.03;
			pos_dy[8].data -= 0.03;
		}

		if(b[5] == 1)//開く
		{
			pos_dy[7].data -= 0.03;
			pos_dy[8].data += 0.03;
		}

		//ROS_INFO("angle[0]:%f",pos[0].data);
		angle1_ = acos((pow(l1,2)-pow(l2,2)+(pow(arm_x,2)+pow(arm_y,2)))/((2*l1)*sqrt(pow(arm_x,2)+pow(arm_y,2))));
		angle1 = atan2(arm_y,arm_x)-angle1_;
		angle2_ = acos((pow(l2,2)-pow(l1,2)+(pow(arm_x,2)+pow(arm_y,2)))/((2*l2)*sqrt(pow(arm_x,2)+pow(arm_y,2))));
		angle2 = angle1_ + angle2_;

		pos_dy[1].data= angle1;
		pos_dy[2].data = angle2;

		////////////////////////////////////////////////////////////



		pos_dy[5].data = -pos_dy[1].data;


			//指令の送信//カメラアーム
		for(int i=0;i<9;i++)
		{
			//ROS_INFO("kitani");
			dyn_pub[i].publish(pos_dy[i]);
		}

	}


		//test_topic.data=test_topic.data+0.1;
		//pub[0].publish(test_topic);
		
		
		
		max = sqrt(joy_axes[0]*joy_axes[0] + joy_axes[1]*joy_axes[1]) ;
		//if (x < 0 ) x = -x ; //xに絶対値を取る
		

		float pos0row=joy_axes[1]+joy_axes[0];
		float pos1row=joy_axes[1]-joy_axes[0];
		
		
		//pos[0].data = -1 * speed * pos0row * abs(pos0row);
		//pos[1].data = -1 * speed * pos1row * abs(pos1row);
		
		pos[0].data = -1 * speed * pos0row;
		pos[1].data = -1 * speed * pos1row;
/*		
		

		if (s0 == 1 ) //前進または後退		//前進または後退のみの範囲を広げる
		{
			pos[0].data = -1 * speed * joy_axes[1] * max ;//右前進（または後退)(右はマイナスが前進)
			pos[1].data = -1 * speed * joy_axes[1] * max; //左前進（または後退)
			printf("前進\n");
		}
		else if (s0 == 2)					//その場右旋回またはその場左旋回	//その場旋回のみの範囲を広げる
		{
			pos[0].data = 1 * speed * (-joy_axes[0]) * max ;//右後退(または前進)(右はマイナスが前進)
			pos[1].data = 1 * speed * joy_axes[0] * max; //左前進(または後退)
			printf("前進\n");
		}

		else if (s1 == 1)//右カーブ前進
		{
			pos[0].data = -1 * speed * (sin(atan2(joy_axes[1],joy_axes[0]))-cos(atan2(joy_axes[1],joy_axes[0]))) * max ;//右はマイナスが前進
			pos[1].data = 1 * speed * max; //左前進
			printf("右カーブ前進\n");
			printf("x:%f\n",joy_axes[0]) ;
			printf("y:%f\n",joy_axes[1]) ;

		}
		else if (s2 == 1)//左カーブ前進
		{
			pos[0].data = -1 * speed *  max ;//右前進(右はマイナスが前進)
			pos[1].data = 1 * speed * (sin(atan2(joy_axes[1],joy_axes[0]))+cos(atan2(joy_axes[1],joy_axes[0]))) * max;
			printf("左カーブ前進\n");
			printf("x:%f\n",joy_axes[0]) ;
			printf("y:%f\n",joy_axes[1]) ;
		}
		else if (s3 == 1)//右カーブ後退??
		{
			pos[0].data = 1 * speed * (sin(atan2(joy_axes[1],joy_axes[0]))-cos(atan2(joy_axes[1],joy_axes[0]))) * max ; //右はマイナスが前進
			pos[1].data = -1 * speed * (-max) ; //左後退
			printf("右カーブ後退\n");
			printf("x:%f\n",joy_axes[0]) ;
			printf("y:%f\n",joy_axes[1]) ;
		}
		else if (s4 == 1)//左カーブ後退??
		{
			pos[0].data = 1 * speed * (-max);//右後退(右はマイナスが前進)
			pos[1].data = -1 * speed * (sin(atan2(joy_axes[1],joy_axes[0]))+cos(atan2(joy_axes[1],joy_axes[0]))) * max;
			printf("右カーブ後退\n");
			printf("x:%f\n",joy_axes[0]) ;
			printf("y:%f\n",joy_axes[1]) ;
		}
		else
		{
			pos[0].data = 0;
			pos[1].data = 0;
		}

*/


		if(b[8] == 1)
		{
			if(b[6] == 1)
			{
				arm_x = 0.03;
				arm_y = 0.02;
				joint_plus = 0;
				pos_dy[0].data = 0;
			}
			
			if(b[4] == 1)
			{
				arm_x = 0.131304;
				arm_y = 0.207527;
				joint_plus = -1.450000;
				pos_dy[0].data = 0;
			}
			if(b[7] == 1)
			{
				arm_x = 0.279957;
				arm_y = 0.015715;
				joint_plus = 0;
				pos_dy[0].data = 0;
			}
			
		}

		ROS_INFO("arm_x:%f arm_y:%f joint_plus:%f pos_dy[0] %f\n",arm_x,arm_y,joint_plus,pos_dy[0]);



		dyn_pub11.publish(pos[0]);
		dyn_pub12.publish(pos[1]);








	}








int main(int argc, char **argv)//main関数
{
    ros::init(argc, argv, "hitro_control"); // callback巻数を呼び出すのに必要??
    Hitro_control Hitro_control_class;
    ros::spin();//callback関数を繰り返すのに必要
    return 0;
}
