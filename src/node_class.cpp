#include "ros/ros.h"
#include "../../ssl_common/include/ssl_common/config.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include "../../ssl_common/include/ssl_common/geometry.hpp"


#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
#include <deque>
#include <fstream>

const int BALL_AT_CORNER_THRESH                        = 20; 
const int HALF_FIELD_MAXX                              = 3000; 
const int HALF_FIELD_MAXY                              = 2000;
const float MAX_DRIBBLE_R                              = 3;
const int DBOX_WIDTH                                   = 600;
const int DBOX_HEIGHT                                  = 600;
const int MAX_QUEUE_SZ								   = 5;

short int isteamyellow = 0;

using namespace std;
using geometry_msgs::Pose2D;
using geometry_msgs::Point32;

ros::Publisher pub;

void print(Pose2D p){
	cout<<"x: "<<p.x<<" y: "<<p.y<<endl;
}

void print(vector<Pose2D> p){
	for(int i = 0; i < p.size(); i++)
	{
		cout<<"i: \t";
		print(p[i]);
	}
}

void print(vector<bool> p){
	for(int i=0;i<p.size();i++)
	{
		cout<<"i: \t"<<p[i]<<endl;
	}
}

class BeliefState {

	//Time
	ros::Time time;

	//Technical
	bool isteamyellow;
	int frame_number;
	double t_capture, t_sent;

	//Positions
	Pose2D ballPos;
	vector<Pose2D> awayPos, homePos;

	//Velocities
	Pose2D ballVel;
	vector<Pose2D> awayVel, homeVel;

	//Detections
	bool ballDetected;
	vector<bool> awayDetected, homeDetected;

	//Predictions
	short int 	our_bot_closest_to_ball,
				opp_bot_closest_to_ball,
				our_goalie,
				opp_goalie,
				opp_bot_marking_our_attacker;

	bool ball_at_corners, ball_in_our_half, ball_in_our_possession;

	BeliefState();
	BeliefState(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& , list<BeliefState> &);
	BeliefState(BeliefState &);
	void initialise();
	void print();

};

bool isValidMsg(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg){
	if(vmsg->robots_yellow.size() == 0 && vmsg->robots_blue.size()==0)
		return false;
	if(vmsg->balls.size() == 0)
		return false;
	return true;
}

BeliefState::BeliefState(){
	this->initialise();
}

BeliefState::BeliefState(BeliefState &msg){
	this->isteamyellow = msg.isteamyellow;
	this->frame_number = msg.frame_number;
	this->t_capture = msg.t_capture;
	this->t_sent = msg.t_sent;
	this->ballPos = msg.ballPos;
	this->awayPos = msg.awayPos;
	this->homePos = msg.homePos;
	this->ballVel = msg.ballVel;
	this->awayVel = msg.awayVel;
	this->homeVel = msg.homeVel;
	this->ballDetected = msg.ballDetected;
	this->homeDetected = msg.homeDetected;
	this->awayDetected = msg.awayDetected;
	this->our_bot_closest_to_ball = msg.our_bot_closest_to_ball;
	this->opp_bot_closest_to_ball = msg.opp_bot_closest_to_ball;
	this->our_goalie = msg.our_goalie;
	this->opp_goalie = msg.opp_goalie;
	this->opp_bot_marking_our_attacker = msg.opp_bot_marking_our_attacker;
	this->ball_at_corners = msg.ball_at_corners;
	this->ball_in_our_half = msg.ball_in_our_half;
	this->ball_in_our_possession = msg.ball_in_our_possession;
}

float distFn(krssg_ssl_msgs::SSL_DetectionRobot p, float x, float y){
	return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
}

BeliefState::BeliefState(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg, list<BeliefState> &prev_msgQ){
	if(!isValidMsg(vmsg)){
		if(prev_msgQ.size())
			BeliefState(prev_msgQ.back());
		else{
			this->initialise();
		}
	}
	else{
		if(prev_msgQ.size())
			this(prev_msgQ.back());
		else this->initialise();
		this->time = ros::Time::now();
		this->frame_number = vmsg->frame_number;
		this->t_capture = vmsg->t_capture;
		this->t_sent = vmsg->t_sent;
		this->isteamyellow = ::isteamyellow;

		vector<SSL_DetectionRobot> homePos, awayPos;
		if(::isteamyellow){
			homePos = vmsg->robots_yellow;
			awayPos = vmsg->robots_blue;
		}
		else{
			homePos = vmsg->robots_blue;
			awayPos = vmsg->robots_yellow;
		}


		if(vmsg.balls.size()){
			this->ballDetected = 1;
			this->ballPos = vmsg->balls[0];


			double FILTER[MAX_QUEUE_SZ] = {0.086, -0.143, -0.086, 0.257, 0.886};
			//FILTERING BALLPOS
			if(prev_msgQ.size() == MAX_QUEUE_SZ)
			{
				this->ballPos.x *= 0.886;
				this->ballPos.y *= 0.886;
				int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()){
						this->ballPos.x += it->ballPos.x*FILTER[count];
						this->ballPos.y += it->ballPos.y*FILTER[count];
					}
					count ++;
				}
			}

			if(this->ballPos.x <= 0)
				this->ball_in_our_half = true;
			else this->ball_in_our_half = false;
			if(	fabs(this->ballPos.x) > (HALF_FIELD_MAXX - BALL_AT_CORNER_THRESH) && 
				fabs(this->ballPos.y) > (HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH))
				this->ball_at_corners = true;
			else this->ball_at_corners = false;

		}
		else this->ballDetected = 0;


		float distance_from_ball = 99999999, tempGoalie=7, tempDist=9999999;
		for(int i=0;i<homePos.size();i++)
		{
			int bot_id = homePos[i].robot_id;
    		this->homeDetected[bot_id] = 1;
    		this->homePos[bot_id].x = homePos[i].x;
    		this->homePos[bot_id].y = homePos[i].y;
    		this->homePos[bot_id].theta = homePos[i].orientation;

    		//HOMEPOS FILTERING
    		if(prev_msgQ.size() == MAX_QUEUE_SZ)
			{
				this->homePos[i].x *= 0.886;
				this->homePos[i].y *= 0.886;
				int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()){
						this->homePos[i].x += it->homePos[i].x*FILTER[count];
						this->homePos[i].y += it->homePos[i].y*FILTER[count];
					}
					count ++;
				}
			}

    		float dist = sqrt(pow((homePos[i].x - this->ballPos.x),2) + pow((homePos[i].y - this->ballPos.y) , 2));
   			if(dist < distance_from_ball){
      			distance_from_ball = dist;
     	 		this->our_bot_closest_to_ball = i;
	      		if(distance_from_ball < MAX_DRIBBLE_R){
	        		this->ball_in_our_possession = true;
	      		}
    		}

    		
    		float thisDist = 	distFn(homePos[i],(-HALF_FIELD_MAXX + DBOX_WIDTH),DBOX_HEIGHT) +
    							distFn(homePos[i],(-HALF_FIELD_MAXX + DBOX_WIDTH),-DBOX_HEIGHT);
    		if(thisDist	< tempDist){
    			tempDist = thisDist;
    			tempGoalie = bot_id;
    		}
    	}
    	
    	this->our_goalie = tempGoalie;

    	tempGoalie = 7;
    	distance_from_ball = 9999999;
    	tempDist = 9999999;

    	for(int i=0;i<awayPos.size();i++){
    		int bot_id = awayPos[i].robot_id;
    		this->awayDetected[bot_id] = 1;
    		this->awayPos[bot_id].x = awayPos[i].x;
    		this->awayPos[bot_id].y = awayPos[i].y;
    		this->awayPos[bot_id].theta = awayPos[i].orientation;

    		//AWAYPOS FILTERING
    		if(prev_msgQ.size() == MAX_QUEUE_SZ)
			{
				this->awayPos[i].x *= 0.886
				this->awayPos[i].y *= 0.886
				int count = -1;
				for(list<BeliefState>::iterator it = prev_msgQ.begin(); it != prev_msgQ.end(); it++){
					if(it != prev_msgQ.begin()){
						this->awayPos[i].x += it->awayPos[i].x*FILTER[count];
						this->awayPos[i].y += it->awayPos[i].y*FILTER[count];
					}
					count ++;
				}
			}

    		dist = sqrt(((awayPos[i].x - this->ballPos.x)*(awayPos[i].x - this->ballPos.x)) + ((awayPos[i].y - this->ballPos.y)*(awayPos[i].y - this->ballPos.y)));

    		if(dist < distance_from_ball){
      			distance_from_ball = dist;
      			this->opp_bot_closest_to_ball = bot_id;
   			}

   			float thisDist = 	distFn(awayPos[i],(HALF_FIELD_MAXX - DBOX_WIDTH),DBOX_HEIGHT) +
    							distFn(awayPos[i],(HALF_FIELD_MAXX - DBOX_WIDTH),-DBOX_HEIGHT);
    		if(thisDist	< tempDist){
    			tempDist = thisDist;
    			tempGoalie = bot_id;
    		}
    	}
    	this->opp_goalie = tempGoalie;

    	//VELOCITY CALCULATIONS

    	BeliefState prevState;
    	if(prev_msgQ.size())
    		prevState = prev_msgQ.front();

    	assert(prevState.homePos.size() == 6);
    	assert(prevState.awayPos.size() == 6);

    	this->ballVel.x = (this->ballPos.x - prevState.ballPos.x)/(this->time - prevState.time);
    	this->ballVel.y = (this->ballPos.y - prevState.ballPos.y)/(this->time - prevState.time);

    	for(int i=0;i<this->homePos.size();i++)
    	{
    		this->homeVel[i].x = (this->homePos[i].x - prevState.homePos[i].x)/(this->time - prevState.time);
    		this->homeVel[i].y = (this->homePos[i].y - prevState.homePos[i].y)/(this->time - prevState.time);
    	}

		for(int i=0;i<this->awayPos.size();i++)
    	{
    		this->awayVel[i].x = (this->awayPos[i].x - prevState.awayPos[i].x)/(this->time - prevState.time);
    		this->awayVel[i].y = (this->awayPos[i].y - prevState.awayPos[i].y)/(this->time - prevState.time);
    	}

    	//UPDATING LIST
		if(prev_msgQ.size() == MAX_QUEUE_SZ){
			prev_msgQ.pop_front();
			prev_msgQ.push_back(*this);
			assert(prev_msgQ.size()==MAX_QUEUE_SZ);
		}
		else{
			prev_msgQ.push_back(*this);
		}
	}
}

void BeliefState::initialise(){

	this->time = ros::Time::now();
	this->isteamyellow = ::isteamyellow;
	this->frame_number = 0;
	this->t_capture = 0.0;
	this->t_sent = 0.0;

	this->ballPos = Pose2D();
	this->awayPos = vector<Pose2D>(6,Pose2D());
	this->homePos = vector<Pose2D>(6,Pose2D());

	this->ballVel = Pose2D();
	this->awayVel = vector<Pose2D>(6,Pose2D());
	this->homeVel = vector<Pose2D>(6,Pose2D());

	this->ballDetected = false;
	this->homeDetected = vector<bool>(6,0);
	this->awayDetected = vector<bool>(6,0);

	this->our_bot_closest_to_ball = this->opp_bot_closest_to_ball = 
	this->our_goalie = this->opp_goalie = this->opp_bot_marking_our_attacker = 0;

	this->ball_at_corners = this->ball_in_our_possession = this->ball_in_our_half = false;
}

void BeliefState::print(){
	system("clear");
	cout<<"time: "<<this->time<<endl;
	cout<<"isteamyellow: "<<isteamyellow<<endl;
	cout<<"frame_number: "<<frame_number<<endl;
	cout<<"t_capture: "<<t_capture<<" t_sent: "<<t_sent<<endl;
	cout<<"ballPos:"<<endl;
	print(ballPos);
	cout<<"awayPos:"<<endl;
	print(awayPos);
	cout<<"homePos:"<<endl;
	print(homePos);
	cout<<"ballVel:"<<endl;
	print(ballVel);
	cout<<"awayVel:"<<endl;
	print(awayVel);
	cout<<"homeVel:"<<endl;
	print(homeVel);
	cout<<"ballDetected: "<<ballDetected;
	cout<<"homeDetected:"<<endl;
	print(homeDetected);
	cout<<"awayDetected:"<<endl;
	print(awayDetected);
	cout<<"our_bot_closest_to_ball: "<<our_bot_closest_to_ball<<
	" opp_bot_closest_to_ball: "<<opp_bot_closest_to_ball<<
	" \n our_goalie: "<<our_goalie<<" opp_goalie: "<<opp_goalie<<
	" opp_bot_marking_our_attacker: "<<opp_bot_marking_our_attacker<<
	"\n ball_at_corners: "<<ball_at_corners<<" ball_in_our_half: "<<ball_in_our_half<<
	"\n ball_in_our_possession: "<<ball_in_our_possession<<endl;
}

krssg_ssl_msgs::BeliefState BeliefState::getBeliefStateMsg(){
	krssg_ssl_msgs::BeliefState msg;
	msg.isteamyellow = this->isteamyellow;
	msg.frame_number = this->frame_number;
	msg.t_capture = this->t_capture;
	msg.t_sent = this->t_sent;
	msg.ballPos = this->ballPos;
	msg.awayPos = this->awayPos;
	msg.homePos = this->homePos;
	msg.ballVel = this->ballVel;
	msg.awayVel = this->awayVel;
	msg.homeVel = this->homeVel;
	msg.ballDetected = this->homeDetected;
	msg.homeDetected = this->homeDetected;
	msg.awayDetected = this.awayDetected;
	msg.our_bot_closest_to_ball = this->our_bot_closest_to_ball;
	msg.opp_bot_closest_to_ball = this->opp_bot_closest_to_ball;
	msg.our_goalie = this->our_goalie;
	msg.opp_goalie = this->opp_goalie;
	msg.opp_bot_marking_our_attacker = this->opp_bot_marking_our_attacker;
	msg.ball_at_corners = this->ball_at_corners;
	msg.ball_in_our_half = this->ball_in_our_half;
	msg.ball_in_our_possession = this->ball_in_our_possession;
	return msg;
}


void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg){
	static list<BeliefState> prev_msgQ;
	BeliefState bs(vmsg,prev_msgQ);
	bs.print();
	pub.publish(bs.getBeliefStateMsg());
}


int main(int argc, char const *argv[])
{
	ros::init(argc,argv,"beliefstate_node");

	if(argc > 1){
		::isteamyellow = atof(argv[1]);
	}

	ros::NodeHandle n;
	ros::Subscriber vision_sub = n.subscribe("/vision", 10000, Callback);
	::pub = n.advertise<krssg_ssl_msgs::BeliefState>("/belief_state", 10000);
	ros::spin();
	return 0;
}