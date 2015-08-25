#include "Aria.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
using namespace std;
#include <utility>
#include <string.h>
#include <stdlib.h>
#include <time.h> 
#include <vector>
#include <algorithm> // for std::find
#include <iterator> // for std::begin, std::end

#define PI 3.14159265
#define m_max 100
#define n_max 100
#define edge_size 500

int get_policy_matrix(int policy_mat[][n_max]);
void get_position(ArRobot *robot, double *X_current, double *Y_current);
void get_n_m (double X, double Y, int n_m[]);
void get_row_col_pol(int n_m_rel[], int row_col_pol[]);
void get_pol(int policy_mat[][n_max], int row_col_pol[], int *pol);
void get_n_m_next_policy(int pol, int next_rel[][2], int n_m_current[], int n_m_next[]);
void get_X_Y_next(int n_m_next[], double *X_next, double *Y_next);
void move_to_next(ArRobot *robot, double *X_current, double *Y_current, double X_next, double Y_next, float error);
void turn_to(ArRobot *robot, double AngleToEnd);

typedef struct position {
    int index;
    double X;
	double Y;
}position;

void save_path(std::vector<position> *pos_vector);




int main(int argc, char **argv) 
{
Aria::init();
  
ArArgumentParser argParser(&argc, argv);
argParser.loadDefaultArguments();


ArRobot robot;
ArRobotConnector con(&argParser, &robot);

if(!Aria::parseArgs())
{
Aria::logOptions();
Aria::exit(1);
return 1;
}

if(!con.connectRobot())
{
ArLog::log(ArLog::Normal, "test1: Could not connect to the robot. Exiting.");

if(argParser.checkHelpAndWarnUnparsed()) 
{
    Aria::logOptions();
}
Aria::exit(1);
return 1;
}

ArLog::log(ArLog::Normal, "test1: Connected.");

if(!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
{
Aria::logOptions();
Aria::exit(1);
}

// Run the robot processing cycle in its own thread. Note that after starting this
// thread, we must lock and unlock the ArRobot object before and after
// accessing it.
robot.runAsync(false);
robot.enableMotors();
srand (time(NULL));

//Current position
double X_current = 0;
double Y_current = 0;

//Range of n and m
int n_range[] = {1,n_max};
int m_range[] = {1,m_max};

//Goal position
double X_goal = double(rand()%(edge_size*n_max + 1));
double Y_goal = double(rand()%(edge_size*m_max + 1));
 
//tolerance
float error = 200;
float tolerance = 400;

int n_m_current[2];
int n_m_next[2];
  
int n_m_goal[2];

int n_m_rel[2];
int row_col_pol[2];

int pol;

double X_next;
double Y_next;
  
int policy_mat[m_max][n_max];
int next_rel[][2] = {{1,1},{0,1},{-1,1},{1,0},{-1,0},{1,-1},{0,-1},{-1,-1}};
get_policy_matrix(policy_mat);

double X_goals[] = {4750, 2250};
double Y_goals[] = {2750, 2750};

for (int i = 0; i<1; i++)
{
	X_goal = X_goals[i];
	Y_goal = Y_goals[i];

	get_n_m(X_goal, Y_goal, n_m_goal);
	get_X_Y_next(n_m_goal, &X_goal, &Y_goal);
	  
	printf("Goal Position: ( %lf, %lf )", X_goal, Y_goal);
	printf("\nGoal(n): %d, Goal(m): %d", n_m_goal[0], n_m_goal[1]);


	while(sqrt(pow((X_current - X_goal),2) + pow((Y_current - Y_goal),2))>tolerance)
	{
		get_position(&robot, &X_current, &Y_current);
		printf("\nCurrent Position: ( %lf, %lf )", X_current, Y_current);
	  
		get_n_m(X_current,Y_current,n_m_current);
		printf("\nCurrent(n): %d, Current(m): %d", n_m_current[0], n_m_current[1]);

		get_row_col_pol(n_m_current, row_col_pol);
	
		get_pol(policy_mat, row_col_pol, &pol);
		printf("\nPolicy: %d\n", pol);
	
		get_n_m_next_policy(pol, next_rel, n_m_current, n_m_next);
	
		get_X_Y_next(n_m_next, &X_next, &Y_next);
	
		move_to_next(&robot, &X_current, &Y_current, X_next, Y_next, error);
		//ArUtil::sleep(5000);
	}
	get_position(&robot, &X_current, &Y_current);
	printf("\nCurrent Position: ( %lf, %lf )\n", X_current, Y_current);
}
ArUtil::sleep(10000);
Aria::exit(0);
return 0;
}

int get_policy_matrix(int policy_mat[][n_max])
{
	char buffer[1024] ;
	char *record,*line;
	int i=0,j=0;
	FILE *fstream = fopen("pol_obst_6_10.csv","r");

	if(fstream == NULL)
	{
		printf("\n file opening failed ");
		return -1 ;
	}

	while((line=fgets(buffer,sizeof(buffer),fstream))!=NULL)
	{
		record = strtok(line,",");
		while(record != NULL)
		{
			//printf("record : %s",record) ;    //here you can put the record into the array as per your requirement.
			policy_mat[i][j++] = atoi(record);
			record = strtok(NULL,",");
		}
		++i;
		j = 0;
	}
	return 0;
}

void get_position(ArRobot *robot, double *X_current, double *Y_current)
{
	(*robot).lock();
	*X_current = (*robot).getX();
	*Y_current = (*robot).getY();
	(*robot).unlock();
}

void get_n_m (double X, double Y, int n_m[])
{
	if (X >= 0)
		n_m[0] = int(X/edge_size) + 1;
	else
		n_m[0] = int(X/edge_size) - 1;

	if (Y >= 0)
		n_m[1] = int(Y/edge_size) + 1;
	else
		n_m[1] = int(Y/edge_size) - 1;

}

void get_row_col_pol(int n_m[], int row_col_pol[])
{
	row_col_pol[0] = n_m[1] - 1;
	row_col_pol[1] = n_m[0] - 1;
}

void get_pol(int policy_mat[][n_max], int row_col_pol[], int *pol)
{
	*pol = policy_mat[row_col_pol[0]][row_col_pol[1]];
}

void get_n_m_next_policy(int pol, int next_rel[][2], int n_m_current[], int n_m_next[])
{
	n_m_next[0] = n_m_current[0] + next_rel[(pol-1)][0];
	n_m_next[1] = n_m_current[1] + next_rel[(pol-1)][1];
}

void get_X_Y_next(int n_m_next[], double *X_next, double *Y_next)
{	
	if(n_m_next[0] >= 0)
	  	*X_next = n_m_next[0] * edge_size - edge_size/2;
	else
	  	*X_next = n_m_next[0] * edge_size + edge_size/2;
	if(n_m_next[1] >= 0)
	  	*Y_next = n_m_next[1] * edge_size - edge_size/2;
	else
	  	*Y_next = n_m_next[1] * edge_size + edge_size/2;
}

void move_to_next(ArRobot *robot, double *X_current, double *Y_current, double X_next, double Y_next, float error)
{
	ArPose EndPosition(X_next, Y_next);
	ArPose StartPosition(*X_current, *Y_current);

	(*robot).lock();
	double AngleToEnd = (*robot).findDeltaHeadingTo(EndPosition);
	(*robot).unlock();

	turn_to(robot, AngleToEnd);

	(*robot).lock();
	double DistToEnd = (*robot).findDistanceTo(EndPosition);
	(*robot).unlock();
  
	(*robot).lock();
	(*robot).setVel(150);
	(*robot).unlock();
	ArUtil::sleep(500);

	while(1)
	{
		(*robot).lock();
		if(sqrt(pow(((*robot).getX()-X_next),2) + pow(((*robot).getY()-Y_next),2))<error)
		{
			(*robot).unlock();
			(*robot).stop();
			break;
		}
		(*robot).unlock();
	}
	//ArUtil::sleep(2000);
	get_position(robot, X_current, Y_current);
}

void turn_to(ArRobot *robot, double AngleToEnd)
{
	(*robot).setVel(0);
	ArUtil::sleep(500);
	//(*robot).lock();
	(*robot).setDeltaHeading(AngleToEnd);
	//(*robot).unlock();

	while (1)
	{
		(*robot).lock();
		if ((*robot).isHeadingDone())
		{
		  (*robot).unlock();
		  break;
		}

		(*robot).unlock();
	}
 
	//ArUtil::sleep(1000);

}

void save_path(std::vector<position> *pos_vector)
{
	ofstream fp;
	fp.open("position.txt");
	if(fp.is_open())
	{
		for(std::vector<position>::iterator it = pos_vector->begin(); it != pos_vector->end(); ++it)
		{
			fp<< it->index << "," << it->X << "," << it->Y << "\n";
		}
		fp.close();
	}
	else
		cout<< "\nUnable to open 'position'";
}