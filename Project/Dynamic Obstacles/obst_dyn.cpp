#include "Aria.h"
#include <ArSonarDevice.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include <utility>
#include <stdlib.h>
#include <time.h> 
#include <vector>
#include <algorithm> // for std::find
#include <iterator> // for std::begin, std::end

using namespace std;

#define PI 3.14159265
#define m_max 10
#define n_max 10
#define edge_size 500

int get_policy_matrix(int policy_mat[][2*(n_max-1)+1][5], int n);
void get_position(ArRobot *robot, double *X_current, double *Y_current);
void get_n_m (double X, double Y, int n_m[]);
void get_n_m_rel(int n_m_current[], int n_m_goal[] , int n_m_rel[]);
void get_row_col_pol(int n_m_rel[], int row_col_pol[], int n_range[], int m_range[]);
int get_mat_pol(ArRobot *robot, ArSonarDevice *sonar, double *range, double *angle);
void get_pol(int policy_mat[][2*(n_max-1)+1][5], int n, int row_col_pol[], int *pol);
void get_n_m_next_policy(int pol, int next_rel[][2], int n_m_current[], int n_m_next[]);
void get_X_Y_next(int n_m_next[], double *X_next, double *Y_next);
void move_to_next(ArRobot *robot, double *X_current, double *Y_current, double X_next, double Y_next, float error);
void turn_to(ArRobot *robot, double AngleToEnd);


void get_n_m_next (int n_range[], int m_range[], int n_m[], int n_m_next[][2], int *n_m_valid);
void get_coeff_next_random(int *coeff_next, int n_m_valid);
void get_coeff_next_maxValue(double values[][500], int *coeff_next, int n_m_next[][2], int n_m_valid);
void set_initial_values(double value, double values[][500], int n_range[], int m_range[]);

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

  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  int max_range = 800;
  sonar.setMaxRange(max_range);

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

  int pol_mats[2*(m_max-1)+1][2*(n_max-1)+1][5];

  int next_rel[][2] = {{0,1},{1,0},{-1,0},{0,-1}};


  double X_goals[] = {3200};
  double Y_goals[] = {3200};

  double angle;
  double range;
  int mat;


  get_policy_matrix(pol_mats, 0);
  get_policy_matrix(pol_mats, 1);
  get_policy_matrix(pol_mats, 2);
  get_policy_matrix(pol_mats, 3);
  get_policy_matrix(pol_mats, 4);

  for(int i = 0; i<1; i++)
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
		  get_n_m_rel(n_m_current, n_m_goal, n_m_rel);
		  printf("\nRelative(n): %d, Relative(m): %d", n_m_rel[0], n_m_rel[1]);
		  get_row_col_pol(n_m_rel, row_col_pol, n_range, m_range);
		  ArUtil::sleep(800);
		  mat = get_mat_pol(&robot, &sonar, &range, &angle);
          printf("\nMatrix: %d", mat);

		  get_pol(pol_mats, mat, row_col_pol, &pol);
		  printf("\nPolicy: %d\n", pol);

		  get_n_m_next_policy(pol, next_rel, n_m_current, n_m_next);
		  
		  get_X_Y_next(n_m_next, &X_next, &Y_next);
		  move_to_next(&robot, &X_current, &Y_current, X_next, Y_next, error);
		  
	  }
	  get_position(&robot, &X_current, &Y_current);
	  printf("\nCurrent Position: ( %lf, %lf )\n", X_current, Y_current);
  }

  ArUtil::sleep(10000);
  Aria::exit(0);
  return 0;
}

int get_policy_matrix(int policy_mat[][2*(n_max-1)+1][5], int n)
{
	char buffer[1024] ;
	char char_name[1024];
	char *record,*line;
	int i=0,j=0;
        
    string name = "pol_obst_";
	string num = to_string((long long)n);        
	string end = ".csv";
	
	name = name + num + end;	
 	strcpy(char_name,name.c_str());
 	
	FILE *fstream = fopen(char_name,"r");

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
			policy_mat[i][j++][n] = atoi(record);
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

void get_n_m_rel(int n_m_current[], int n_m_goal[] , int n_m_rel[])
{
	n_m_rel[0] = - n_m_goal[0] + n_m_current[0];
	n_m_rel[1] = - n_m_goal[1] + n_m_current[1];
}

void get_row_col_pol(int n_m_rel[], int row_col_pol[], int n_range[], int m_range[])
{
	row_col_pol[0] = n_m_rel[1] + m_range[1] - 1;
	row_col_pol[1] = n_m_rel[0] + n_range[1] - 1;
}


int get_mat_pol(ArRobot *robot, ArSonarDevice *sonar, double *range, double *angle)
{
	*range = (*robot).checkRangeDevicesCurrentPolar(-90, 90, angle);
	double dist = *range - robot->getRobotRadius();
	printf("\nRange = %lf", *range);
    printf("\nDist = %lf", dist);
	
	
	*angle = *angle + (*robot).getTh();
	if((*angle) <= -180)
	    (*angle) = (*angle) + 360.0;
	    else
	        if((*angle) >= 180)
	            (*angle) = (*angle) - 360.0;
	        
    printf("\nAngle = %lf", *angle);

    (*sonar).lockDevice();
	if(*range >= (*sonar).getMaxRange())
	{
	    (*sonar).unlockDevice();
		return 0;
	}
	else
	{
		(*sonar).unlockDevice();
		if(*angle >= -45 && *angle < 45)
			return 2;
		if(*angle >= 45 && *angle < 135)
			return 1;
		if((*angle >= 135 && *angle < 180) || (*angle >= -180 && *angle < -135))
			return 3;
		if(*angle >= -135 && *angle < -45)
			return 4;

	}
	
}

void get_pol(int policy_mat[][2*(n_max-1)+1][5], int n, int row_col_pol[], int *pol)
{
	*pol = policy_mat[row_col_pol[0]][row_col_pol[1]][n];
}

void get_n_m_next_policy(int pol, int next_rel[][2], int n_m_current[], int n_m_next[])
{
	n_m_next[0] = n_m_current[0] + next_rel[(pol-1)][0];
	n_m_next[1] = n_m_current[1] + next_rel[(pol-1)][1];
	
	if (n_m_next[0] == 0)
	    n_m_next[0] = n_m_next[0] + next_rel[(pol-1)][0];
	if (n_m_next[1] == 0)
	    n_m_next[1] = n_m_next[1] + next_rel[(pol-1)][1];
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

void get_coeff_next_random(int *coeff_next, int n_m_valid)
{
	*coeff_next = rand()%n_m_valid;
}

void get_coeff_next_maxValue(double values[][500], int *coeff_next, int n_m_next[][2], int n_m_valid)
{
	int i; double maxValue = -10000;
	for(i = 0; i < n_m_valid; i++)
	{
		if(values[n_m_next[i][1]][n_m_next[i][0]] > maxValue)
		{
			maxValue = values[n_m_next[i][1]][n_m_next[i][0]];
			*coeff_next = i;
		}
	}
}

void set_initial_values(double value, double values[][500], int n_range[], int m_range[])
{
	int n, m;
	for(m = 0; m < (m_range[1]); m++)
		for(n = 0; n < (n_range[1]); n++)
		{
			values[m][n] = value;
		}
}

void get_n_m_next (int n_range[], int m_range[], int n_m[], int n_m_next[][2], int *n_m_valid)
{
	int i = 0;

	if((n_m[0]-1)>=(n_range[0]))
	{
		if((n_m[1]+1)<=(m_range[1]))
		{
			n_m_next[i][0] = n_m[0]-1;
			n_m_next[i][1] = n_m[1]+1;
			i++;
		}
		
		n_m_next[i][0] = n_m[0]-1;
		n_m_next[i][1] = n_m[1];
		i++;

		if((n_m[1]-1)>=(m_range[0]))
		{
			n_m_next[i][0] = n_m[0]-1;
			n_m_next[i][1] = n_m[1]-1;
			i++;
		}
	}

	if((n_m[1]+1)<=(m_range[1]))
	{
		n_m_next[i][0] = n_m[0];
		n_m_next[i][1] = n_m[1]+1;
		i++;
		if((n_m[0]+1)<=(n_range[1]))
		{
			n_m_next[i][0] = n_m[0]+1;
			n_m_next[i][1] = n_m[1]+1;
			i++;
		}
	}

	if((n_m[1]-1)>=(m_range[0]))
	{
		n_m_next[i][0] = n_m[0];
		n_m_next[i][1] = n_m[1]-1;
		i++;
		if((n_m[0]+1)<=(n_range[1]))
		{
			n_m_next[i][0] = n_m[0]+1;
			n_m_next[i][1] = n_m[1]-1;
			i++;
		}
	}

	if((n_m[0]+1)<=(n_range[1]))
	{
		n_m_next[i][0] = n_m[0]+1;
		n_m_next[i][1] = n_m[1];
		i++;
	}

	*n_m_valid = i;
}
