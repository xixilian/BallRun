/************************************************************************

dynamics.c: This is where the numerical integration and ODE stuff is done.

/************************************************************************/
/*
Dropping ball program

I assume some version of Open Dynamics Engine (ODE) is installed.

Changed drawstuff so this is negative and coordinate system has
X positive:
#define LIGHTY (-0.4f)

Note: 
x points to viewer's right.
z is up.
y points away from viewer, so angles are negative.

/************************************************************************/

#include <stdio.h>
#include <math.h>
#include <ode/ode.h>
#include <iostream>
#include <fstream>
#include <algorithm>



#ifdef WIN32
#include <float.h>
#define isnan _isnan
#define isinf !_finite
#endif

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

//#include "./main.h"

/************************************************************************/

/******************************************************************/
/* DEFINES */

/* These are handy constants. XX rather than X to avoid math.h conflicts */
#define XX 0
#define YY 1
#define ZZ 2

/* Status flags from sim.status */
#define OK 0
#define DONE 1

/* This is used to allocate parameter vectors etc. */
#define MAX_N_PARAMETERS 100

#define TRUE 1
#define FALSE 0

#define TIME_STEP 0.001
#define WAIT_TO_START 0.1

// These should be set up in configuration file
#define BALL_DIAMETER 0.017270193333 // meters
#define WALL_WIDTH 0.55 // meters
#define WALL_THICKNESS 0.01 // meters
#define WALL_HEIGHT 0.55 // meters
#define FUDGE1 (BALL_DIAMETER/10) // space between ball and wall

// used to allocate space for obstacles
#define MAX_N_GEOMS 100

// object types
#define BALL 0
#define OBSTACLE 1
#define WALL 2
#define UNKNOWN -1

#define GOAL_PIECES 3

#define BALLPOS 0.52





/****************************************************************/
// Typedefs

typedef struct rect
{
  double dimensions[3];
  double pos[3];
  double angle;
}
  RECT;

/******************************************************************/

typedef struct setup
{
  // these are static ODE objects: just geometries

  dGeomID the_wall;

  RECT obstacles[MAX_N_GEOMS];
  dGeomID the_obstacles[MAX_N_GEOMS];
  int n_obstacles;
}
  SETUP;

/****************************************************************/

/* For each parameter read in from a file we have this information */
typedef struct parameter
{
  char *name;
  double value;
  int optimize;
  int regularize;
  double nominal_value;
  double regularize_weight;
  struct parameter *next;
  double *pointer;
} PARAMETER;

/******************************************************************/


// for the goal
typedef struct goal
{

  RECT pieces[MAX_N_GEOMS];
  // geometries already exist, maybe I don't need it
  dGeomID the_pieces[MAX_N_GEOMS];
  // maybe just left and right bounderies
  RECT laying_piece;
  RECT left_piece;
  RECT right_piece;
  float left_bound;
  float right_bound;
  float height;
  int n_pieces;
  float total_height;
}
  GOAL;
/******************************************************************/
// parameters for simulation
typedef struct {

  int status;

  double time_step;          // simulation physics time step    
  double time;               // current time in simulation

  double duration;  // simulation duration

  // used in a state machine in controller.c
  double state_start_time;
  double state_elapsed_time;

  // random stuff
  int rand_seed;
  double rand_scale;

  // Optimization variables
  double o0_x;
  double o0_z;
  double o0_angle;

  double o1_x;
  double o1_z;
  double o1_angle;

  double o2_x;
  double o2_z;
  double o2_angle;

  /* Objective function */
  double final_x;

  /* Useful variables for optimization */
  int n_parameters;
  double all_time_low_cost;
  double debug_criterion;
  int func_calls;
  int n_func_calls_per_eval;
  PARAMETER *params;
  char output_file[10000];
  int iter;

  SETUP setup;

  GOAL goal;

  // ODE dynamics and collision objects
  dWorldID world;
  dSpaceID space;
  dJointGroupID contactgroup;

  // the ODE ball
  dBodyID ball_body;
  dGeomID ball_geom;
}
  SIM;

/******************************************************************/
static SIM sim;

static int simulation_running = 1;
static int counter = 0;

static std::ofstream track;
      	



/************************************************************************/
/****************************************************************/
// this program figures out what kind of collision it is.

void identify_contact( dGeomID o, int *type, const char **str, SIM *s )
{
  int i;
  SETUP *h;

  h = &(s->setup);

  if ( o == s->ball_geom )
    {
      *type = BALL;
      *str = "ball";
      return;
    }
  if ( o == h->the_wall )
    {
      *type = WALL;
      *str = "wall";
      return;
    }
  for ( i = 0; i < h->n_obstacles; i++ )
    if ( o == h->the_obstacles[i] )
      {
    	*type = OBSTACLE;
    	*str = "obstacle";
    	return;
      }
  *type = UNKNOWN;
  *str = "unknown";
}

/****************************************************************/

/*
When the collision system detects that two objects are colliding, it
calls this routine which determines the points of contact and creates
temporary joints. The surface parameters of the joint (friction,
bounce velocity, CFM, etc) are also set here.
*/
  // this is called by dSpaceCollide when two objects in space are
  // potentially colliding.
static void nearCallback( void *data, dGeomID o1, dGeomID o2 )
{
  dBodyID b1 = dGeomGetBody( o1 );
  dBodyID b2 = dGeomGetBody( o2 );
  dContact contact;
  int o1_type, o2_type, c_type;
  const char *o1_string, *o2_string, *c_string;

  // Need to identify contact:
  identify_contact( o1, &o1_type, &o1_string, &sim );
  identify_contact( o2, &o2_type, &o2_string, &sim );
  // printf( "%s %s\n", o1_string, o2_string );
  // give up if unknown object involved
  if ( o1_type == UNKNOWN )
    return;
  if ( o2_type == UNKNOWN )
    return;
  if ( o1_type == BALL )
    {
      c_type = o2_type;
      c_string = o2_string;
    }
  else if ( o2_type == BALL )
    {
      c_type = o1_type;
      c_string = o1_string;
    }
  else 
    return; // no ball involved, give up.

  if ( c_type == WALL )
    {
      contact.surface.mode = dContactBounce | dContactSoftCFM;
      // friction parameter
      contact.surface.mu = dInfinity;
      // bounce is the amount of "bouncyness".
      contact.surface.bounce = 0.3;
      // bounce_vel is the minimum incoming velocity to cause a bounce
      contact.surface.bounce_vel = 1e10;
      // constraint force mixing parameter
      contact.surface.soft_cfm = 0.01;  // 0.001 in bounce
    }
  else if ( c_type == OBSTACLE )
    {
      contact.surface.mode = dContactBounce | dContactSoftCFM;
      // friction parameter
      contact.surface.mu = 3.0;
      // bounce is the amount of "bouncyness".
      contact.surface.bounce = 0.38; // used to be 0.1
      // bounce_vel is the minimum incoming velocity to cause a bounce
      contact.surface.bounce_vel = 0.01;
      // constraint force mixing parameter
      contact.surface.soft_cfm = 0.01;  // 0.001 in bounce
    }
  else
    return; // no obstacle or wall involved, give up

  if (dCollide (o1,o2,1,&contact.geom,sizeof(dContact)))
    {
      dJointID c = dJointCreateContact ( sim.world, sim.contactgroup,
					 &contact );
      dJointAttach( c, b1, b2 );
    }
}

/****************************************************************/
/****************************************************************/

int read_setup_file( const char *filename, SETUP *h, GOAL *g )
{
  int i;
  FILE *stream;
  char buffer[1000];

  stream = fopen( filename, "r" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Can't open setup file %s", filename );
      exit( -1 );
    }

  h->n_obstacles = 0;

  for( ; ; )
    {
      // read keyword
      if ( fscanf( stream, "%s", buffer ) < 1 )
	break; // if we didn't read anything we are done

      // handle an obstacle
      if ( strcmp( buffer, "o" ) == 0 )
	{
	  i = h->n_obstacles;
	  if ( fscanf( stream, "%lg%lg%lg%lg%lg%lg%lg",
		       &(h->obstacles[i].dimensions[XX]),
		       &(h->obstacles[i].dimensions[YY]),
		       &(h->obstacles[i].dimensions[ZZ]),
		       &(h->obstacles[i].pos[XX]),
		       &(h->obstacles[i].pos[YY]),
		       &(h->obstacles[i].pos[ZZ]),
		       &(h->obstacles[i].angle) ) < 7 )
	    {
	      fprintf( stderr, "bad obstacle in setup file %s\n", filename );
	      exit( -1 );
	    }
	  printf( "rsf: obstacle: %g %g %g %g %g %g %g %0lx\n",
		  h->obstacles[i].dimensions[XX],
		  h->obstacles[i].dimensions[YY],
		  h->obstacles[i].dimensions[ZZ],
		  h->obstacles[i].pos[XX],
		  h->obstacles[i].pos[YY],
		  h->obstacles[i].pos[ZZ],
		  h->obstacles[i].angle,
		  (unsigned long) h );
	  h->n_obstacles++;
	  continue;
	}
      
      fprintf( stderr, "bad keyword %s in setup file %s\n", buffer, filename );
      exit( -1 );
    }

  fclose( stream );

    g->n_pieces = GOAL_PIECES;
  // the last 3 pieces of obstacles
  //for(i = current_setup.n_obstacles - 1 ; i > 2; i -- )
  //{
  //  j = g->n_pieces;
  //  g-> pieces[j]
  //}
  int total_pieces = h ->n_obstacles;
  int o_pieces = total_pieces -3;

  // copy array 
  std::copy(h -> obstacles+ o_pieces, h -> obstacles+total_pieces,g->pieces);


	float max_z_len = 0.0;
	
	float max_x_len = 0.0;
	int max_x_in ;
	int max_z_in ;
	for (i = 0; i < g->n_pieces;i++){
		if (max_x_len <= g->pieces[i].dimensions[XX])
		{
			max_x_len = g->pieces[i].dimensions[XX];
			max_x_in = i;	
		}

		if (max_z_len < g->pieces[i].dimensions[ZZ])
		{
			max_z_len = g->pieces[i].dimensions[ZZ];
			max_z_in = i;	
		}
		
	}
	g->height = g->pieces[max_x_in].dimensions[ZZ];
	g->laying_piece = g->pieces[max_x_in];

  	g->total_height = max_z_len + g->height;



  //printf( "laying_piece x: %g\n", g->laying_piece.pos[XX]);

  for (i = 0; i < g->n_pieces;i++)
  {
    printf("piece x index %i\n" , i );
    printf("piece x pos %g\n" , g->pieces[i].pos[XX] );

    if(i != max_x_in)
    {
      // left piece x position is smaller than the laying one
      

      if(g->pieces[i].pos[XX] < g->laying_piece.pos[XX])
      {
        g->left_piece = g->pieces[i];
      }
      else
      { 
        g->right_piece = g->pieces[i];
      } 
    }   
  }

  g->left_bound = g->left_piece.pos[XX] + g->left_piece.dimensions[XX]/2.0;
  g->right_bound = g->right_piece.pos[XX] - g->right_piece.dimensions[XX]/2.0;

}

/****************************************************************/

void create_bodies( const char *filename, SIM *s )
{
  int i;
  SETUP *h;
  dMass m;
  dQuaternion q;
  GOAL *g;
  
  h = &(s->setup);
  g = &(s->goal);

  read_setup_file( filename, h, g );

  // create ball object
  s->ball_body = dBodyCreate( s->world );
  s->ball_geom = dCreateSphere( s->space, BALL_DIAMETER/2 );
  dMassSetSphere( &m, 0.6, BALL_DIAMETER/2 );
  dBodySetMass( s->ball_body, &m );
  dGeomSetBody( s->ball_geom, s->ball_body );
  dBodySetLinearDamping( s->ball_body, 1e-4 ); 
  dBodySetLinearDampingThreshold( s->ball_body, 1e-7 );
  // set initial position and velocity
  dBodySetPosition( s->ball_body, 0.0, 0.0, BALLPOS);
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
  dBodySetQuaternion ( s->ball_body, q );
  dBodySetLinearVel( s->ball_body, 0.1, 0.0, 0.0 );
  dBodySetAngularVel( s->ball_body, 0.0, 0.0, 0.0 );

  /*
    FROM FAQ:

  How can an immovable body be created?

  In other words, how can you create a body that doesn't move, but
  that interacts with other bodies? The answer is to create a geom
  only, without the corresponding rigid body object. The geom is
  associated with a rigid body ID of zero. Then in the contact
  callback when you detect a collision between two geoms with a
  nonzero body ID and a zero body ID, you can simply pass those two
  IDs to the dJointAttach function as normal. This will create a
  contact between the rigid body and the static environment.
  */

  h->the_wall = dCreateBox( s->space,
			    WALL_WIDTH, WALL_THICKNESS, WALL_HEIGHT );
  dGeomSetPosition( h->the_wall, WALL_WIDTH/2 - BALL_DIAMETER,
		    BALL_DIAMETER + FUDGE1, WALL_HEIGHT/2 );

  for ( i = 0; i < h->n_obstacles; i++ )
    {
      h->the_obstacles[i] =
	dCreateBox( s->space, h->obstacles[i].dimensions[XX],
		    h->obstacles[i].dimensions[YY],
		    h->obstacles[i].dimensions[ZZ] );
      dGeomSetPosition( h->the_obstacles[i], h->obstacles[i].pos[XX],
		    h->obstacles[i].pos[YY],
		    h->obstacles[i].pos[ZZ] );
      q[0] = cos( h->obstacles[i].angle/2 );
      q[1] = 0.0;
      q[2] = sin( h->obstacles[i].angle/2 );
      q[3] = 0.0;
      dGeomSetQuaternion ( h->the_obstacles[i], q );
      printf( "cb: obstacle: %g %g %g %g %g %g %g %0lx\n",
	      h->obstacles[i].dimensions[XX],
	      h->obstacles[i].dimensions[YY],
	      h->obstacles[i].dimensions[ZZ],
	      h->obstacles[i].pos[XX],
	      h->obstacles[i].pos[YY],
	      h->obstacles[i].pos[ZZ],
	      h->obstacles[i].angle,
	      (unsigned long) h );
    }
}

/*****************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/* Initialize the state vector */

void init_dynamics_state( SIM *s, double *state )
{
}

/************************************************************************/
/************************************************************************/
/* Initialize dynamics package: ODE, sdfast, ...  */

void init_dynamics( SIM *s )
{
}

/*****************************************************************************/
/* call this many times to restart a simulation */

int reinit_sim( SIM *s )
{
  int i;
  dQuaternion q;
  SETUP *h;
  dReal x, y, z;

  s->time = 0;
  s->final_x = 0;

  dBodySetPosition( s->ball_body, 0.0, 0.0, BALLPOS);

  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
  dBodySetQuaternion ( s->ball_body, q );
  dBodySetLinearVel( s->ball_body, 0.1, 0.0, 0.0 );
  dBodySetAngularVel( s->ball_body, 0.0, 0.0, 0.0 );
  
  /*
  const dReal *v = dBodyGetLinearVel( ball_body );
  printf( "velocity: %lg %lg %lg\n", v[0], v[1], v[2] );
  */

  h = &(s->setup);

  //apply_parameters( h, s );

  for ( i = 0; i < h->n_obstacles; i++ )
    {
      /*
      printf( "rs: obstacle: %g %g %g %g %g %g %g %0lx\n",
	      h->obstacles[i].dimensions[XX],
	      h->obstacles[i].dimensions[YY],
	      h->obstacles[i].dimensions[ZZ],
	      h->obstacles[i].pos[XX],
	      h->obstacles[i].pos[YY],
	      h->obstacles[i].pos[ZZ],
	      h->obstacles[i].angle,
	      (unsigned long) h );
      printf( "32 %d\n", i );
      */
      printf( "o%d: %lg %lg %lg %lg\n", i, h->obstacles[i].pos[XX],
	      h->obstacles[i].pos[YY], h->obstacles[i].pos[ZZ],
	      h->obstacles[i].angle );
      // printf( "33 %d\n", i );
      dGeomSetPosition( h->the_obstacles[i], h->obstacles[i].pos[XX],
		    h->obstacles[i].pos[YY],
		    h->obstacles[i].pos[ZZ] );
      // printf( "34 %d\n", i );
      q[0] = cos( h->obstacles[i].angle/2 );
      q[1] = 0.0;
      q[2] = sin( h->obstacles[i].angle/2 );
      q[3] = 0.0;
      // printf( "35 %d\n", i );
      dGeomSetQuaternion ( h->the_obstacles[i], q );
      // printf( "36 %d\n", i );
    }
  s->status = OK;

  return 0;
}

/*****************************************************************************/
/* Call this once to do one time operations like memory allocation */

int init_sim( SIM *s , const char *filename)
{
  dReal erp, cfm;

  dInitODE();
  // create world
  s->world = dWorldCreate( );
  s->space = dHashSpaceCreate( 0 );
  dWorldSetGravity( s->world, 0.0, 0.0, -9.81 );
  dWorldSetCFM( s->world, 1e-5 );
  // dWorldSetERP (dWorldID, dReal erp);
  erp = dWorldGetERP( s->world );
  cfm = dWorldGetCFM( s->world );
  /*
  printf( "erp: %g, cfm: %g, kp: %g, kd: %g\n",
	  erp, cfm, erp/(cfm*TIME_STEP), (1 - erp)/cfm );
  */

  s->contactgroup = dJointGroupCreate( 0 );

  //create_bodies( "./params/dyn_setup.txt", s );
  create_bodies( filename, s );

  init_dynamics( s );

  reinit_sim( s );

  return 0;
}

/************************************************************************/
/* This is what is called on each integration step */

void integrate_one_time_step( SIM *s )
{ 
  const dReal *pos;

  static int printed_result = 0;

  // find collisions and add contact joints
  dSpaceCollide( s->space, 0, &nearCallback );

  // step the simulation
  dWorldStep( s->world, TIME_STEP );  

  s->time += TIME_STEP;

  // remove all contact joints
  dJointGroupEmpty( s->contactgroup );

  /*
  pos = dBodyGetPosition( s->ball_body );
  if ( pos[ZZ] < BALL_DIAMETER/2 )
    {
      s->final_x = pos[XX];
      s->status = DONE;
      printf ( "DONE: %lg %lg %lg\n", s->time, pos[XX], pos[ZZ] );
       if ( !printed_result )
       {
        // only the ball on the ground it will be printed...
        std::ofstream file;
        file.open("./results/result.txt");
        file << pos[XX] << " ";
        file << pos[ZZ] << "\n";
        file.close();
        printed_result = 1;
        exit(0);
      }
    }

    if  
*/
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/

/********************************/
// do not need to draw ODE 
// simulation loop
static void simLoop (int pause, SIM *s)
{
  const dReal *pos;
  const dReal *pos2;
  const dReal *vel;
  static dReal z_vel_1 = 5.0;
  static dReal x_vel_1 = 5.0; 
  static dReal z_pos_1 = 5.0;
  static dReal x_pos_1 = 0.0;

  static dReal z_vel = 5.0;
  static dReal x_vel = 5.0; 
  static dReal z_pos = 5.0;
  static dReal x_pos = 0.0;
  static int printed_result = 0;
  dQuaternion q;
  static int write_vel_1 = 0;
  static int write_vel_2 = 0;
  static int count = 0;

  // do user defined stuff
 // do_user_stuff( &current_setup );

  if ( simulation_running )
    {
      integrate_one_time_step(s);
    }

  // hold ball at top
  
    pos = dGeomGetPosition( s->ball_geom );
    pos2 = dBodyGetPosition( s->ball_body );

	vel = dBodyGetLinearVel (s->ball_body);

	
	//track << pos[XX] << " ";
      	//track << pos[ZZ] << std::endl;

	// slightly above
	if ((pos[ZZ] - s->goal.total_height ) <= (BALL_DIAMETER/2.0) && (pos[ZZ] - s->goal.total_height ) > 0 )
	{
		//printf ( " %lg %lg %lg \n", vel[ZZ], pos[XX], pos[ZZ]);
		//printf("hit");
		//printf ( "%lg %lg\n",pos[ZZ], goal.total_height );
		// std::ofstream file;
      		//file.open("./results/vels.txt");
      		//file << vel[ZZ] << " ";
		//file << an_vel[ZZ] << " ";
		//file << pos[XX] << " ";
      		//file << pos[ZZ] << "\n";
      		//file.close();
		//printf("hit");

		// ensure the first time
		if (!write_vel_1){
			x_vel_1 = vel[XX];
			z_vel_1 = vel[ZZ];
			z_pos_1 = pos[ZZ];
			x_pos_1 = pos[XX];
			write_vel_1 = 1 ;
		}
		
                	
		
	}


	if (vel[ZZ] <= 0.01 && vel[ZZ] >= 0 && pos[XX] > s->goal.left_bound && pos[XX] < s->goal.right_bound  
	&& (pos[ZZ] + (BALL_DIAMETER/2.0) - s->goal.total_height ) > 0) 
	{
		printf ( " %lg %lg %lg \n", vel[ZZ], pos[ZZ],  s->goal.total_height);
		count +=1;
	}

	//printf ( "%lg %lg\n",pos[ZZ], s->goal.total_height );
	// slightly under
	if ((s->goal.total_height - pos[ZZ]) <= (BALL_DIAMETER/2.0) && (s->goal.total_height - pos[ZZ]) > 0 )
	{
		//printf ( " %lg %lg %lg \n", vel[ZZ], pos[XX], pos[ZZ]);
		//printf("hit");
		//printf ( "%lg %lg\n",pos[ZZ], goal.total_height );
		// std::ofstream file;
      		//file.open("./results/vels.txt");
      		//file << vel[ZZ] << " ";
		//file << an_vel[ZZ] << " ";
		//file << pos[XX] << " ";
      		//file << pos[ZZ] << "\n";
      		//file.close();
		//printf("hit");

		// ensure the first time
		if (!write_vel_2){
			x_vel = vel[XX];
			z_vel = vel[ZZ];
			z_pos = pos[ZZ];
			x_pos = pos[XX];
			write_vel_2 = 1 ;
		}
		
                	
		
	}


    // printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
      if ( pos[ZZ] < BALL_DIAMETER/2 )
        {
          simulation_running = FALSE;
          s->final_x = pos[XX];
          s->status = DONE;

          dGeomSetPosition( s->ball_geom, pos[XX], pos[YY], BALL_DIAMETER/2 - 0.0001 );
        dBodySetPosition( s->ball_body, pos2[XX], pos2[YY], BALL_DIAMETER/2 - 0.0001 );
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
        dBodySetQuaternion ( s->ball_body, q );
        dBodySetLinearVel( s->ball_body, 0.0, 0.0, 0.0 );
        dBodySetAngularVel( s->ball_body, 0.0, 0.0, 0.0 );
        if ( !printed_result )
       {
      // only the ball on the ground it will be printed...
      std::ofstream file;
      std::ofstream f2;
      file.open("./results/result.txt");
      f2.open("./results/rstring.txt");
      file << pos[XX] << " ";
      file << pos[ZZ] << " ";
      file << x_vel_1 << " ";
      file << z_vel_1 << " ";
      file << z_pos_1 << " ";
      file << x_pos_1 << " ";
      file << x_vel << " ";
      file << z_vel << " ";
      file << z_pos << " ";
      file << x_pos << " ";
      file << count << "\n";
      file.close();

      printf ( "%lg %lg %lg\n", s->time, pos[XX], pos[ZZ] );
      std::cout <<"failed \n";
      f2 << "failed \n";
      f2.close();
      printed_result = 1;
      
      exit( 0 );
       }
    }
   

    if (pos[ZZ] < s->goal.total_height &&  pos[ZZ] > s->goal.height  && pos[XX] > s->goal.left_bound && pos[XX] < s->goal.right_bound)
    {
        
        counter ++;
    }

    //printf("%i\n", counter);
    // maybe later need to compare the left and right boundaries, so far so good
    if (counter > 400)
    {
       if ( !printed_result ){
        std::ofstream file;
        std::ofstream f2;
        file.open("./results/result.txt");
        f2.open("./results/rstring.txt");
      //  file.open("./results/result.txt");
        file << pos[XX] << " ";
        file << pos[ZZ] << " ";
	file << x_vel_1 << " ";
      	file << z_vel_1 << " ";
      	file << z_pos_1 << " ";
      	file << x_pos_1 << " ";
	file << x_vel << " ";
	file << z_vel << " ";
      	file << z_pos << " ";
	file << x_pos << " ";
	file << count << "\n";
        file.close();
        f2 << "succeed \n";
        f2.close();
        std::cout << "succeed \n";
        printed_result = 1;
	//track.close();
	exit(0);
      }
        //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
    }

    //if (counter > 100)
    //{
     // exit(0);
    //}

    //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
    // if till now not terminated, means the ball stucks somewhere 
    if (!printed_result){
      if (s->time > 4){
        std::ofstream file;
        file.open("./results/result.txt");
        std::ofstream f2;
        //file.open("./results/result.txt");
        f2.open("./results/rstring.txt");
        file << pos[XX] << " ";
        file << pos[ZZ] << " ";
	file << x_vel_1 << " ";
     	file << z_vel_1 << " ";
      	file << z_pos_1 << " ";
      	file << x_pos_1 << " ";

	file << x_vel << " ";
	file << z_vel << " ";
     	file << z_pos << " ";
	file << x_pos << " ";	
	file << count << "\n";
        file.close();
        std::cout << "failed \n";
        f2 << "failed \n";
        f2.close();
	//track.close();
        printed_result = 1;
        exit(0);
      }
    }
  


}



/*********************/
int main( int argc, const char **argv )
{

  //float init_parameters[MAX_N_PARAMETERS+1];


  //nit_default_parameters( &sim );

  /* Overall parameters */
  sim.duration = 10.0;
  sim.time_step = 0.001;
 // s->time_step = 0.001;

  sim.rand_scale = 0.0;
  sim.rand_seed = 1;

  const char* filename;
  if (argc == 2){
	filename = argv[1];
  }
  else {
	filename = "./params/dyn_setup.txt";
	}

  init_sim(&sim, filename);



  sim.n_func_calls_per_eval = 1;
  sim.all_time_low_cost = 1e20;
  sim.debug_criterion = 1;

 
		
  //track.open("./tests/track.txt");
  while (1) {
    simLoop(0, &sim);
  }
  //track.close();
  // clean up
  dJointGroupDestroy (sim.contactgroup);
  dSpaceDestroy (sim.space);
  dWorldDestroy (sim.world);
  dCloseODE();
  return 0;
}
