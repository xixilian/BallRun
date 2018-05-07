/****************************************************************
Dropping ball program

I assume some version of Open Dynamics Engine (ODE) is installed.

Changed drawstuff so this is negative and coordinate system has
X positive:
#define LIGHTY (-0.4f)

Note: 
x points to viewer's right.
z is up.
y points away from viewer, so angles are negative.

****************************************************************/

#include <math.h>
#include <string.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <algorithm>
#include <iostream>
#include <fstream>

/****************************************************************/

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

/****************************************************************/
/****************************************************************/

#define TRUE 1
#define FALSE 0

#define XX 0
#define YY 1
#define ZZ 2

// #define TIME_STEP 0.005
#define TIME_STEP 0.001
//#define WAIT_TO_START 0.0

// These should be set up in configuration file
#define BALL_DIAMETER 0.017270193333 // meters
#define WALL_WIDTH 0.55 // meters
#define WALL_THICKNESS 0.01 // meters
#define WALL_HEIGHT 0.55 // meters
#define FUDGE1 (BALL_DIAMETER/10) // space between ball and wall

// size in pixels of window
#define WINDOW_WIDTH 700
#define WINDOW_HEIGHT 700

// used to allocate space for obstacles
#define MAX_N_GEOMS 100

//#define OBSTACLE_NUMBER 3

// object types
#define BALL 0
#define OBSTACLE 1
#define WALL 2
#define UNKNOWN -1

#define BALLPOS 0.52

/****************************************************************/

// select correct drawing functions (we are using double version)
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif

/****************************************************************/
// Typedefs

typedef struct rect
{
  double dimensions[3];
  double pos[3];
  double angle;
}
  RECT;

typedef struct setup
{
  // these are static ODE objects: just geometries

  dGeomID the_wall;

  RECT obstacles[MAX_N_GEOMS];
  dGeomID the_obstacles[MAX_N_GEOMS];
  int n_obstacles;
}
  SETUP;

// separate the goal geoms from the obstacles
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


/****************************************************************/
// Globals

static dReal my_time = 0; // what time is it?

// ODE dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// the ODE ball
static dBodyID ball_body;
static dGeomID ball_geom;

// the current setup
static SETUP current_setup;

// new stuff
static GOAL  goal;

static int simulation_running = 1;

static int counter = 0;

static float  WAIT_TO_START = 0.0;

static int play = 1;

static int set_body = 0;

static int write = 0;
/****************************************************************/

// This function is called at the start of the simulation
// to set up the point of view of the camera.
// to initialize the ball position
static void start()
{
  float xyz[3] = {(float) (WALL_WIDTH/2),-0.45f,(float) (WALL_HEIGHT/2)};
  float hpr[3] = {90.000f,0.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf("(Press:\t 'a' to move first (from top to down) obstacle in x axis to the right.))\n"
          "\t's' to move the first obstacle in x axis to the left\n"
          "\t 'q' to move the first obstacle in the z axis up\n"
          "\t 'w' to move the first obstacle in the z axis down\n"
          "\t 'z' to turn the first obstacle clockwise\n"
          "\t 'x' to turn the first obstacle anti-clockwise\n"
          "\t 'f' to move the second obstacle in x axis to the right\n"
          "\t 'g' to move the second obstacle in x axis to the left\n"
          "\t 'r' to move the second obstacle in z axis up\n"
          "\t 't' to move the second obstacle in z axis down\n"
          "\t 'v' to turn the second obstacle clockwise\n"
          "\t 'b' to turn the second obstacle anti-clockwise\n"
          "\t 'j' to move the third obstacle in x axis to the right\n"
          "\t 'k' to move the third obstacle in x axis to the left\n"
          "\t 'i' to move the third obstacle in the z axis up\n"
          "\t 'o' to move the third obstacle in the z axis down\n"
          "\t 'n' to turn the third obstacle clockwise\n"
          "\t 'm' to turn the third obstacle anti-clockwise\n"
          "\t '-' to start\n"
           );
}

/****************************************************************/
// this program figures out what kind of collision it is.

void identify_contact( dGeomID o, int *type, const char **str, SETUP *h )
{
  int i;

  if ( o == ball_geom )
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
  identify_contact( o1, &o1_type, &o1_string, &current_setup );
  identify_contact( o2, &o2_type, &o2_string, &current_setup );
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
     //  contact.surface.mu =1.0;
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
      //contact.surface.mu = 3.0;
	contact.surface.mu = 3.0;
      // bounce is the amount of "bouncyness".
      //contact.surface.bounce = 0.38; // used to be 0.1
	contact.surface.bounce = 0.38;
      // bounce_vel is the minimum incoming velocity to cause a bounce
      contact.surface.bounce_vel = 0.01;
      // constraint force mixing parameter
      contact.surface.soft_cfm = 0.01;  // 0.001 in bounce
    }
  else
    return; // no obstacle or wall involved, give up

  if (int numc = dCollide (o1,o2,1,&contact.geom,sizeof(dContact)))
    {
      dJointID c = dJointCreateContact (world,contactgroup,&contact);
      dJointAttach (c,b1,b2);
    }
}

/****************************************************************/
// no controller

void do_user_stuff( SETUP *h )
{
  return;
}

/****************************************************************/

void do_simulation_step()
{
  // find collisions and add contact joints
  dSpaceCollide( space, 0, &nearCallback );

  // step the simulation
  dWorldStep( world, TIME_STEP );  

  my_time += TIME_STEP;

  // remove all contact joints
  dJointGroupEmpty( contactgroup );
}

/****************************************************************/

void draw_stuff()
{
  int i;
  const dReal *pos;
  const dReal *R;
  dReal sides[3];

  // redraw sphere at new location
  pos = dGeomGetPosition( ball_geom );
  R = dGeomGetRotation( ball_geom );
  dsSetColor (5,0,0);
  dsDrawSphere( pos, R, dGeomSphereGetRadius( ball_geom ) );

  // redraw wall
  pos = dGeomGetPosition( current_setup.the_wall );
  R = dGeomGetRotation( current_setup.the_wall );
  dGeomBoxGetLengths( current_setup.the_wall, sides );
  dsSetColor( 5, 5, 5 );
  dsDrawBox( pos, R, sides );

  // redraw obstacles
  for ( i = 0; i < current_setup.n_obstacles; i++ )
    {
      pos = dGeomGetPosition( current_setup.the_obstacles[i] );
      R = dGeomGetRotation( current_setup.the_obstacles[i] );
      dGeomBoxGetLengths( current_setup.the_obstacles[i], sides );
      dsSetColor( 0, 2, 1 );
      if ( i > 2){
	dsSetColor(2, 1, 0);
	}
      dsDrawBox( pos, R, sides );
    }
}
/***************************************************************/

void update(RECT r, dGeomID o)
{
  const dReal *pos;
  const dReal *R;
  dReal sides[3];

  dQuaternion q;

  dGeomSetPosition( o, r.pos[XX], r.pos[YY], r.pos[ZZ] );
  
  q[0] = cos( r.angle/2 );
  q[1] = 0.0;
  q[2] = sin( r.angle/2 );
  q[3] = 0.0;
  dGeomSetQuaternion ( o, q );

  pos = dGeomGetPosition( o );
  R = dGeomGetRotation( o );
  dGeomBoxGetLengths( o, sides );
  dsSetColor( 0, 0, 1 );
  dsDrawBox( pos, R, sides );
}


/****************************************************************/

/*
This is the main simulation loop that calls the collision detection
function, steps the simulation, resets the temporary contact joint
group, and redraws the objects at their new position.
*/

// simulation loop
static void simLoop (int pause)
{
  const dReal *pos;
  const dReal *pos2;
  const dReal *vel;
  //const dReal *an_vel;
  static int printed_result = 0;
  dQuaternion q;

  // do user defined stuff
  do_user_stuff( &current_setup );

  if ( simulation_running )
    do_simulation_step();

    if(play){

        dMass m;
        ball_body = dBodyCreate( world );
        //ball_geom = dCreateSphere( space, BALL_DIAMETER/2 );
        dMassSetSphere( &m, 0.6, BALL_DIAMETER/2 );
        dBodySetMass( ball_body, &m );
        dGeomSetBody( ball_geom, ball_body );
        dBodySetLinearDamping( ball_body, 1e-4 ); 
        dBodySetLinearDampingThreshold( ball_body, 1e-7 );
        //set initial position and velocity
        dBodySetPosition( ball_body, 0.0, 0.0, BALLPOS );
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
        dBodySetQuaternion ( ball_body, q );
        dBodySetLinearVel( ball_body, 0.1, 0.0, 0.0 );
        dBodySetAngularVel( ball_body, 0.0, 0.0, 0.0 );

        set_body = 1;
        // do it once
        play = 0;

        write = 1;
  }

  // hold ball at top
  if (set_body){
    if ( my_time < WAIT_TO_START )
      {
        dBodySetPosition( ball_body, 0.0, 0.0, 0.52 );
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
        dBodySetQuaternion ( ball_body, q );
        dBodySetLinearVel( ball_body, 0.1, 0.0, 0.0 );
        dBodySetAngularVel( ball_body, 0.0, 0.0, 0.0 );
      }

      pos = dGeomGetPosition( ball_geom );
      pos2 = dBodyGetPosition( ball_body );
	vel = dBodyGetLinearVel (ball_body);
	//an_vel = dBodyGetAngularVel(ball_body);
        //printf ( " %lg %lg\n", pos[XX], pos[ZZ] );
	
	if ((goal.total_height - pos[ZZ]) <= (BALL_DIAMETER/2.0) && (goal.total_height - pos[ZZ]) > 0 && pos[XX] > goal.left_bound && pos[XX] < goal.right_bound)
	{
		//printf ( " %lg %lg %lg %lg %lg \n", vel[XX], vel[ZZ], pos[XX], pos[ZZ], goal.total_height );
		//printf("hit");
		//printf ( "%lg %lg %lg\n",vel[ZZ], pos[ZZ], goal.total_height );
		 std::ofstream file;
      		file.open("./results/vels.txt");
      		file << vel[ZZ] << " ";
		//file << an_vel[ZZ] << " ";
		file << pos[XX] << " ";
      		file << pos[ZZ] << "\n";
      		file.close();
	}

	if ( vel[ZZ] >= 0.0 && pos[XX] > goal.left_bound && pos[XX] < goal.right_bound  
	&& (pos[ZZ] + (BALL_DIAMETER/2.0) - goal.total_height ) > 0.0 && (pos[ZZ] - goal.total_height ) < (BALL_DIAMETER/2.0)) 
	{
		printf ( " %lg %lg %lg \n", vel[ZZ], pos[ZZ],  goal.total_height);
		
	}

      if ( pos[ZZ] < BALL_DIAMETER/2 )
        {
          simulation_running = FALSE;
          dGeomSetPosition( ball_geom, pos[XX], pos[YY], BALL_DIAMETER/2 - 0.0001 );
        dBodySetPosition( ball_body, pos2[XX], pos2[YY], BALL_DIAMETER/2 - 0.0001 );
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
        dBodySetQuaternion ( ball_body, q );
        dBodySetLinearVel( ball_body, 0.0, 0.0, 0.0 );
        dBodySetAngularVel( ball_body, 0.0, 0.0, 0.0 );
        if ( !printed_result )
  	   {
  	  // only the ball on the ground it will be printed...
      std::ofstream file;
      file.open("./results/result.txt");
      file << pos[XX] << " ";
      file << pos[ZZ] << "\n";
      file.close();

  	  //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
  	  std::cout <<"failed \n";
  	  printed_result = 1;
  	  exit( 0 );
  	   }
    }
    
    if(write)
    {
      std::ofstream f;
      f.open("./results/output.txt");

      for (int i = 0; i < current_setup.n_obstacles; i ++)
      {
          f << current_setup.obstacles[i].pos[XX] << " ";
          f << current_setup.obstacles[i].pos[YY] << " ";
          f << current_setup.obstacles[i].pos[ZZ] << " ";
          f << current_setup.obstacles[i].angle << "\n";
        
      }
      f.close();
      // write it once
      write = 0;
    }
    // not exit, it's success
    //std::cout << "successed \n";
    //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );

    if (pos[ZZ] < goal.total_height &&  pos[ZZ] > goal.height && pos[XX] > goal.left_bound && pos[XX] < goal.right_bound)
    {
        
        counter ++;
    }

    //printf("%i\n", counter);
    // maybe later need to compare the left and right boundaries, so far so good
    if (counter > 500)
    {
       if ( !printed_result ){
        std::ofstream file;
        file.open("./results/result.txt");
        file << pos[XX] << " ";
        file << pos[ZZ] << "\n";
        file.close();
        std::cout << "succeed \n";
        printed_result = 1;
	exit(0);
      }
        //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
    }

   // if (counter > 150)
   // {
    //  exit(0);
    //}

    //printf ( "%lg %lg %lg\n", my_time, pos[XX], pos[ZZ] );
    // if till now not terminated, means the ball stucks somewhere 
    if (!printed_result){
      if (my_time > 5){
        std::ofstream file;
        file.open("./results/result.txt");
        file << pos[XX] << " ";
        file << pos[ZZ] << "\n";
        file.close();
        std::cout << "failed \n";
        printed_result = 1;
        exit(0);
      }
    }
  }

  draw_stuff();
}

/****************************************************************/

int read_setup_file( char *filename, SETUP *h, GOAL *g )
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
	  printf( "obstacle: %g %g %g %g %g %g %g\n",
		  h->obstacles[i].dimensions[XX],
		  h->obstacles[i].dimensions[YY],
		  h->obstacles[i].dimensions[ZZ],
		  h->obstacles[i].pos[XX],
		  h->obstacles[i].pos[YY],
		  h->obstacles[i].pos[ZZ],
		  h->obstacles[i].angle );
	  h->n_obstacles++;
	  continue;
	}
      
      fprintf( stderr, "bad keyword %s in setup file %s\n", buffer, filename );
      exit( -1 );
    }

  fclose( stream );

	g->n_pieces = 3;
	// the last 3 pieces of obstacles
	//for(i = current_setup.n_obstacles - 1 ; i > 2; i -- )
	//{
	//	j = g->n_pieces;
	//	g-> pieces[j]
	//}
  int total_pieces = current_setup.n_obstacles;
  int o_pieces = total_pieces -3;


	// copy array 
	std::copy(current_setup.obstacles+ o_pieces, current_setup.obstacles+total_pieces,g->pieces);

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

	//printf("goal total height: %g\n ", g->total_height);


  //printf( "laying_piece x: %g\n", g->laying_piece.pos[XX]);

	for (i = 0; i < g->n_pieces;i++)
  {
   // printf("piece x index %i\n" , i );
   // printf("piece x pos %g\n" , g->pieces[i].pos[XX] );

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


  //printf( "left_bound: %g \n", g->left_bound);
  //printf( "right_bound: %g\n", g->right_bound);

  //printf( "height: %g\n", g->height);
}

/****************************************************************/

void create_bodies( char *filename, SETUP *h, GOAL *g )
{
  int i;
  dQuaternion q;
  
  
  read_setup_file( filename, h,g );

  dMass m;
  // create ball object
  //ball_body = dBodyCreate( world );
  ball_geom = dCreateSphere( space, BALL_DIAMETER/2 );
  //dMassSetSphere( &m, 0.6, BALL_DIAMETER/2 );
  //dBodySetMass( ball_body, &m );
  //dGeomSetBody( ball_geom, ball_body );
  //dBodySetLinearDamping( ball_body, 1e-4 ); 
  //dBodySetLinearDampingThreshold( ball_body, 1e-7 );
  // set initial position and velocity
  //dBodySetPosition( ball_body, 0.0, 0.0, WALL_HEIGHT );
  //q[0] = 1.0;
  //q[1] = 0.0;
  //q[2] = 0.0;
  //q[3] = 0.0;
  //dBodySetQuaternion ( ball_body, q );
  //dBodySetLinearVel( ball_body, 0.0, 0.0, 0.0 );
  //dBodySetAngularVel( ball_body, 0.0, 0.0, 0.0 );

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

  h->the_wall = dCreateBox( space, WALL_WIDTH, WALL_THICKNESS, WALL_HEIGHT );
  dGeomSetPosition( h->the_wall, WALL_WIDTH/2 - BALL_DIAMETER,
		    BALL_DIAMETER + FUDGE1, WALL_HEIGHT/2 );

  for ( i = 0; i < h->n_obstacles; i++ )
    {
      h->the_obstacles[i] =
	     dCreateBox( space, h->obstacles[i].dimensions[XX],
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
    }
}

/*******************************************/


//============================

// called when a key pressed

static void command (int cmd)
{
  
  switch(cmd) {
    case 'a' : case 'A' :
       current_setup.obstacles[0].pos[XX] += 0.005;
      //dGeomSetPosition( current_setup.the_obstacles[0], current_setup.obstacles[0].pos[XX],
      //  current_setup.obstacles[0].pos[YY],
      //  current_setup.obstacles[0].pos[ZZ] );
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
      break;
    
    case 's' : case 'S' :
      current_setup.obstacles[0].pos[XX] -= 0.005;
      
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
       break;
    case 'q' : case 'Q' :
      current_setup.obstacles[0].pos[ZZ] += 0.005;
      
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
       break;
    case 'w' : case 'W' :
      current_setup.obstacles[0].pos[ZZ] -= 0.005;
      
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
       break;
    case 'z' : case 'Z' :
      current_setup.obstacles[0].angle += 0.01;
      
      
      //dGeomSetQuaternion ( current_setup.the_obstacles[0], q );
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
       break;
      
    case 'x' : case 'X' :
      current_setup.obstacles[0].angle -= 0.01;
      
      
      //dGeomSetQuaternion ( current_setup.the_obstacles[0], q );
      update(current_setup.obstacles[0], current_setup.the_obstacles[0]);
       break;

    case 'f' : case 'F' :
      current_setup.obstacles[1].pos[XX] += 0.005;
      
      update(current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 'g' : case 'G' :
      current_setup.obstacles[1].pos[XX] -= 0.005;
     
      update(current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 'r' : case 'R' :
      current_setup.obstacles[1].pos[ZZ] += 0.005;
      
      update(current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 't' : case 'T' :
      current_setup.obstacles[1].pos[ZZ] -= 0.005;
      
      update(current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 'v' : case 'V' :
      current_setup.obstacles[1].angle += 0.01;
      
      
      //dGeomSetQuaternion ( current_setup.the_obstacles[1], q );
      update(current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 'b' : case 'B' :
      current_setup.obstacles[1].angle -= 0.01;
      
      
      //dGeomSetQuaternion ( current_setup.the_obstacles[1], q );
      update( current_setup.obstacles[1], current_setup.the_obstacles[1]);
       break;
    case 'j' : case 'J' :
      current_setup.obstacles[2].pos[XX] += 0.005;
     
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
    case 'k' : case 'K' :
      current_setup.obstacles[2].pos[XX] -= 0.005;
     
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
    case 'i' : case 'I' :
      current_setup.obstacles[2].pos[ZZ] += 0.005;
      
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
    case 'o' : case 'O' :
      current_setup.obstacles[2].pos[ZZ] -= 0.005;
     
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
    case 'n' : case 'N' :
      current_setup.obstacles[2].angle += 0.01;
     
    
      //dGeomSetQuaternion ( current_setup.the_obstacles[2], q );
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
    case 'm' : case 'M' :
      current_setup.obstacles[2].angle -= 0.01;
      
      
      //dGeomSetQuaternion ( current_setup.the_obstacles[2], q );
      update(current_setup.obstacles[2], current_setup.the_obstacles[2]);
       break;
      
    case '-' : 
      WAIT_TO_START = 0.05;
      play = 1;
  }
}

/****************************************************************/

/*
Main program: When the program starts, the callbacks are set up,
everything is initialized, and then the simulation is started.
*/

int main( int argc, const char **argv )
{
  dReal erp, cfm;
  char *filename;

  if ( argc < 2 )
    filename = (char *) "./params/dyn_setup.txt";
  else
    filename = (char *) (argv[1]);
  printf( "Using setup file %s\n", filename );

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  
  fn.step = &simLoop;

  
  fn.stop = 0;
  fn.command = &command;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
 
  dInitODE ();
  // create world
  world = dWorldCreate( );
  space = dHashSpaceCreate( 0 );
  dWorldSetGravity( world, 0.0, 0.0, -9.81 );
  dWorldSetCFM( world, 1e-5 );
  // dWorldSetERP (dWorldID, dReal erp);
  erp = dWorldGetERP( world );
  cfm = dWorldGetCFM( world );
  /*
  printf( "erp: %g, cfm: %g, kp: %g, kd: %g\n",
	  erp, cfm, erp/(cfm*TIME_STEP), (1 - erp)/cfm );
  */

  contactgroup = dJointGroupCreate( 0 );

  create_bodies( filename, &current_setup, &goal );

  // run simulation

  dsSimulationLoop( argc, (char **) argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn );
  
  

  // clean up
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}

/****************************************************************/
