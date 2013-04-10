/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

//! These are user defined include files
//! Included in double quotes - the path to find these has to be given at compile time
#include "render.hpp"
#include "cs296_base.hpp"
#include "callbacks.hpp"

//! GLUI is the library used for drawing the GUI
//! Learn more about GLUI by reading the GLUI documentation
//! Learn to use preprocessor diectives to make your code portable
#ifndef __APPLE__
#include "GL/glui.h"
#else
#include "GL/glui.h"
#endif

//! These are standard include files
//! These are usually available at standard system paths like /usr/include
//! Read about the use of include files in C++
#include <cstdio>
#include <iostream>
#include <sys/time.h>

//! Notice the use of extern. Why is it used here?
namespace cs296
{
  extern int32 test_index;
  extern int32 test_selection;
  extern int32 test_count;
  extern cs296::sim_t* entry;
  extern cs296::base_sim_t* test;
  extern cs296::settings_t settings;
  extern const int32 frame_period;
  extern float settings_hz;
  extern int32 width;
  extern int32 height;
  extern int32 main_window;
};

//! This opens up the cs296 namespace
//! What is the consequence of opening up a namespace?
using namespace cs296;


//! This function creates all the GLUI gui elements
void create_glui_ui(void)
{
  GLUI *glui = GLUI_Master.create_glui_subwindow( main_window, GLUI_SUBWINDOW_BOTTOM );
  
  glui->add_statictext("Simulation Timesteps"); 
  GLUI_Spinner* velocityIterationSpinner =
    glui->add_spinner("Velocity Iterations", GLUI_SPINNER_INT, &settings.velocity_iterations);
  velocityIterationSpinner->set_int_limits(1, 500);
  
  GLUI_Spinner* positionIterationSpinner =
    glui->add_spinner("Position Iterations", GLUI_SPINNER_INT, &settings.position_iterations);
  positionIterationSpinner->set_int_limits(0, 100);
  
  GLUI_Spinner* hertzSpinner =
    glui->add_spinner("Sim steps per frame", GLUI_SPINNER_FLOAT, &settings_hz);
  hertzSpinner->set_float_limits(5.0f, 200.0f);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Simulation Parameters"); 
  glui->add_checkbox("Warm Starting", &settings.enable_warm_starting);
  glui->add_checkbox("Time of Impact", &settings.enable_continuous);
  glui->add_checkbox("Sub-Stepping", &settings.enable_sub_stepping);


  
  new GLUI_Column( glui, false );
  glui->add_statictext("Display Options"); 
  GLUI_Panel* drawPanel =	glui->add_panel("Draw");
  glui->add_checkbox_to_panel(drawPanel, "Shapes", &settings.draw_shapes);
  glui->add_checkbox_to_panel(drawPanel, "Joints", &settings.draw_joints);
  glui->add_checkbox_to_panel(drawPanel, "AABBs", &settings.draw_AABBs);
  glui->add_checkbox_to_panel(drawPanel, "Statistics", &settings.draw_stats);
  glui->add_checkbox_to_panel(drawPanel, "Profile", &settings.draw_profile);
  
  new GLUI_Column( glui, false );
  glui->add_button("Pause", 0, callbacks_t::pause_cb);
  glui->add_button("Single Step", 0, callbacks_t::single_step_cb);
  glui->add_button("Restart", 0, callbacks_t::restart_cb);
  
  glui->add_button("Quit", 0,(GLUI_Update_CB)callbacks_t::exit_cb);
  glui->set_main_gfx_window( main_window );
}


//! This is the main function
int main( int count, char *numIter[ ] ){
	if( count != 2 )
		return 1;

	test_count = 1;
	test_index = 0;
	test_selection = test_index;

	entry = sim;
	test = entry->create_fcn();

	//! This initializes GLUT
	//glutInit(&argc, argv);
	//glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	//glutInitWindowSize(width, height);

	//char title[75];
	/*
	sprintf(title, "CS296 Base Code for Group 13. Running on Box2D %d.%d.%d",
		b2_version.major, b2_version.minor, b2_version.revision);
	*/
	//main_window = glutCreateWindow(title);

	//! Here we setup all the callbacks we need
	//! Some are set via GLUI
	//GLUI_Master.set_glutReshapeFunc(callbacks_t::resize_cb);  
	//GLUI_Master.set_glutKeyboardFunc(callbacks_t::keyboard_cb);
	//GLUI_Master.set_glutSpecialFunc(callbacks_t::keyboard_special_cb);
	//GLUI_Master.set_glutMouseFunc(callbacks_t::mouse_cb);
	//! Others are set directly
	//glutDisplayFunc(callbacks_t::display_cb);
	//glutMotionFunc(callbacks_t::mouse_motion_cb);
	//glutKeyboardUpFunc(callbacks_t::keyboard_up_cb); 
	//glutTimerFunc(frame_period, callbacks_t::timer_cb, 0);

	//! We create the GLUI user interface
	//create_glui_ui();

	//! Enter the infinite GLUT event loop
	//glutMainLoop();
  
	int iterations = atoi( numIter[ 1 ] );
	b2World *rtTestWorld = test->get_world();

	float32 stepTime = 0.0, collisionResolutionTime = 0.0;
	float32 updateVelocitiesTime = 0.0, updatePositionsTime = 0.0;
	double totalTime;
	
	float32 time_step = settings.hz > 0.0f ? 1.0f / settings.hz : float32( 0.0f );
	
	struct timeval startTime, endTime;
	gettimeofday( &startTime, NULL );
	for( int i = 0; i < iterations; i++ ){
		rtTestWorld->Step( time_step, settings.velocity_iterations, settings.position_iterations );

		stepTime += rtTestWorld->GetProfile().step;
		collisionResolutionTime += rtTestWorld->GetProfile().collide;
		updateVelocitiesTime += rtTestWorld->GetProfile().solveVelocity;
		updatePositionsTime += rtTestWorld->GetProfile().solvePosition;
	}
	gettimeofday( &endTime, NULL );
	totalTime = startTime.tv_usec + startTime.tv_sec * 1000000.0;
	totalTime = ( endTime.tv_usec + endTime.tv_sec * 1000000.0 - totalTime )/1000.0;

	stepTime /= iterations;
	collisionResolutionTime /= iterations;
	updateVelocitiesTime /= iterations;
	updatePositionsTime /= iterations;

	std::printf( "Total Iterations: %d\n", iterations );
	std::printf( "Average time per step is %f ms\n", stepTime );
	std::printf( "Average time for collisions is %f ms\n", collisionResolutionTime );
	std::printf( "Average time for velocity updates is %f ms\n", updateVelocitiesTime );
	std::printf( "Average time for position updates is %f ms\n\n", updatePositionsTime );
	std::printf( "Total time for loop is %f ms\n", totalTime );

	return 0;
}
