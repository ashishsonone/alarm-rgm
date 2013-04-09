/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"




namespace cs296
{
  
  /// creates a plank at a specified position
/*!
   Plank is basically a thin long rectangular object.It has b2PolygonShape set as box.
   Its postion,angle,length and width of the plank can be passed as parameters. 
   @param[w]      b2World*  world in which to create the object
   @param[x]      float32   x-coordinate of plank's center
   @param[y]      float32   y-coordinate of plank's center
   @param[l]      float32   half-length of plank
   @param[isDyn]  bool      boolean variable to set whether plank is static or dynamic.Static by default
   @param[angle]  float32   angle at which plank(length-wise) is inclined to the horizontal anticlockwise.Angle is zero by default
   @param[b]      float32   width of the plank.It is 0.15 by default for most of the planks in the design are uniform
 */
b2Body* dominos_t::fixedplank(b2World* w,float32 x,float32 y,float32 l,bool isDyn=false,float32 angle=0.0,float32 b=0.15f){
  /**
  <ul>
  <li>first declare body b2body* plank and bodydefinition b2bodyDef bdef.
  <li>if the isDynamic input parameter is true set body-defintion's type to b2_dynamicBody.
  <li>use bdef.position.Set(x,y) to set position and bdef.angle=angle to set the inclination angle.
  <li>define a b2PolygonShape shape and use SetAsBox to set shape as box of length l and width b (inputs).
  <li>declare a body-fixture definition b2Fixturedef fd and set shape,density(0.25) and friction(0.1);
  <li>use world->CreateBody(bodydefinition) to create the plank and then assign fixture to it using CreateFixture(fixture def)
  **/
  float32 density=0.0f;
  b2Body *plank; //declare body
  b2BodyDef bdef;//declare bodydef
  if(isDyn){
    bdef.type=b2_dynamicBody;
  }
  bdef.position.Set(x,y); //set position
  bdef.angle=angle;
  b2PolygonShape shape; //declare shape
  shape.SetAsBox(l, b); //set shape to be box of length l

  b2FixtureDef fd; //fixture definition
  fd.shape = &shape;
  fd.density = 0.25f;
  fd.friction = 0.1f;

  plank = w->CreateBody(&bdef); //create body using body definition
  plank->CreateFixture(&fd); //density for static body is not used(as static body has zero density by default)

  return plank;
}

/// creates a domino
/*!
   domino is basically a thin vertical rectangular shape object. It has b2PolygonShape set as box. 
   Its postion,height and width of the domino can be passed as parameters. 

   @param[w]      b2World*  world in which to create the object
   @param[x]      float32   x-coordinate of domino's center
   @param[y]      float32   y-coordinate of domino's center
   @param[h]      float32   half-height of domino
   @param[b]      float32   width of the domino.It is 0.15 by default.
 */
b2Body* dominos_t::domino(b2World* w,float32 x,float32 y,float32 h=1.5f,float32 b=0.15f){
  /**
  <ul>
  <li>first declare body b2body* domino and bodydefinition b2bodyDef bdef.
  <li>use bdef.position.Set(x,y) to set position.
  <li>define a b2PolygonShape shape and use SetAsBox to set shape as box of length(or height) h and width b (which are inputs).
  <li>declare a body-fixture definition b2Fixturedef fd and set shape,density(4.0) and friction(0.1);
  <li>use world->CreateBody(bodydefinition) to create the domino and then assign fixture to it using CreateFixture(fixture def)
  **/
  b2Body *domino; //declare body
  b2BodyDef bdef;//declare bodydef

  bdef.type=b2_dynamicBody;
  bdef.position.Set(x,y); //set position
  b2PolygonShape shape; //declare shape
  shape.SetAsBox(b, h); //set shape to be box of length l

  b2FixtureDef fd; //fixture definition
  fd.shape = &shape;
  fd.density = 4.0f;
  fd.friction = 0.1f;


  domino = w->CreateBody(&bdef); //create body using body definition
  domino->CreateFixture(&fd); //density for static body is not used(as static body has zero density by default)
  
  return domino;
}


/// draws an edge between two points
/*!
    Its shape is b2EdgeShape.It is static object.
   @param[w]       b2World*  world in which to create the object
   @param[x1]      float32   x-coordinate of first point
   @param[y1]      float32   y-coordinate of first point
   @param[x2]      float32   x-coordinate of second point
   @param[y2]      float32   y-coordinate of second point
 */

b2Body* dominos_t::edge(b2World* w,float32 x1,float32 y1,float32 x2,float32 y2){
  /**
  <ul>
  <li>first declare body b2body* edge and bodydefinition b2bodyDef bd.
  <li>define a b2EdgeShape shape and use shape.Set(point1,point2) to define the edge location.point1 and point2 are of type b2Vec2
  <li>declare a body-fixture definition b2Fixturedef fd and set its shape
  <li>use world->CreateBody(bodydefinition) to create the edge and then assign fixture to it using CreateFixture(fixture def)
  **/
    b2Body* edge;
  
    b2EdgeShape shape;
    shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
  
    b2BodyDef bd;
    b2FixtureDef fd; //fixture definition
    fd.shape = &shape;

    edge = w->CreateBody(&bd);
    edge->CreateFixture(&fd);
    return edge;
}

/// creates a ball(circle) object
/*!
    Its position,radius and density can be given as parameters.It has b2CircleShape.
   @param[w]        b2World*  world in which to create the object
   @param[x1]       float32   x-coordinate of center
   @param[y1]       float32   y-coordinate of center
   @param[rad]      float32   radius of the ball
   @param[density]  float32   density of ball material.Default is 10.0
 */
b2Body* dominos_t::ball(b2World*w,float32 x,float32 y,float32 rad,float32 density=10.0){
  /**
  <ul>
  <li>first declare body b2body* ball and bodydefinition b2bodyDef bdef.
  <li> set type to b2_dynamicBody.
  <li>use bdef.position.Set(x,y) to set position.
  <li>define a b2PolygonShape circle and set its radius.
  <li>declare a body-fixture definition b2Fixturedef ballfd and set shape,density(from inputs) and friction to 0
  <li>use world->CreateBody(bodydefinition) to create the ball and then assign fixture to it using CreateFixture(fixture def)
  **/
  b2Body* ball;
  b2BodyDef bdef;//declare bodydef
  bdef.type=b2_dynamicBody;
  bdef.position.Set(x,y); //set position

  b2CircleShape circle;
  circle.m_radius = rad;
  
  b2FixtureDef ballfd;
  ballfd.shape = &circle;
  ballfd.density = density;
  ballfd.friction = 0.0f;
  ballfd.restitution = 0.0f;

  ball=w->CreateBody(&bdef);
  ball->CreateFixture(&ballfd);

  return ball;
}

/// creates a box at a specified position
/*!
   Box is basically a rectangular object.It has b2PolygonShape set as box.
   The postion,length,width,density and friction values of the plank can be passed as parameters. 
   @param[w]        b2World*  world in which to create the object
   @param[x]        float32   x-coordinate of box's center
   @param[y]        float32   y-coordinate of box's center
   @param[len]      float32   half-length of box
   @param[wid]      float32   half-width of the box.
   @param[friction] float32   coefficient of friction that box exibits with external world.Defaults to 0.5
 */
b2Body* dominos_t::box(b2World*w,float32 x,float32 y,float32 len,float32 wid,float32 density,float32 friction=0.5f){
  /**
  <ul>
  <li>first declare body b2body* ball and bodydefinition b2bodyDef bdef.
  <li>set type to b2_dynamicBody.
  <li>use bdef.position.Set(x,y) to set position.
  <li>define a b2PolygonShape shape and use SetAsBox to set shape as box of length len and width wid (which are inputs).
  <li>declare a body-fixture definition b2Fixturedef ballfd and set shape,density and friction (from inputs)
  <li>use world->CreateBody(bodydefinition) to create the ball and then assign fixture to it using CreateFixture(fixture def)
  **/
  b2Body* box;
  b2BodyDef bdef;//declare bodydef
  bdef.type=b2_dynamicBody;
  bdef.position.Set(x,y); //set position

  b2PolygonShape shape;
  shape.SetAsBox(len, wid);

  b2FixtureDef fd;
  fd.shape = &shape;
  fd.density = density;
  fd.friction = friction;
  fd.restitution = 0.0f;

  box=w->CreateBody(&bdef);
  box->CreateFixture(&fd);

  return box;
}


/// main constructor creating the box2d simulation world.
  dominos_t::dominos_t()
  {
    
    /**
    <B> Note : Here we use functions fixedplank(),edge(),box(),ball(),domino() which are members of dominos_t class to create objects wherever
    requird without any detailed explaination .Please refer to documentation of these functions for detaied explanation about above functions</B><br>
    <h3>Ground at the botton </h3>
    The ground is essentially an edge shape object extending from left(-90) to right end(+90) of the screen located at the bottom(y=0).
    It is as bottommost part of the design.
     **/
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }



    /**<h2>EXPLAINING THE LOWER HALF SECTION OF THE DESIGN</h2>
      consists of dominos system,falling maze, see-saw system,the pulley blocking system
    **/

    /**
      <h3>Dominos Holder</h3>
      It is the plank which holds the dominos.Centered at (15.0,25.0) it is of half-length 25.0.
      used fixedplank() function to create it.
    **/

    float32 domHY=25.0f,domHCenterX=15.0f,domHLen=25.0f;
    float32 domHX1 = domHCenterX - domHLen-1;

    b2Body* dominosholder=fixedplank(m_world,domHCenterX,domHY,domHLen);

    /**
      <h3>Dominos on top of dominos holder</h3>
      create stack of dominos of half-height 1.5, placed on top of the holder.Use a for-loop to create 8 dominos side by side 
      with a gap of 1.5 between them.used domino() function.
    **/
    {
      float32 DOMY=26.5f,STARTX=-5.0f,GAP=1.5f;
      for(int i=0;i<8;i++){
        domino(m_world,STARTX+i*GAP,DOMY);
      }

    /**
      <h3>the inclined edge </h3>
      situated towards the right-most domino.
      makes the incoming ball coming from the right maze jump into the stack of dominos thus making them fall.
      used edge() function to create it.
    **/
    edge(m_world,STARTX+7*GAP,domHY+1,STARTX+7*GAP+5,domHY);
    }

    
    /**<h3> the ball at end of dominos</h3>
      ball is of radius 0.8 and density 10.0. It is located to the left of the dominos on the domino-holder itself.
      It is the ball which goes down the falling-maze and makes the anvil fall on the see-saw.
      used ball() function.
    **/
    float32 radius=0.8;
    ball(m_world,-7.0f,domHY+radius,radius);
    

    /**<h3>the falling maze</h3>
      It is zig-zag maze at left end of domino-holder which directs the ball down to anvil.
      the edges are inclined like \ / \ and vertical gap(bottom to bottom) between two successive edges of maze is 2.5 m.
      So the maze spans 7.5 m vertically. 
      used edge() function.
    **/
    float32 mazeHeight=10,mazegap=2.5;
    edge(m_world,(domHX1-4),domHY+1,(domHX1+0),domHY-mazegap);
    edge(m_world,(domHX1+3),domHY-mazegap,(domHX1+0),domHY-2*mazegap);
    edge(m_world,(domHX1-4),domHY-2*mazegap,(domHX1+2),domHY-3*mazegap-1);

    /**<h3>anvil holder</h3>
      It is plank of half-length 5 and located at 10 m below the domino holder.
      placed vertically below the falling-maze.
      used fixedplank() method.
    **/
    float32 anvHLen=5; //anvil holder length
    fixedplank(m_world,domHX1,domHY-mazeHeight,anvHLen);

    /**<h3>the anvil</h3>
      it is simply a box of size 1x1 placed at right end of anvil-holder.It is supposed to fall on the see-saw just below it.
      It is very heavy(density 30.0)
    **/
    float32 anvLen=1.0f;
    float32 anvilDensity=30.0f;
    box(m_world,domHX1+anvHLen,domHY-mazeHeight+1,anvLen,anvLen,anvilDensity); 



    /**<h3>The see-saw system at the bottom</h3>**/
    {
      /**<ul>
      <li><b>The triangular wedge</b>.
          It is on which rests the see-saw.It is of height 1.5.
          It is of shape b2PolygonShape defined by 3 vertices(which are b2Vec2 type) set at (-1,0),(1,0) ,(0,1.5) relative to wedge's center
          its fixture def is wedgefd. wedgefd's shape is one defined above and density to 10.0.
          b2BodyDef wedgebd's position is set and the m_world->createBody is used to create the body. 
      </ul>
      **/
      float32 wedX=domHX1+anvHLen-2;
      float32 wedY=0.5f;
      float32 wedHeight=1.5;

      b2Body* wedge;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,wedHeight);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      b2BodyDef wedgebd;

      wedgebd.position.Set(wedX, wedY);

      wedge = m_world->CreateBody(&wedgebd);
      wedge->CreateFixture(&wedgefd);

      /**<ul>
      <li><b>the see-saw plank</b>
        create and put the plank on the triangle wedge of lenght 5 using fixedplank() function.Set isDynamic parameter to true to 
        make the plank dynamic.
      </ul>
      **/
      b2Body* plank = fixedplank(m_world,wedX,wedY+wedHeight,5,true);

      /**<ul>
      <li><b>the hinge</b>
        join the two - see saw and plank using a revolute joint.
        define b2RevoluteJointDef jd and vector anchor.Set anchor to top point of triangular wedge where plank rests
        initialise the joint with wedge ,plank (as bodies) and anchor(as hinge)
      </ul>
      **/
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(wedX, wedY+wedHeight);
      jd.Initialize(wedge, plank, anchor);
      m_world->CreateJoint(&jd);

      /**<ul>
      <li><b>the weight-box</b>
          put a box on the left end of the see-saw of size 0.5 and density 1.0.
      </ul>
      **/
      float32 botDensity=1.f,botSize=0.5f;
      b2Body *bot=box(m_world,wedX-3.7,wedY+wedHeight+botSize,botSize,botSize, botDensity,2);

      /**<ul>
      <li><b>the see-saw holder</b>
          a rectangle to hold plank at an inclination to make sure box goes in correct projectile path when anvil falls)
      </ul>
      **/
      
      float32 suppSize=wedHeight/2; //half-height just touching levelled plank
      box(m_world,wedX-5,suppSize,0.5,suppSize+0.5,botDensity); //extra height(+0.1) to keep plank slant

    }


    //now the bloking system at bottom right
    {
    float32 SysX=10;
    float32 SysLowerY=5.0f;

    //plank holding the bottom support
    float32 lowerPlankLen=5.0f;
    fixedplank(m_world,SysX,SysLowerY,lowerPlankLen);

    //support resting on the lower plank
    float32 supportHt=1,Density=0.2f,friction=0.02;//hor len of box is 1
    float32 supportX=SysX-lowerPlankLen+1  ,  supportY=SysLowerY+supportHt;
    box(m_world,supportX,supportY,1.3,supportHt,Density,friction);

    //the blocker
    float32 blockerHt=4;    //hor len of box is 1
    float32 blockerX=supportX  ,   blockerY=SysLowerY+2*supportHt+blockerHt;
    box(m_world,blockerX,blockerY,1,blockerHt,0,0);

    //the upper plank(left and right part)
    float32 SysUpperY=SysLowerY+2*supportHt+2*blockerHt-2; //effectively 8.0 above the lower plank
    float32 leftLen=1,leftX=supportX-1.01-leftLen;
    fixedplank(m_world,leftX,SysUpperY,leftLen);//THE LEFT PART

    edge(m_world, supportX-1,SysUpperY,supportX-1,SysUpperY-4);//the guards
    edge(m_world,supportX+1,SysUpperY,supportX+1,SysUpperY-4);

    float32 rightLen=5,rightX=supportX+1.01+rightLen;
    fixedplank(m_world,rightX,SysUpperY,rightLen);//THE RIGHT PART

    //the box on left end of pulley
    float32 boxLen=1,boxHt=1.5;
    float32 boxX=supportX-1.01-boxLen , boxY= SysUpperY+boxHt;
    b2Body* lbox=box(m_world,boxX,boxY,boxLen,boxHt,2);

    //weight on the right end of pulley
    float32 WtSize=2;
    float32 WtX=rightX+rightLen+0.5 ,  WtY=SysUpperY-5;
    b2Body* rWt= box(m_world,WtX,WtY,WtSize,WtSize,10);

    //now the pulley joining
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorGround1(WtX, boxY); // Anchor point for ground 1(in horizontally aligned with left box and vertically aligned with right Weight)
      b2Vec2 worldAnchorGround2(WtX, boxY); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(lbox, rWt, worldAnchorGround1, worldAnchorGround2, lbox->GetWorldCenter(), rWt->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

    }


    //THE UPPER PART
    //the right plank holding the piston and stack of balls
    float32 upY=33.0f,rplankLen=15,rplankX=10;
    fixedplank(m_world,rplankX,upY,rplankLen);



    //the crank shaft(the rotating part of piston system)
    float32 crankshaftRad=2,crankLen=2.1,crankWid=0.2,shift=2.0;//shift is the distance of crankshaft from the end of the right plank
    float32 crankshaftX=rplankX - rplankLen -crankshaftRad-shift;//at the left end of rplank
    float32 crankshaftY=upY+crankshaftRad-0.3;
    b2Body* crankshaft=ball(m_world,crankshaftX,crankshaftY,crankshaftRad,2);
    // b2Body* crankshaft=box(m_world,crankshaftX,crankshaftY,crankLen,crankWid,3);

    //fix it at its center
    b2Joint* crankjoint;
    {

      b2BodyDef dummybdef; //dummy body to act as another body of revolute joint
      dummybdef.position.Set(crankshaftX, crankshaftY);
      b2Body* dummybody = m_world->CreateBody(&dummybdef);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = crankshaft;
      jointDef.bodyB = dummybody;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      jointDef.maxMotorTorque = 1000.0f;
      jointDef.motorSpeed = 2.f;
      jointDef.enableMotor = true;
      // =(b2RevoluteJoint)m_world.CreateJoint(&jointDef) ;
      crankjoint=(m_world->CreateJoint(&jointDef));
    }

    //the rod connecting crankshaft with the piston
    float32 rodLen=3,rodHt=0.2;
    b2Body* rod=box(m_world,crankshaftX+crankshaftRad+rodLen,crankshaftY,rodLen,rodHt,0.5);

    //connect crank and rod
    b2RevoluteJointDef chainjoint1;
    chainjoint1.bodyA=crankshaft;
    chainjoint1.bodyB=rod;
    chainjoint1.localAnchorA.Set(crankshaftRad,0);
    chainjoint1.localAnchorB.Set(-rodLen,0);
    m_world->CreateJoint( &chainjoint1 );

    //piston
    float32 pistonLen=2,pistonHt=1;
    float32 pistonX=crankshaftX+crankshaftRad+2*rodLen+pistonLen;
    float32 pistonY=upY+pistonHt;
    b2Body* piston= box(m_world,pistonX,pistonY,pistonLen,pistonHt,0.5);

    //to keep piston horizontal add an cover edge above it
    float32 edgeLen=2;
    float32 edgeX2=pistonX-1.5,edgeX1=edgeX2-edgeLen,edgeY=pistonY+pistonHt+0.2;
    edge(m_world,edgeX1,edgeY,edgeX2,edgeY);

    //connect rod and piston
    b2RevoluteJointDef chainjoint2;
    chainjoint2.bodyA=rod;
    chainjoint2.bodyB=piston;
    chainjoint2.localAnchorA.Set(rodLen,0);
    chainjoint2.localAnchorB.Set(-pistonLen,0);
    m_world->CreateJoint( &chainjoint2 );

    //the tower holding balls
    float32 ballRad=1;
    float32 towerX1=pistonX-1.5,towerX2=towerX1+2*ballRad+0.05;
    float32 towerHt= 5*ballRad ;//half height to accomodate 5 balls of radius 'ballRad'
    float32 towerY1=upY+2*pistonHt+0.2,   towerY11=towerY1+1,   towerY12=towerY11+0.4,  towerY2=towerY12+ 2*towerHt;//to accomodate blocking plank (total width 0.3)in b/w
    edge(m_world, towerX1,towerY1,towerX1,towerY11); //the lower left wall
    edge(m_world, towerX2,towerY1,towerX2,towerY11); //the lower right wall
    edge(m_world, towerX1,towerY12,towerX1,towerY2); //the lower left wall
    edge(m_world, towerX2,towerY12,towerX2,towerY2); //the lower left wall

    //the balls
    float32 ballX=(towerX1+towerX2)/2;
    float32 ballstartY=towerY12+ballRad;
    for(int i=0;i<3;i++){
      ball(m_world,ballX,ballstartY+i*2*ballRad,ballRad,1.5);
    }
    //the plank blocking the balls initially
    b2Body* blockingplank=fixedplank(m_world,ballX,(towerY11+towerY12)/2,ballRad+0.5,true);
    //edges supporting the plank when pulled    
    edge(m_world,towerX1-3,towerY11-0.1,towerX1-0.5,towerY11-0.1); //lower edge
    edge(m_world,towerX1-3,towerY12,towerX1-0.5,towerY12); //upper edge

    //Now the pulley system to take out the blocking plank
      //the topmost plank
      float32 fixedplankX=crankshaftX-5,fixedplankY=crankshaftY+crankshaftRad+3,fixedplankLen=8;
      fixedplank(m_world,fixedplankX,fixedplankY,fixedplankLen);
      //the box on topmost plank
      float32 boxSize=1,boxY=fixedplankY+boxSize;
      b2Body* thebox= box(m_world,fixedplankX,boxY,boxSize,boxSize,2);

      //the right pulley joining box and blocking plank
      {
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorGround1(fixedplankX+fixedplankLen, boxY); //Ground Anchor point for the box
        b2Vec2 worldAnchorGround2(towerX1-3,(towerY11+towerY12)/2); //Ground Anchor point for the plank
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(thebox , blockingplank,  worldAnchorGround1, worldAnchorGround2, thebox->GetWorldCenter(), blockingplank->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
      }
      //the left open box
      b2Body* openbox;
      float32 openboxX=fixedplankX- fixedplankLen-2,openboxY=fixedplankY-5;
      {
        b2BodyDef *obDef = new b2BodyDef;
        obDef->type = b2_dynamicBody;
        obDef->position.Set(openboxX,openboxY);
        obDef->fixedRotation = true;

        b2FixtureDef fd;
        fd.density = 0.1;
        fd.friction = 0.5;
        fd.restitution = 0.f;
        b2PolygonShape shape;
        openbox = m_world->CreateBody(obDef);
    //   box1->CreateFixture(fd1);
        //left side
        shape.SetAsBox(2,0.1, b2Vec2(0.f,-1.f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
        //right side
        shape.SetAsBox(0.1,1, b2Vec2(1.9f,-0.f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
        //bottom side
        shape.SetAsBox(0.1,1, b2Vec2(-1.9f,-0.f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
      }

      //platform holding the openbox when it falls
      fixedplank(m_world,openboxX,openboxY-5,3);

    //the second pulley connecting openbox and the box on the topmost plank
      {
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorGround1(openboxX, boxY); //Common Ground Anchor point for the box
        b2Vec2 worldAnchorGround2(openboxX,boxY); //same as above anchor
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(thebox , openbox,  worldAnchorGround1, worldAnchorGround2, thebox->GetWorldCenter(), openbox->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
      }

      //THE MAZE
      {//starts at right end of the upper level
        //the first slope
        float32 mazeStartX=rplankX+rplankLen,  mazeStartY=upY;
        float32 mx1=mazeStartX+2,   my1=mazeStartY-0.5;
        edge(m_world,mazeStartX,mazeStartY,mx1,my1);
        float32 mx2=mx1,            my2=my1-2;
        edge(m_world,mx1,my1,mx2,my2);
        mx1=mx2+2*ballRad+0.1;        my1=my2;
        edge(m_world,mx2,my2,mx1,my1);
        mx2=mx1;                    my2=my1+2;
        edge(m_world,mx1,my1,mx2,my2);
        mx1=mx2+6;        my1=my2-1;
        edge(m_world,mx2,my2,mx1,my1);

        //the second slope-
        mx1=mx1+2.5;                    my1=my1+2;
        mx2=mx1;                      my2=my1-4;
        edge(m_world,mx1,my1,mx2,my2);
        mx1=mx2-3;        my1=my2-1;
        edge(m_world,mx2,my2,mx1,my1);

        //the turner
        b2Body* turner;
        float32 turnerX=mx1-2.5,turnerY=my1-1;
        float32 turnerLen1=1,turnerLen2=1.5;
        {
          b2BodyDef *obDef = new b2BodyDef;
          obDef->type = b2_dynamicBody;
          obDef->position.Set(turnerX,turnerY);
          obDef->angle=3.1415/6;
          b2FixtureDef fd;
          fd.density = 0.1;
          fd.friction = 0.5;
          fd.restitution = 0.f;
          b2PolygonShape shape;
          turner = m_world->CreateBody(obDef);
    //   box1->CreateFixture(fd1);
        //left side
          shape.SetAsBox(turnerLen1,0.1, b2Vec2(turnerLen1,0.f), 0);
          fd.shape=&shape;
          turner->CreateFixture(&fd);

          shape.SetAsBox(0.1,turnerLen2, b2Vec2(0,turnerLen2), 0);
          fd.shape=&shape;
          turner->CreateFixture(&fd);
        }
        //pivoting the turner
        {
          b2BodyDef dummybdef; //dummy body to act as another body of revolute joint
          dummybdef.position.Set(turnerX, turnerY);
          b2Body* dummybody = m_world->CreateBody(&dummybdef);

          b2RevoluteJointDef jointDef;
          jointDef.bodyA = turner;
          jointDef.bodyB = dummybody;
          jointDef.localAnchorA.Set(0,0);
          jointDef.localAnchorB.Set(0,0);
          (m_world->CreateJoint(&jointDef));
        }
        //adding small nail to prevent turning in wrong direction
        fixedplank(m_world,turnerX-1.5,turnerY+2.5,0.2);



      }

      //THE PENDULUM SYSTEM TRIGGERING THE MACHINE
      //the ballholder
      float32 ballholderLen=4;
      float32 ballholderX=openboxX-3-ballholderLen,ballholderY=openboxY+5;
      fixedplank(m_world,ballholderX,ballholderY,ballholderLen);

      //the ballwhich will fall in openbox
      float32 triggerballRad=0.7;
      float32 triggerballX=ballholderX,triggerballY=ballholderY+triggerballRad;
      ball(m_world,triggerballX,triggerballY,triggerballRad,10);


      //the pendulum bobs
      float32 bobX=triggerballX-1.6,bobY=triggerballY+0.5,bobRad=0.8;
      float32 stringLen=5;
      for(int i=0;i<4;i++){
        float32 x=bobX-i*2*(bobRad);
        b2Body* bob;
        float32 y=bobY;
        if(i==3){
          bob= ball(m_world, x- stringLen*(0.5) , y+(stringLen-stringLen*(1.732/2)),bobRad,10);//sin 30 and cos 30
        }
        else bob= ball(m_world,x,y,bobRad,3);
        //suspend the pendulum
        b2BodyDef bdef;
        bdef.position.Set(x,bobY+stringLen);
        b2Body* dummy=m_world->CreateBody(&bdef);

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(x,bobY+stringLen);
        jd.Initialize(bob, dummy, anchor);
        m_world->CreateJoint(&jd);
      }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
