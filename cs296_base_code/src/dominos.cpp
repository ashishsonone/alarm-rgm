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



b2Body* fixedplank(b2World* w,float32 x,float32 y,float32 l,bool isDyn=false,float32 angle=0.0,float32 b=0.15f){
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

b2Body* domino(b2World* w,float32 x,float32 y,float32 h=1.5f,float32 b=0.15f){
  b2Body *domino; //declare body
  b2BodyDef bdef;//declare bodydef

  bdef.type=b2_dynamicBody;
  bdef.position.Set(x,y); //set position
  b2PolygonShape shape; //declare shape
  shape.SetAsBox(b, h); //set shape to be box of length l

  b2FixtureDef fd; //fixture definition
  fd.shape = &shape;
  fd.density = 2.0f;
  fd.friction = 0.1f;


  domino = w->CreateBody(&bdef); //create body using body definition
  domino->CreateFixture(&fd); //density for static body is not used(as static body has zero density by default)
  
  return domino;
}

b2Body* edge(b2World* w,float32 x1,float32 y1,float32 x2,float32 y2){
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

b2Body* ball(b2World*w,float32 x,float32 y,float32 rad,float32 density=10.0){
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

b2Body* box(b2World*w,float32 x,float32 y,float32 len,float32 wid,float32 density,float32 friction=0.5f){
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





namespace cs296
{
  

  dominos_t::dominos_t()
  {
    //Ground
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }


    //MY CODE STARTS HERE

    //make dominoholder
    float32 domHY=25.0f,domHCenterX=15.0f,domHLen=25.0f;
    float32 domHX1 = domHCenterX - domHLen;

    b2Body* dominosholder=fixedplank(m_world,domHCenterX,domHY,domHLen);

    //make dominos on the holder
    {
      float32 DOMY=26.5f,STARTX=-5.0f,GAP=1.5f;
      for(int i=0;i<8;i++){
        domino(m_world,STARTX+i*GAP,DOMY);
      }
    }

    //put a ball at end of dominos train
    float32 radius=0.8;
    ball(m_world,-7.0f,domHY+radius,radius);

    //ball making dominos fall as for now
    ball(m_world,6,domHY+3,1);
    

    //the falling maze
    float32 mazeHeight=10,mazegap=2.5;
    edge(m_world,(domHX1-4),domHY+1,(domHX1+0),domHY-mazegap);
    edge(m_world,(domHX1+3),domHY-mazegap,(domHX1+0),domHY-2*mazegap);
    edge(m_world,(domHX1-4),domHY-2*mazegap,(domHX1+2),domHY-3*mazegap-1);

    //anvil holder
    float32 anvHLen=5; //anvil holder length
    fixedplank(m_world,domHX1,domHY-mazeHeight,anvHLen);

    //the anvil
    float32 anvLen=1.0f;
    float32 anvilDensity=30.0f;
    box(m_world,domHX1+anvHLen,domHY-mazeHeight+1,anvLen,anvLen,anvilDensity); 



    //The see-saw system at the bottom
    {
      //The triangle wedge
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
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;

      wedgebd.position.Set(wedX, wedY);

      wedge = m_world->CreateBody(&wedgebd);
      wedge->CreateFixture(&wedgefd);

      //create and put the plank on the triangle wedge
      b2Body* plank = fixedplank(m_world,wedX,wedY+wedHeight,5,true);

      //join the two components
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(wedX, wedY+wedHeight);
      jd.Initialize(wedge, plank, anchor);
      m_world->CreateJoint(&jd);

      //put a box on the left end of the see-saw
      float32 botDensity=0.9f,botSize=0.5f;
      b2Body *bot=box(m_world,wedX-3.7,wedY+wedHeight+botSize,botSize,botSize, botDensity,2);

      //also a box to hold plank straight until anvil falls (makes sure bot goes in right path jump)
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
      jointDef.maxMotorTorque = 100.0f;
      jointDef.motorSpeed = 10.0f;
      jointDef.enableMotor = true;
      // =(b2RevoluteJoint)m_world.CreateJoint(&jointDef) ;
      crankjoint=(m_world->CreateJoint(&jointDef));
    }

/***
    //the driver Motor
    float32 driverRad=crankshaftRad;
    float32 driverX=crankshaftX- crankshaftRad - driverRad,driverY=crankshaftY;
    b2Body* driver=ball(m_world, driverX,driverY,driverRad);

    //fix it at its center
    b2Joint* driverjoint;
    {

      b2BodyDef dummybdef; //dummy body to act as another body of revolute joint
      dummybdef.position.Set(driverX, driverY);
      b2Body* dummybody = m_world->CreateBody(&dummybdef);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = driver;
      jointDef.bodyB = dummybody;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      jointDef.maxMotorTorque = 100.0f;
      jointDef.motorSpeed = 10.0f;
      jointDef.enableMotor = true;
      // =(b2RevoluteJoint)m_world.CreateJoint(&jointDef) ;
      driverjoint=(m_world->CreateJoint(&jointDef));
    }

    //the gear join connecting driver motor and crankshaft
    {
      b2GearJointDef gear_joint ;
      gear_joint.bodyA=driver;
      gear_joint.bodyB=crankshaft;
      gear_joint.joint1=driverjoint;
      gear_joint.joint2=crankjoint;
      gear_joint.ratio=2;
      gear_joint.collideConnected = true;
      m_world->CreateJoint(&gear_joint);

    }
***/

    //the rod
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
    float32 towerX1=pistonX-1.5,towerX2=towerX1+2*ballRad+0.1;
    float32 towerHt= 5*ballRad ;//half height to accomodate 5 balls of radius 'ballRad'
    float32 towerY1=upY+2*pistonHt+0.2,   towerY11=towerY1+1,   towerY12=towerY11+0.4,  towerY2=towerY12+ 2*towerHt;//to accomodate blocking plank (total width 0.3)in b/w
    edge(m_world, towerX1,towerY1,towerX1,towerY11); //the lower left wall
    edge(m_world, towerX2,towerY1,towerX2,towerY11); //the lower right wall
    edge(m_world, towerX1,towerY12,towerX1,towerY2); //the lower left wall
    edge(m_world, towerX2,towerY12,towerX2,towerY2); //the lower left wall

    //the balls
    float32 ballX=(towerX1+towerX2)/2;
    float32 ballstartY=towerY12+ballRad;
    for(int i=0;i<4;i++){
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
      float32 openboxX=fixedplankX- fixedplankLen-1,openboxY=fixedplankY-8;
      {
        b2BodyDef *obDef = new b2BodyDef;
        obDef->type = b2_dynamicBody;
        obDef->position.Set(openboxX,openboxY);

        b2FixtureDef fd;
        fd.density = 5;
        fd.friction = 0.5;
        fd.restitution = 0.f;
        b2PolygonShape shape;
        openbox = m_world->CreateBody(obDef);
    //   box1->CreateFixture(fd1);
        //left side
        shape.SetAsBox(2,0.1, b2Vec2(0.f,-1.9f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
        //right side
        shape.SetAsBox(0.1,2, b2Vec2(1.9f,0.f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
        //bottom side
        shape.SetAsBox(0.1,2, b2Vec2(-1.9f,0.f), 0);
        fd.shape=&shape;
        openbox->CreateFixture(&fd);
      }

    //the second pulley connecting openbox and the box on the topmost plank
      {
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorGround1(openboxX, boxY); //Common Ground Anchor point for the box
        b2Vec2 worldAnchorGround2(openboxX,boxY); //same as above anchor
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(thebox , openbox,  worldAnchorGround1, worldAnchorGround2, thebox->GetWorldCenter(), openbox->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
      }

      box(m_world, openboxX,openboxY,1,1,500);

      //THE MAZE
      //starts 





    

      
    //Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      
    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(4.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    //The train of small spheres
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  //spherebody->CreateFixture(&ballfd);
	}
    }

    //The pulley system
    // {
    //   b2BodyDef *bd = new b2BodyDef;
    //   bd->type = b2_dynamicBody;
    //   bd->position.Set(-10,15);
    //   bd->fixedRotation = true;
      
    //   //The open box
    //   b2FixtureDef *fd1 = new b2FixtureDef;
    //   fd1->density = 10.0;
    //   fd1->friction = 0.5;
    //   fd1->restitution = 0.f;
    //   fd1->shape = new b2PolygonShape;
    //   b2PolygonShape bs1;
    //   bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
    //   fd1->shape = &bs1;
    //   b2FixtureDef *fd2 = new b2FixtureDef;
    //   fd2->density = 10.0;
    //   fd2->friction = 0.5;
    //   fd2->restitution = 0.f;
    //   fd2->shape = new b2PolygonShape;
    //   b2PolygonShape bs2;
    //   bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
    //   fd2->shape = &bs2;
    //   b2FixtureDef *fd3 = new b2FixtureDef;
    //   fd3->density = 10.0;
    //   fd3->friction = 0.5;
    //   fd3->restitution = 0.f;
    //   fd3->shape = new b2PolygonShape;
    //   b2PolygonShape bs3;
    //   bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
    //   fd3->shape = &bs3;
       
    //   b2Body* box1 = m_world->CreateBody(bd);
    //   box1->CreateFixture(fd1);
    //   box1->CreateFixture(fd2);
    //   box1->CreateFixture(fd3);

    //   //The bar
    //   bd->position.Set(10,15);	
    //   fd1->density = 34.0;	  
    //   b2Body* box2 = m_world->CreateBody(bd);
    //   box2->CreateFixture(fd1);

    //   // The pulley joint
    //   b2PulleyJointDef* myjoint = new b2PulleyJointDef();
    //   b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
    //   b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
    //   b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
    //   b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
    //   float32 ratio = 1.0f; // Define ratio
    //   myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
    //   m_world->CreateJoint(myjoint);
    // }

    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      // body->CreateFixture(fd);

      // b2PolygonShape shape2;
      // shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      jointDef.maxMotorTorque = 100.0f;
      jointDef.motorSpeed = 23.0f;
      jointDef.enableMotor = true;

      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      // sbody->CreateFixture(&ballfd);
    }
    
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
