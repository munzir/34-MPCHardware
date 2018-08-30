#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <config4cpp/Configuration.h>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace config4cpp;

// mutexes to ensure that the main and sim loops alternate in sequence: main1->sim1->main2->sim2->...
pthread_mutex_t simSync_mutex1;
pthread_mutex_t simSync_mutex2;


class MyWindow : public dart::gui::SimWindow {
  
  public: 
    MyWindow(const WorldPtr& world) {

      // Attach the world passed in the input argument to the window, and fetch the robot from the world
      setWorld(world);
      mkrang = world->getSkeleton("krang");
      int joints = mkrang->getNumJoints();
      for(int i=3; i < joints; i++) {
        mkrang->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
      }
    }

    void timeStepping() override;

    ~MyWindow() {}

  protected:

    /// Full robot and low level controller
    SkeletonPtr mkrang;
};

//====================================================================
void MyWindow::timeStepping() {

  pthread_mutex_lock(&simSync_mutex1);
  pthread_mutex_unlock(&simSync_mutex1);

  SimWindow::timeStepping();

  pthread_mutex_unlock(&simSync_mutex1);
  pthread_mutex_lock(&simSync_mutex1);  
}

//====================================================================
SkeletonPtr createFloor() {
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  //  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//====================================================================
dart::dynamics::SkeletonPtr createTray(dart::dynamics::BodyNodePtr ee) {

  Eigen::Matrix3d EELRot, trayLocalRot, trayRot;
  Eigen::Vector3d EELPos, trayLocalTranslation, trayPos;
  Eigen::Matrix<double, 6, 1> qObject;
  
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr tray =
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/scenes/tray.urdf");
  tray->setName("tray");

  // Orientation
  EELRot = ee->getTransform().rotation();
  trayLocalRot << 1,  0, 0,
                  0,  0, 1,
                  0, -1, 0;
  trayRot = EELRot*trayLocalRot;
  Eigen::AngleAxisd aa(trayRot);
  
  // Position
  EELPos = ee->getTransform().translation();
  trayLocalTranslation << 0, -0.286303, 0.04;
  trayPos = EELPos + trayRot*trayLocalTranslation;

  // Set the position
  qObject << (aa.angle()*aa.axis()), trayPos;
  tray->setPositions(qObject);

  return tray;
}

//====================================================================
dart::dynamics::SkeletonPtr createCup(dart::dynamics::BodyNodePtr ee) {
  
  Eigen::Matrix3d EELRot, cupLocalRot, cupRot;
  Eigen::Vector3d EELPos, cupLocalTranslation, cupPos;
  Eigen::Matrix<double, 6, 1> qObject;
  
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr cup =
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/scenes/cup.urdf");
  cup->setName("cup");

  // Orientation
  EELRot = ee->getTransform().rotation();
  cupLocalRot << 1,  0, 0,
                 0,  0, 1,
                 0, -1, 0;
  cupRot = EELRot*cupLocalRot;
  Eigen::AngleAxisd aa(cupRot);
  
  // Position
  EELPos = ee->getTransform().translation();
  cupLocalTranslation << 0, -0.286303, 0.04;
  cupPos = EELPos + cupRot*cupLocalTranslation;

  // Set the position
  qObject << (aa.angle()*aa.axis()), cupPos;
  cup->setPositions(qObject);

  return cup;
}

//==================================================================
struct simArguments {
  WorldPtr world;     
  SkeletonPtr robot;
};

//====================================================================
void *simfunc(void *arg) {

  pthread_mutex_lock(&simSync_mutex2);

  struct simArguments *simArg = arg;

  // Create world
  WorldPtr world = simArg->world;

  // Load Floor
  SkeletonPtr floor = createFloor();
  world->addSkeleton(floor); //add ground and robot to the world pointer

  // Load robot
  SkeletonPtr robot = simArg->robot;
  // world->addSkeleton(robot);
  
  // To load tray and cup or not
  bool loadTray, loadCup; double trayCupFriction;
  Configuration *  cfg = Configuration::create();
  const char *     scope = "";
  const char *     configFile = "/home/munzir/project/krang/28-balance-kore/balancing/src/controlParams.cfg";
  try {
    cfg->parse(configFile);
    loadTray = cfg->lookupBoolean(scope, "tray"); 
    loadCup = cfg->lookupBoolean(scope, "cup"); 
    trayCupFriction = cfg->lookupFloat(scope, "trayCupFriction");
  } catch(const ConfigurationException & ex) {
      cerr << ex.c_str() << endl;
      cfg->destroy();
  }
  cout << "loadTray: " << (loadTray?"true":"false") << endl;
  cout << "loadCup: " << (loadCup?"true":"false") << endl;
  cout << "trayCupFriction: " << trayCupFriction << endl;
  
  // Load Tray
  if(loadTray) {
    SkeletonPtr tray = createTray(robot->getBodyNode("lGripper"));
    world->addSkeleton(tray);
    tray->getBodyNode(0)->setFrictionCoeff(trayCupFriction);
    cout << "tray surface friction: " << tray->getBodyNode(0)->getFrictionCoeff() << endl;
  }
  
  // Load Cup
  if(loadCup) { 
    SkeletonPtr cup = createCup(robot->getBodyNode("lGripper")); //cup->setPositions(tray->getPositions());
    world->addSkeleton(cup);
    cup->getBodyNode(0)->setFrictionCoeff(trayCupFriction);
    cout << "cup surface friction: " << cup->getBodyNode(0)->getFrictionCoeff() << endl;
  }
  
  // Create window
  MyWindow window(world);

  // Run the world
  glutInit(&argc, argv);
  window.initWindow(1280,720, "3DOF URDF");
  glutMainLoop();

  return 0;
}