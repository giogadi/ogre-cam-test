// Example program:
// Using SDL2 to create an application window

#include <SDL.h>
#include <SDL_syswm.h>
#include <iostream>

#include <OgreRoot.h>
#include <OgreManualObject.h>
#include <OgreRenderWindow.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OgreEntity.h>

#include <opencv2/opencv.hpp>

#include "OgreBackground.h"
#include "OgreCVCamera.h"

void initScene(const std::string& sceneMgrName, Ogre::Camera* camera, OgreCV::OgreBackground* bg, const OgreCV::WebcamProperties& camProps)
{
  Ogre::RenderTarget* renderTarget = Ogre::Root::getSingleton().getRenderTarget("OGRE Window");
  renderTarget->addViewport(camera);

  Ogre::SceneManager* sceneMgr = Ogre::Root::getSingleton().getSceneManager(sceneMgrName);

  // Add background rectangle to scene
  Ogre::SceneNode* node = sceneMgr->getRootSceneNode()->createChildSceneNode("Background");
  node->attachObject(bg->getRect());

  Ogre::MaterialPtr boxMaterial = Ogre::MaterialManager::getSingleton().create("box_material",
                                                                               Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  boxMaterial->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  boxMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  boxMaterial->getTechnique(0)->getPass(0)->setDiffuse(0.5, 0.5, 0.5, 0.5);
  boxMaterial->getTechnique(0)->getPass(0)->setAmbient(0.2, 0.2, 0.2);

  // Add prop cube to scene
  Ogre::Entity* cube = sceneMgr->createEntity("Cube", Ogre::SceneManager::PT_CUBE);
  cube->setMaterial(boxMaterial);
  node = sceneMgr->getRootSceneNode()->createChildSceneNode("Cube", Ogre::Vector3::ZERO);
  node->attachObject(cube);
  float offset = camProps.squareSize / 2.0;
  node->scale(0.01f * camProps.squareSize * Ogre::Vector3::UNIT_SCALE);
  node->translate(offset, offset, -offset);
  //node->translate(offset + 6*bg.properties.squareSize, offset + 4*bg.properties.squareSize, -offset);
  //node->translate(offset, 0.195f - offset, -0.23f + offset);
  //node->scale(0.01f * 0.03f * Ogre::Vector3::UNIT_SCALE);

  // Lights
  sceneMgr->setAmbientLight(Ogre::ColourValue(0.1f, 0.1f, 0.1f, 1.0f));

  Ogre::Light* light = sceneMgr->createLight("MainLight");
  light->setType(Ogre::Light::LightTypes::LT_POINT);
  light->setPosition(1.0f, 1.0f, 1.0f);
  light->setDiffuseColour(1.0, 1.0, 1.0);

  light = sceneMgr->createLight("MainLight2");
  light->setType(Ogre::Light::LightTypes::LT_POINT);
  light->setPosition(-1.0f, 0.0f, -1.0f);
  light->setDiffuseColour(1.0f, 1.0f, 1.0f);
}

void updateCameraPositionFromChessboard(const cv::Mat& undistortedImg, const OgreCV::WebcamProperties& camProps, Ogre::Camera* cam)
{
  std::vector<cv::Point2f> imgPoints;
  bool chessboardFound = cv::findChessboardCorners(undistortedImg, camProps.boardSize, imgPoints,
                                                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
  if (chessboardFound)
  {
    // Improve accuracy
    cv::Mat viewGray;
    cv::cvtColor(undistortedImg, viewGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(viewGray, imgPoints, cv::Size(11,11),
                     cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

    // Generate positions of chessboard corners with top-left corner at (0,0,0)
    std::vector<cv::Point3d> worldPoints;
    for (int r = 0; r < camProps.boardSize.height; ++r)
      for (int c = 0; c < camProps.boardSize.width; ++c)
        worldPoints.push_back(cv::Point3d(c*camProps.squareSize, r*camProps.squareSize, 0.0));

    // Perform pose estimation (origin w.r.t. webcam)
    cv::Mat w_R_o;
    cv::Mat w_p_o;
    cv::solvePnP(worldPoints, imgPoints, camProps.cameraMat, cv::Mat::zeros(4,1,CV_32F),
                 w_R_o, w_p_o, false, cv::EPNP);

    OgreCV::updateOgreCameraFromCVExtrinsics(w_R_o, w_p_o, cam);
  }
}

int main(int argc, char** argv)
{
  const int WINDOW_WIDTH = 640;
  const int WINDOW_HEIGHT = 480;

  Ogre::Root* root = new Ogre::Root("", "", "");
	root->loadPlugin (OGRE_PLUGIN_DIR_REL + std::string("/RenderSystem_GL"));
	root->setRenderSystem(root->getRenderSystemByName("OpenGL Rendering Subsystem"));

	root->initialise(false);

  SDL_Init(SDL_INIT_VIDEO);   // Initialize SDL2
  
  SDL_Window *window;
  
  // Create an application window with the following settings:
  window = SDL_CreateWindow( 
    "An SDL2 window",                  //    window title
    SDL_WINDOWPOS_UNDEFINED,           //    initial x position
    SDL_WINDOWPOS_UNDEFINED,           //    initial y position
    WINDOW_WIDTH,                               //    width, in pixels
    WINDOW_HEIGHT,                               //    height, in pixels
    SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN //    flags - see below
  );
  
  // Check that the window was successfully made
  if(window==NULL){   
    // In the event that the window could not be made...
    std::cout << "Could not create window: " << SDL_GetError() << '\n';
    return 1;
  }

  SDL_GLContext glContext = SDL_GL_CreateContext(window);
  if (glContext == NULL)
  {
    std::cout << "ERROR: SDL_GL_CreateContext failed!" << std::endl;
    return 1;
  }

	//get the native whnd
	struct SDL_SysWMinfo wmInfo;
	SDL_VERSION(&wmInfo.version);

	if (SDL_GetWindowWMInfo(window, &wmInfo) == SDL_FALSE)
  {
    std::cout << "ERROR: Couldn't get WM info!" << std::endl;
      return 1;
  }

  Ogre::NameValuePairList params;
  params["externalGLControl"] = "1";
#ifdef __WINDOWS__
  params["externalWindowHandle"] = Ogre::StringConverter::toString((unsigned long) wmInfo.info.win.window);
  params["externalGLContext"] = Ogre::StringConverter::toString((unsigned long) glContext);
#else
  params["currentGLContext"] = "1";
  params["parentWindowHandle"] = Ogre::StringConverter::toString((unsigned long) wmInfo.info.x11.window);
#endif

  params["title"] = "Ogre Window";
  params["FSAA"] = "0";
  params["vsync"] = "false";

	Ogre::RenderWindow* ogreWindow = Ogre::Root::getSingleton().createRenderWindow("OGRE Window", WINDOW_WIDTH, WINDOW_HEIGHT, false, &params);

  SDL_GL_SetSwapInterval(1);

	ogreWindow->setVisible(true);
	
  OgreCV::WebcamProperties camProps("out_camera_data.xml");
  OgreCV::OgreBackground bg(camProps.w, camProps.h);

  std::string sceneMgrName("SceneMgr");
  Ogre::Root::getSingleton().createSceneManager(Ogre::ST_EXTERIOR_CLOSE, sceneMgrName);
  Ogre::Camera* cam = camProps.generateOgreCamera("camera", sceneMgrName);
  initScene(sceneMgrName, cam, &bg, camProps);

  cv::VideoCapture webcam(0);
  if (!webcam.isOpened())
    throw std::runtime_error("Error in opening webcam!");

	webcam.set(CV_CAP_PROP_FRAME_WIDTH, camProps.w);
	webcam.set(CV_CAP_PROP_FRAME_HEIGHT, camProps.h);

  cv::Mat webcamImg;
  cv::Mat undistortedImg;

  bool done = false;
  while (!done)
  {
    webcam >> webcamImg;
    cv::undistort(webcamImg, undistortedImg, camProps.cameraMat, camProps.distCoeffs);
    bg.updateFromCVImg(undistortedImg);

    updateCameraPositionFromChessboard(undistortedImg, camProps, cam);
	  root->renderOneFrame();
    SDL_GL_SwapWindow(window);
	
    SDL_Event event;
    if(SDL_PollEvent(&event))
    {
		  if (event.type == SDL_QUIT)
			  done = true;
    }
  }

	delete root;

  // Close and destroy the window
  SDL_DestroyWindow(window); 
   
  // Clean up
  SDL_Quit();

  return 0;   
}
