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

struct CameraProperties
{
  int w;
  int h;
  cv::Mat cameraMat;
  cv::Mat distCoeffs;
  cv::Size boardSize;
  double squareSize;
};

struct WebcamBackground
{
  cv::VideoCapture* webcam;
  CameraProperties properties;
  Ogre::TexturePtr tex;
  Ogre::MaterialPtr mat;
  Ogre::Rectangle2D* rect;
};

CameraProperties loadCameraProperties(const std::string& cameraFilename)
{
  CameraProperties prop;

  // Load in relevant data from file
  cv::FileStorage fs(cameraFilename, cv::FileStorage::READ);
  if (!fs.isOpened())
    throw std::runtime_error("Error in opening camera file!");
  fs["image_Width"] >> prop.w;
  fs["image_Height"] >> prop.h;
  fs["Camera_Matrix"] >> prop.cameraMat;
  fs["Distortion_Coefficients"] >> prop.distCoeffs;
  fs["board_Width"] >> prop.boardSize.width;
  fs["board_Height"] >> prop.boardSize.height;
  fs["square_Size"] >> prop.squareSize;
  fs.release();

  return prop;
}

WebcamBackground createWebcamBackground(int camHandle, const CameraProperties& camProperties)
{
  cv::VideoCapture* webcam = new cv::VideoCapture(camHandle);
  if (!webcam->isOpened())
    throw std::runtime_error("Error in opening webcam!");

	webcam->set(CV_CAP_PROP_FRAME_WIDTH, camProperties.w);
	webcam->set(CV_CAP_PROP_FRAME_HEIGHT, camProperties.h);

  // Create the texture
	Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().createManual(
		"WebcamTexture", // name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,      // type
		camProperties.w, camProperties.h,         // width & height
		0,                // number of mipmaps
		Ogre::PF_BYTE_BGRA,     // pixel format
		Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
												// textures updated very often (e.g. each frame)

  // ---------------------- Zero-out background texture ---------------
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = tex->getBuffer();
	pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!
	const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
  for (int i = 0; i < camProperties.h; ++i)
    for (int j = 0; j < camProperties.w; ++j)
      for (int k = 0; k < 4; ++k)
        *pDest++ = 0;
  pixelBuffer->unlock();

	// Create a material using the texture
	Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
		"WebcamBackgroundMaterial", // name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
 
	mat->getTechnique(0)->getPass(0)->createTextureUnitState("WebcamTexture");
	mat->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
	mat->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
	mat->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
	rect->setCorners(-1.0, 1.0, 1.0, -1.0);
	rect->setMaterial("WebcamBackgroundMaterial");
 
	// Render the background before everything else
	rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
 
	// Use infinite AAB to always stay visible
	Ogre::AxisAlignedBox aabInf;
	aabInf.setInfinite();
	rect->setBoundingBox(aabInf);

  WebcamBackground bg = {webcam, camProperties, tex, mat, rect};
  return bg;
}

Ogre::Camera* initScene(const WebcamBackground& bg)
{
  Ogre::SceneManager* sceneMgr = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_EXTERIOR_CLOSE, "SceneMgr");

  // Create a camera with default pose (to be changed using pose estimation)
  // TODO set the FOVY according to data read from camera file
  Ogre::Camera* camera = sceneMgr->createCamera("cam");
  camera->setPosition(1.0, 0.0, 0.0);
  camera->lookAt(0.0, 0.0, 0.0);
  camera->setNearClipDistance(0.001f);
  camera->setFOVy(Ogre::Radian(2.0*atan(0.5 * bg.properties.h / bg.properties.cameraMat.at<double>(1,1))));

  Ogre::RenderTarget* renderTarget = Ogre::Root::getSingleton().getRenderTarget("OGRE Window");
  renderTarget->addViewport(camera);

  // Add background rectangle to scene
  Ogre::SceneNode* node = sceneMgr->getRootSceneNode()->createChildSceneNode("Background");
  node->attachObject(bg.rect);

  // Add prop cube to scene
  Ogre::Entity* cube = sceneMgr->createEntity("Cube", Ogre::SceneManager::PT_CUBE);
  node = sceneMgr->getRootSceneNode()->createChildSceneNode("Cube", Ogre::Vector3::ZERO);
  node->attachObject(cube);
  float offset = bg.properties.squareSize / 2.0;
  /*node->scale(0.01f * bg.properties.squareSize * Ogre::Vector3::UNIT_SCALE);
  node->translate(offset + 6*bg.properties.squareSize, offset + 4*bg.properties.squareSize, -offset);*/
  node->translate(offset, 0.18f - offset, -0.13f + offset);
  node->scale(0.01f * 0.03f * Ogre::Vector3::UNIT_SCALE);

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

  return camera;
}

void updateBackgroundFromWebcam(WebcamBackground* bg, cv::Mat* undistortedImg)
{
  cv::Mat rawImg;
  (*bg->webcam) >> rawImg;

  cv::undistort(rawImg, *undistortedImg, bg->properties.cameraMat, bg->properties.distCoeffs);
  /**undistortedImg = rawImg;*/
  
	Ogre::HardwarePixelBufferSharedPtr pixelBuffer = bg->tex->getBuffer();
	pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!
	const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
	Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	Ogre::uint8* pSrc = static_cast<Ogre::uint8*>(undistortedImg->data);
	for (size_t r = 0; r < bg->properties.h; ++r)
	{
    for (size_t c = 0; c < bg->properties.w; ++c)
    {
			*pDest++ = *pSrc++; // B
			*pDest++ = *pSrc++; // G
			*pDest++ = *pSrc++; // R
			*pDest++ = 255; // A
		} 
		pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
	}
	pixelBuffer->unlock();
}

void updateCameraPositionFromChessboard(const WebcamBackground& bg, const cv::Mat& img, Ogre::Camera* cam)
{
  std::vector<cv::Point2f> imgPoints;
  bool chessboardFound = cv::findChessboardCorners(img, bg.properties.boardSize, imgPoints,
                                                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
  if (chessboardFound)
  {
    // Improve accuracy
    cv::Mat viewGray;
    cv::cvtColor(img, viewGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(viewGray, imgPoints, cv::Size(11,11),
                     cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

    // Generate positions of chessboard corners with top-left corner at (0,0,0)
    std::vector<cv::Point3d> worldPoints;
    for (int r = 0; r < bg.properties.boardSize.height; ++r)
      for (int c = 0; c < bg.properties.boardSize.width; ++c)
        worldPoints.push_back(cv::Point3d(c*bg.properties.squareSize, r*bg.properties.squareSize, 0.0));

    // Perform pose estimation (origin w.r.t. webcam)
    cv::Mat w_R_o;
    cv::Mat w_p_o;
    cv::solvePnP(worldPoints, imgPoints, bg.properties.cameraMat, bg.properties.distCoeffs,
                 w_R_o, w_p_o, false, cv::EPNP);

    // Get the inverse transformation (webcam w.r.t. origin)
    cv::Mat o_R_w;
    cv::Rodrigues(w_R_o, o_R_w);
    o_R_w = o_R_w.t();
    cv::Mat o_p_w = -o_R_w*w_p_o;

    // convert to Ogre transformation matrix
    Ogre::Matrix4 o_T_w = Ogre::Matrix4::IDENTITY;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        o_T_w[i][j] = (Ogre::Real) o_R_w.at<double>(i,j);
    for (int i = 0; i < 3; ++i)
      o_T_w[i][3] = (Ogre::Real) o_p_w.at<double>(i,0);

    // Transformation matrix for Ogre camera w.r.t. webcam
    // Just a 180 degree rotation about x-axis
    Ogre::Matrix4 w_T_c = Ogre::Matrix4::IDENTITY;
    w_T_c[1][1] = -1.0;
    w_T_c[2][2] = -1.0;

    // Transformation matrix for Ogre camera w.r.t. origin
    Ogre::Matrix4 o_T_c = o_T_w * w_T_c;

    std::cout << o_T_c << std::endl;

    cam->setOrientation(o_T_c.extractQuaternion());
    cam->setPosition(o_T_c.getTrans());
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
	
  CameraProperties camProps;
  WebcamBackground bg;
  Ogre::Camera* camera;
  try
  {
    camProps = loadCameraProperties("cam.xml");
    bg = createWebcamBackground(0, camProps);
    camera = initScene(bg);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }
  cv::Mat webcamImg;

  bool done = false;
  while (!done)
  {
    updateBackgroundFromWebcam(&bg, &webcamImg);
    updateCameraPositionFromChessboard(bg, webcamImg, camera);
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
  delete bg.webcam;

  // Close and destroy the window
  SDL_DestroyWindow(window); 
   
  // Clean up
  SDL_Quit();

  return 0;   
}
