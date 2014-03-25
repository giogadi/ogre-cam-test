namespace OgreCV
{
  enum CheckerDirection { PLUS_X, MINUS_X, PLUS_Y, MINUS_Y, PLUS_Z, MINUS_Z };

  void updateOgreCameraFromCVExtrinsics(const cv::Mat& w_R_o, const cv::Mat& w_p_o, Ogre::Camera* camera);

  struct WebcamProperties
  {
    int w;
    int h;
    cv::Mat cameraMat;
    cv::Mat distCoeffs;
    cv::Size boardSize;
    double squareSize;

    WebcamProperties(const std::string& camPropsFilename)
    {
      // Load in relevant data from file
      cv::FileStorage fs(camPropsFilename, cv::FileStorage::READ);
      if (!fs.isOpened())
        throw std::runtime_error("Error in opening camera file!");
      fs["image_width"] >> this->w;
      fs["image_height"] >> this->h;
      fs["camera_matrix"] >> this->cameraMat;
      fs["distortion_coefficients"] >> this->distCoeffs;
      fs["board_width"] >> this->boardSize.width;
      fs["board_height"] >> this->boardSize.height;
      fs["square_size"] >> this->squareSize;
      fs.release();
    }

    // Returns a Ogre::Camera with projection properties corresponding to this WebcamProperties object.
    // NOTE: this Ogre::Camera does NOT have an initialized position or orientation!
    Ogre::Camera* generateOgreCamera(const Ogre::String& cameraName, const Ogre::String& sceneMgrName) const
    {
      Ogre::SceneManager* sceneMgr = Ogre::Root::getSingleton().getSceneManager(sceneMgrName);
      Ogre::Camera* camera = sceneMgr->createCamera(cameraName);
    
      // Generate projection matrix
      // http://spottrlabs.blogspot.com/2012/07/opencv-and-opengl-not-always-friends.html
      double A00 = this->cameraMat.at<double>(0,0);
      double A11 = this->cameraMat.at<double>(1,1);
      double A02 = this->cameraMat.at<double>(0,2);
      double A12 = this->cameraMat.at<double>(1,2);
      double f = 10.0; // far clip distance
      double n = 0.001; // near clip distance
      Ogre::Matrix4 PM(2*A00/w, 0.0, -1+(2*A02/w), 0,
                       0, 2*A11/h, -1+(2*A12/h), 0,
                       0, 0, -(f+n)/(f-n), -2*f*n/(f-n),
                       0,0,-1,0);
      camera->setCustomProjectionMatrix(true, PM);

      return camera;
    }

    void undistort(cv::InputArray src, cv::OutputArray dst) const
    {
      cv::undistort(src, dst, this->cameraMat, this->distCoeffs);
    }

    void generate3DCheckerboard(const cv::Point3d& startPoint, 
                                CheckerDirection rowDirection, CheckerDirection colDirection,
                                std::vector<cv::Point3d>* worldPoints) const
    {
      worldPoints->clear();
      for (int r = 0; r < this->boardSize.height; ++r)
        for (int c = 0; c < this->boardSize.width; ++c)
        {
          cv::Point3d p(startPoint);
          switch (colDirection)
          {
          case PLUS_X:
            p.x += c*this->squareSize;
            break;
          case MINUS_X:
            p.x += -c*this->squareSize;
            break;
          case PLUS_Y:
            p.y += c*this->squareSize;
            break;
          case MINUS_Y:
            p.y += -c*this->squareSize;
            break;
          case PLUS_Z:
            p.z += c*this->squareSize;
            break;
          case MINUS_Z:
            p.z += -c*this->squareSize;
            break;
          }

          switch (rowDirection)
          {
          case PLUS_X:
            p.x += r*this->squareSize;
            break;
          case MINUS_X:
            p.x += -r*this->squareSize;
            break;
          case PLUS_Y:
            p.y += r*this->squareSize;
            break;
          case MINUS_Y:
            p.y += -r*this->squareSize;
            break;
          case PLUS_Z:
            p.z += r*this->squareSize;
            break;
          case MINUS_Z:
            p.z += -r*this->squareSize;
            break;
          }

          worldPoints->push_back(p);
        }
    }

    bool updateOgreCameraFromCheckerboardImg(cv::InputArray undistortedImg,
                                             const cv::Point3d& checkerStartCorner,
                                             CheckerDirection rowDirection, CheckerDirection colDirection,
                                             Ogre::Camera* camera) const
    {
      cv::Mat w_R_o, w_p_o;
      if (this->extrinsicsFromCheckerboardImg(undistortedImg, checkerStartCorner, rowDirection, colDirection, w_R_o, w_p_o))
      {
        updateOgreCameraFromCVExtrinsics(w_R_o, w_p_o, camera);
        return true;
      }
      else
        return false;
    }

    // Extrinsics of camera. Given an undistorted image of a checkerboard,
    // where columns of the checker board progress in +x and rows progress in +y.
    // Returns true if a checkerboard was found and the output arrays have been populated.
    bool extrinsicsFromCheckerboardImg(cv::InputArray undistortedImg,
                                       const cv::Point3d& checkerStartCorner,
                                       CheckerDirection rowDirection, CheckerDirection colDirection,
                                       cv::OutputArray w_R_o, cv::OutputArray w_p_o) const
    {
      std::vector<cv::Point2f> imgPoints;
      bool chessboardFound = cv::findChessboardCorners(undistortedImg, this->boardSize, imgPoints,
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
        this->generate3DCheckerboard(checkerStartCorner, rowDirection, colDirection, &worldPoints);

        // Perform pose estimation (origin w.r.t. webcam)
        cv::solvePnP(worldPoints, imgPoints, this->cameraMat, cv::Mat::zeros(4,1,CV_32F),
                     w_R_o, w_p_o);
      }

      return chessboardFound;
    }
  };

  // Takes a transformation representing the known object relative to the webcam 
  // (i.e. output of solvePNP) and updates the Ogre::Camera's pose to match the webcam's
  // view of the known object
  inline void updateOgreCameraFromCVExtrinsics(const cv::Mat& w_R_o, const cv::Mat& w_p_o, Ogre::Camera* camera)
  {
    // Get the inverse transformation (webcam w.r.t. object)
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

    camera->setOrientation(o_T_c.extractQuaternion());
    camera->setPosition(o_T_c.getTrans());
  }
}
