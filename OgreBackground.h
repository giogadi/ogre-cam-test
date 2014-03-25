namespace OgreCV
{
  class OgreBackground
  {
  public:
    OgreBackground(int w, int h) :
      w(w),
      h(h),
      ownsRect(true)
    {
      // Create the texture
	    this->tex = Ogre::TextureManager::getSingleton().createManual(
		    "WebcamTexture", // name
		    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		    Ogre::TEX_TYPE_2D,      // type
		    this->w, this->h,         // width & height
		    0,                // number of mipmaps
		    Ogre::PF_BYTE_BGRA,     // pixel format
		    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
												    // textures updated very often (e.g. each frame)

      // ---------------------- Zero-out background texture ---------------
      Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->tex->getBuffer();
	    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!
	    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
      Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
      for (int i = 0; i < this->h; ++i)
        for (int j = 0; j < this->w; ++j)
          for (int k = 0; k < 4; ++k)
            *pDest++ = 0;
      pixelBuffer->unlock();

	    // Create a material using the texture
	    this->mat = Ogre::MaterialManager::getSingleton().create(
		    "WebcamBackgroundMaterial", // name
		    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
 
	    this->mat->getTechnique(0)->getPass(0)->createTextureUnitState("WebcamTexture");
	    this->mat->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
	    this->mat->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
	    this->mat->getTechnique(0)->getPass(0)->setLightingEnabled(false);

      rect = new Ogre::Rectangle2D(true);
	    this->rect->setCorners(-1.0, 1.0, 1.0, -1.0);
	    this->rect->setMaterial("WebcamBackgroundMaterial");
 
	    // Render the background before everything else
	    this->rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
 
	    // Use infinite AAB to always stay visible
	    Ogre::AxisAlignedBox aabInf;
	    aabInf.setInfinite();
	    this->rect->setBoundingBox(aabInf);
    }

    void updateFromCVImg(const cv::Mat& img)
    {
      Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->tex->getBuffer();
	    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // for best performance use HBL_DISCARD!
	    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
	    Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	    Ogre::uint8* pSrc = static_cast<Ogre::uint8*>(img.data);
	    for (size_t r = 0; r < this->h; ++r)
	    {
        for (size_t c = 0; c < this->w; ++c)
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
    
    // TRANSFERS OWNERSHIP OF RECTANGLE
    Ogre::Rectangle2D* getRect()
    {
      ownsRect = false;
      return this->rect;
    }

    ~OgreBackground()
    {
      if (ownsRect)
        delete rect;
    }

  private:
    int w, h;
    Ogre::TexturePtr tex;
    Ogre::MaterialPtr mat;
    Ogre::Rectangle2D* rect;

    bool ownsRect;
  };
}