#include <iostream>
#include "background_model.h"

using namespace std;
using namespace cv;

/**
 *
 */
Background::Background(){

  m_bg = createBackgroundSubtractorMOG2();

  // // Configure the MOG algorithm parameters
  // m_bg->set("nmixtures",5);
  // m_bg->set("detectShadows", true);
  // m_bg->set("nShadowDetection", 127);
  // m_bg->set("fTau", 0.5);
}


Mat& Background::GetBackground()
{
   m_bg->getBackgroundImage(m_background);
   return m_background;
};

Mat& Background::GetMask()
{
  return m_fgMaskImage;
}

Mat& Background::GetMaskOrig()
{
  return m_fgMaskImageOrig;
}

/**
 *
 */
void Background::Process(const Mat& new_frame)
{
  m_bg->apply(new_frame, m_fgMaskImage);
  m_fgMaskImageOrig = m_fgMaskImage.clone();
}
