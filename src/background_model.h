/*
 *  Copyright (C) 1997-2008 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Authors : Redouane Kachach <redo.robot at gmail.com>
 *
 */
#ifndef _BACKGROUND_MODEL_
#define _BACKGROUND_MODEL_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include "singleton.h"

class Background : public CSingleton<Background>
{

public:

   /**
    *
    */
   void SetShadowDetectionState(bool newState) {m_shadowDetectionEnabled = newState;};

   /**
    *
    */
   void Process (const cv::Mat& new_frame);

   /**
    * returns true if the pixel on position pos has been moved, false otherwise.
    */
   bool isFg(int i, int j)
    {
      cv::Size image_rect = m_fgMaskImage.size();

      if ((j >= 0 && j < image_rect.width) && (i >= 0 && i < image_rect.height))
      {
        if (m_shadowDetectionEnabled)
        {
          return (m_fgMaskImage(i,j) == 255); // shadow detection active
        }
        else
        {
          return (m_fgMaskImage(i,j) != 0);  // shadow detection is NOT active
        }
      }

      return false;
    }

   /**
    *
    */
   cv::Mat& GetMask ();
   cv::Mat& GetMaskOrig();

   /**
    *
    */
   cv::Mat& GetBackground ();

protected:

   friend class CSingleton<Background>;

private:

   /**
    * Fields
    */
   cv::Mat_<uchar> m_fgMaskImage;
   cv::Mat m_background;
   cv::Mat m_fgMaskImageOrig;
   cv::Ptr<cv::BackgroundSubtractor> m_bg; //MOG2 Background subtractor

   bool m_shadowDetectionEnabled;

   cv::Mat m_iplmask;
   cv::Mat m_ipltmp;

   /**
    * This is a singelton class
    */
   Background();
};

#endif
