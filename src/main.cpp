/*
 *
 *  Copyright (C) 1997-2010 JDERobot Developers Team
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
 *  Authors : David Lobato Bravo <dav.lobato@gmail.com>
 *
 */
#include <signal.h>
#include <cstdlib>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <string>
#include <model.h>
#include <execinfo.h>
#include <unistd.h>
#include <chrono>
#include <thread>


#include "colorspacesmm.h"
#include "view.h"
#include "viewgtk.h"
#include "viewfile.h"

using namespace trafficmonitor;

// Exit cleanly to generate gprof output (when compiled with -pg option)
void my_handler(int s){
  printf("Caught signal %d\n",s);
  exit(1);

}

/**
 *
 */
void printStackTraceAndExit()
{
  void *array[30];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 30);

  // print out all the frames to stderr
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

/**
 *
 */
int handleOpenCvError(int status
                      ,const char* func_name
                      ,const char* err_msg
                      ,const char* file_name
                      ,int line
                      ,void* userdata )
{
  printStackTraceAndExit();
}

void init(std::unique_ptr<Model>& model)
{
  // Configure and create algorithm
  TrafficMonitorAlgorithmPtr alg;
  TrafficMonitorAlgorithmConfig cfg;
  alg.reset(new TrafficMonitorAlgorithm(cfg));
  model->setAlgorithm(alg);

  if (!cfg.isValid())
  {
    exit (EXIT_FAILURE);
  }
  else
  {
    cfg.show();
  }

  // Start the corresponding view
  if (cfg.gui == TrafficMonitorAlgorithmConfig::GTK_VIEW)
  {
    ViewPtr vp(new ViewGtk(*model, cfg.get_glade_filename()));
    model->addObserver(vp);
  }
  else if(cfg.gui == TrafficMonitorAlgorithmConfig::FILE_VIEW)
  {
    ViewPtr vp(new ViewFile(*model));
    model->addObserver(vp);
  }
  else
  {
    // No view has been configured, then nothing will be done.
    cout << "Warning: found a non supported view mode '" << cfg.gui <<"'" << endl;
  }
}

/**
 *
 */
void processImage(cv::VideoCapture& capture, cv::Size& configSize, std::unique_ptr<Model>& model){

  static bool firstImage = true;
  colorspaces::Image::FormatPtr fmt = colorspaces::Image::Format::searchFormat("RGB8");

  // Get new frame
  cv::Mat img;
  capture >> img;
  if (not img.data)
  {
    capture.set(CV_CAP_PROP_POS_AVI_RATIO, 0.0);
    capture >> img;
  }
  cv::cvtColor(img, img, CV_RGB2BGR);
  if(configSize != img.size())
    cv::resize(img, img,configSize);

  colorspaces::Image cImg(img.cols,
                          img.rows,
                          fmt,
                          &(img.data[0]));//data will be

  timeval timeStamp;
  gettimeofday(&timeStamp, NULL);
  if (firstImage)
  {
    model.reset(new Model(cImg.clone(), timeStamp));
    init(model);
    firstImage = false;
  }
  else
  {
    model->setImage(cImg.clone(), timeStamp);  //FIXME: find a way to avoid this copy
  }
}

void checkFrameRate(double frame_dur_ms, std::chrono::system_clock::time_point& t1, std::chrono::system_clock::time_point& t2)
{
  t1 = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> work_time = t1 - t2;

  if (work_time.count() < frame_dur_ms)
  {
    std::chrono::duration<double, std::milli> delta_ms(frame_dur_ms - work_time.count());
    auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
  }

  t2 = std::chrono::system_clock::now();
}

void checkArgs(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: ./trafficmonitor video_file_path" << std::endl;
    exit(1);
  }
  else
  {
    std::ifstream infile(argv[1]);
    if (!infile.good())
    {
      std::cout << "Cannot open video file '" << string(argv[1]) << "'" << std::endl;
      exit(1);
    }
  }
}

void configureErrorHandling()
{
  cv::redirectError(handleOpenCvError);
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char **argv)
{
  cv::VideoCapture capture;

  checkArgs(argc, argv);
  capture.open(argv[1]);
  configureErrorHandling();

  if (!capture.isOpened())
  {
    cout << "Cannot open URI!!" << endl;
    exit(1);
  }

  try
  {
    auto t1 = std::chrono::system_clock::now();
    auto t2 = std::chrono::system_clock::now();
    cv::Size configSize = cv::Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    std::unique_ptr<Model> model;
    const int FPS = 30;

    while (capture.isOpened())
    {
      checkFrameRate(1000/FPS, t1, t2);
      processImage(capture, configSize, model);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Error catched : " << e.what() << std::endl;
  }

  return 0;
}
