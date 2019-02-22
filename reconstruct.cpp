#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <array>
#include <ctime>


// Main function
int main(int argc, const char** argv) {

  // Retrieve input video and deconstruct
  cv::VideoCapture cap(argv[1]);
  if (!cap.isOpened()) {
    std::cout << "Failed to open file: " << argv[1] << std::endl;
    return 0;
  }

  // Make directory for reconstruction with ts
  std::time_t result = std::time(nullptr);
  std::string ts = std::to_string(result);
  std::string cmd = "mkdir reconstructions/" + ts;
  system(cmd.c_str());

  while(true) {
    break;
    // Capture frame by frame
    cv::Mat frame;
    cap >> frame;

    // If frame is empty, break loop
    if (frame.empty()) {
      break;
    }

    // Display the frame
    imshow("Frame", frame);

    // Exit when ESC pressed
    char c = (char) cv::waitKey(25);
    if (c == 27) {
      break;
    }
  }

  // Release video capture
  cap.release();

  // Close all frames
  cv::destroyAllWindows();

  return 0;
}
