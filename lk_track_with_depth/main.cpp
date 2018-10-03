/*
 * Lucas-Kanade sparse optical flow, with depth
 *
 * by Mark Fassler
 *
 */


#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <librealsense2/rs.hpp>



#include "jet_colormap.h"  // gives us: colormap[256][3]
cv::Scalar depth_to_color(float d) {

	float dmin = 0.3;  // if d is here, then ii should be 255.0
	float dmax = 3.0;  // if d is here, then ii should be 0.0

	float m = -255.0 / (dmax - dmin);
	float b = 255 - (m * dmin);

	float ii = m*d + b;

	int i = (int) ii;
	if (i < 0) {
		i = 0;
	}  else if (i > 255) {
		i = 255;
	}

	return cv::Scalar(colormap[i][0], colormap[i][1], colormap[i][2]);
}



int main(int argc, char * argv[]) {

	int _DEPTH_WIDTH = 640;
	int _DEPTH_HEIGHT = 480;
	int _FPS = 30;
	int _RGB_WIDTH = 640;
	int _RGB_HEIGHT = 480;


	// ---------------------------------------------------------
	//  BEGIN:  Start RealSense
	// ---------------------------------------------------------
	rs2::context ctx;
	rs2::pipeline pipe;
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_COLOR, _RGB_WIDTH, _RGB_HEIGHT, RS2_FORMAT_RGB8, _FPS);
	cfg.enable_stream(RS2_STREAM_DEPTH, _DEPTH_WIDTH, _DEPTH_HEIGHT, RS2_FORMAT_Z16, _FPS);

	printf("Starting the Intel RealSense driver...\n");
	//auto profile = pipe.start();
	rs2::pipeline_profile profile = pipe.start();
	printf("...started.  (I hope.)\n");

	auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
	float depth_scale = depth_sensor.get_depth_scale();
	printf("Depth scale: %.6f\n", depth_scale);

	// Alignment object
	rs2::align align(RS2_STREAM_COLOR);
	// ---------------------------------------------------------
	//  END:  Start RealSense
	// ---------------------------------------------------------



	// LK Tracking parameters:
	cv::Size winSize(15, 15);
	int maxLevel = 3;
	cv::TermCriteria termcrit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 0.03);

	int track_len = 10;
	int frame_idx = 0;
	int detect_interval = 5;


	// Feature parameters...
	int maxCorners = 500;
	double qualityLevel = 0.3;
	double minDistance = 7.0;
	int blockSize = 7;


	std::vector<std::vector<cv::Point2f>> tracks;
	std::vector<std::vector<cv::Point2f>> new_tracks;

	std::vector<cv::Point2f> p0;
	std::vector<cv::Point2f> p1;
	std::vector<cv::Point2f> p0r;
	std::vector<cv::Point2f> pNew;

	cv::Mat frame_gray;
	cv::Mat prev_gray;
	cv::Mat mask;
	int i;

	while (1) {
		rs2::frameset frames = pipe.wait_for_frames();

		//rs2::video_frame color = frames.get_color_frame();
		//rs2::depth_frame depth = frames.get_depth_frame();

		auto aligned_frames = align.process(frames);
		rs2::video_frame color = aligned_frames.get_color_frame();
		rs2::video_frame depth = aligned_frames.get_depth_frame();

		cv::Mat imRGB(cv::Size(_RGB_WIDTH, _RGB_HEIGHT),
						CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

		cv::Mat imD(cv::Size(_DEPTH_WIDTH, _DEPTH_HEIGHT),
						CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

		// For now, we will use full-color for display:
		cv::cvtColor(imRGB, imRGB, cv::COLOR_RGB2BGR);
		// and grayscale for calculations:
		cv::cvtColor(imRGB, frame_gray, cv::COLOR_RGB2GRAY);

		if (prev_gray.empty()) {
			frame_gray.copyTo(prev_gray);
		}

		if (!tracks.empty()) {
			auto img0 = prev_gray;
			auto img1 = frame_gray;

			std::vector<uchar> status;
			std::vector<float> err;

			// The p0 vector is just the last point in each of the tracks items
			p0.resize(tracks.size());
			for (i=0; i<tracks.size(); ++i) {
				p0[i] = tracks[i].back();
			}

			// Forward tracking:
			cv::calcOpticalFlowPyrLK(prev_gray, frame_gray, p0, p1, status, err,
				winSize, maxLevel, termcrit);

			// Reverse tracking:
			cv::calcOpticalFlowPyrLK(frame_gray, prev_gray, p1, p0r, status, err,
				winSize, maxLevel, termcrit);


			new_tracks.resize(0);

			for (i=0; i<p0.size(); ++i) {

				// If the distance from forward to reverse tracking is about a single pixel,
				// then it's a good track.  Otherwise, discard.
				auto d = cv::norm(p0[i] - p0r[i]);
				if (d < 1.0) {

					tracks[i].push_back(cv::Point2f(p1[i].x, p1[i].y));
					if (tracks[i].size() > track_len) {
						tracks[i].erase(tracks[i].begin());
					}
					new_tracks.push_back(tracks[i]);

				}
			}

			std::swap(new_tracks, tracks);

			for (i=0; i<tracks.size(); ++i) {
				for (int j=0; j<tracks[i].size() - 1; ++j) {
					cv::line(imRGB, tracks[i][j], tracks[i][j+1], cv::Scalar(0, 255, 0));
				}
				// Why this no worky?:
				//cv::polylines(imRGB, tracks[i], false, cv::Scalar(0,255,0));

				auto d_value = imD.at<uint16_t>(tracks[i].back());
				cv::Scalar color;
				if (d_value > 20) {
					color = depth_to_color(depth_scale * d_value);
				} else { // invalid depth reading
					color = cv::Scalar(0,0,0);
				}

				cv::circle(imRGB, tracks[i].back(), 3, color, -1);
			}

		}



		// Every once-in-while, we'll try to add new points to the list of
		// points that we're tracking:
		if (frame_idx % detect_interval == 0) {

			// The p0 vector is just the last point in each of the tracks items
			p0.resize(tracks.size());
			for (i=0; i<tracks.size(); ++i) {
				p0[i] = tracks[i].back();
			}

			// We won't bother detecting near points that we're already tracking:
			mask.create(frame_gray.size(), frame_gray.type());
			mask.setTo(255);
			for (int i=0; i<p0.size(); ++i) {
				cv::circle(mask, p0[i], 5, 0, -1);
			}

			cv::goodFeaturesToTrack(frame_gray, pNew, 
				maxCorners, qualityLevel, minDistance, mask, blockSize); //, 3, 0, 0.04);

			// The new (hopefully) points will be the start of new tracks:
			int tSizeOrig = tracks.size();
			tracks.resize(tSizeOrig + pNew.size());

			for (int i=0; i<pNew.size(); ++i) {
				tracks[i+tSizeOrig].push_back(pNew[i]);
			}
		}


		frame_idx++;
		cv::swap(prev_gray, frame_gray);


		//cv::Mat imD_colorized;
		//cv::applyColorMap(imD, imD_colorized, cv::COLORMAP_JET);
		//cv::imshow("depth", imD);
		//cv::waitKey(1);

		cv::imshow("lk_track", imRGB);
		int ch = cv::waitKey(1);
		// ---------------------------------------------------------
		//  BEGIN:  Handle user closing the window
		// ---------------------------------------------------------
		switch (ch) {
		case 27: // esc key
		case 81:  // "Q"
		case 113:  // "q"
			goto theEnd;  // <-- can't break out of nested loops in C++ :-(
		}

		int res = cv::getWindowProperty("lk_track", 1);
		if (res < 0) { // window doesn't exist (because user closed it)
			break;
		}
		// ---------------------------------------------------------
		//  END:  Handle user closing the window
		// ---------------------------------------------------------
	}

	theEnd:

	return 0;
}




