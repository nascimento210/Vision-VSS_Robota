#include <opencv2/calib3d.hpp>
#include "Funcs.h"
#include "cvui.h"

#define WINDOW_NAME	"Rows and Columns"


int main()
{
	// cvui
	cv::Mat frame = cv::Mat(900, 1300, CV_8UC3);
	cv::Mat display1;
	cv::Mat display2;
	
	int colorCount = 0;
	std::vector<std::string> cores = { "Amarelo","Azul", "Vermelho", "Verde", "Rosa" };

	// fonts of frame
	cv::Mat imgdefault = cv::imread("C:/Users/tisie/Documents/opencvtest/img/lena.jpg", cv::IMREAD_COLOR);
	cv::Mat imgstatic = cv::imread("C:/Users/tisie/Documents/opencvtest/img/model1.png", cv::IMREAD_COLOR);
	cv::VideoCapture vid("C:/Users/tisie/Documents/opencvtest/vid/ball_move.mp4");
	cv::VideoCapture webC(0);
	int pathChose = 0;

	// perspective
	int mouse_count = 0;
	int cornerx[4] = {0};
	int cornery[4] = {0};
	std::vector<cv::Point> dst = {{0, 0}, {600, 0},
		{0, 520}, {600, 520}};
	bool applyPerspective = false;
	bool applyMask = false;

	// mask
	cv::Mat black = cv::Mat(520, 600, CV_8U, cv::Scalar(0, 0, 0));
	std::vector<cv::Mat> colorMask = {black, black, black, black, black };
	std::vector<std::vector<float>> hsvmax = { {45, 255, 255}, {123,255,255}, {16,255,255}, {99,255,255}, {179,255,255}, {0, 0, 0} };
	std::vector<std::vector<float>> hsvmin = { {9, 0, 154}, {95, 0, 69}, {0, 0, 140}, {41, 0, 116}, {137, 0, 199}, {0, 0, 0} };
	
	cv::Mat hsvImg;
	std::vector<std::vector<cv::Point>> contorno[5];

	//robot position
	std::vector<cv::Point> poseYelow = { {0,0}, {0,0}, {0,0} };
	std::vector<cv::Point> poseBlue = { {0,0}, {0,0}, {0,0} };
	std::vector<cv::Point> poseRed = { {0,0}, {0,0} };
	std::vector<cv::Point> poseGreen = { {0,0}, {0,0} };
	std::vector<cv::Point> posePink = { {0,0}, {0,0} };
	
	std::vector<std::vector<cv::Point>> colorpose;//"Amarelo","Azul", "Vermelho", "Verde", "Rosa"
	std::vector<cv::Point> teamyel = { {0,0}, {0,0}, {0,0} };
	std::vector < std::vector<cv::Point>> teamblu = { {{0,0}, {0,0}, {0,0}, {0,0}}, {{0,0}, {0,0}, {0,0}, {0,0}} };

	//other variables
	cv::Mat sup;
	cv::Mat tim;

	cvui::init(WINDOW_NAME);

	while (true) {

		std::vector<cv::Point> src = {
			{cornerx[0], cornery[0]}, {cornerx[1], cornery[1]},
			{cornerx[2], cornery[2]}, {cornerx[3], cornery[3]}};

		frame = cv::Scalar(49, 52, 49);
		
		switch (pathChose)
		{
		case 0: {
			cv::resize(imgdefault, sup, cv::Size(600, 520));
			break;
		}
		case 1: {

			cv::resize(imgstatic, sup, cv::Size(600, 520));		
			break;
			
		}
		case 2: {
			vid.read(sup);
			cv::resize(sup, sup, cv::Size(600, 520));
			break;
		}
		case 3: {
			webC.read(sup);
			cv::flip(sup, sup, 1);
			
			break;
		}
		default:
			break;
		}
		cv::Mat mattrans;
		mattrans = cv::findHomography(src, dst);
		perspective(sup, sup, mattrans, applyPerspective);
		display1 = sup.clone();
		cv::cvtColor(sup, hsvImg, cv::COLOR_BGR2HSV, 3);
		if (applyMask) {
			
			
			colorpose.resize(colorMask.size());
			for (int i = 0; i < colorMask.size(); i++) {
				HsvMask(hsvImg, hsvmin.at(i), hsvmax.at(i), colorMask.at(i));
				cv::findContours(colorMask.at(i), contorno[i], cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
				minArea(contorno[i]);
				centers(contorno[i], colorpose.at(i));
				cv::drawContours(display1, contorno[i], -1, (255, 255, 255), 3);
			}
			robotPose(teamblu, colorpose); 
			display2 = colorMask.at(colorCount).clone();
			//cvui::imshow("teste", display2);
			for(int x = 0; x<2; ++x){
				for (cv::Point y : teamblu.at(x)){
					cv::circle(display1, y, 10, (255,255,255));
				}

			}

		}
		

		int py = display1.rows;
		int px = display1.cols;
		int status = cvui::iarea(10, 20, px, py);

		// first row
		cvui::beginRow(frame, 10, 20, -1, py, 10);
			cvui::image(display1);
			if (applyMask == true) {
				cvui::image(hsvImg);
			}
			
		cvui::endRow();

		//second row
		cvui::beginRow(frame, 10, 30+py, 300, 350, 10);

			cvui::beginColumn(-1, -1, 5);
				pathChose = cvui::counter(&pathChose, 1);
				applyPerspective = cvui::checkbox("Transform", &applyPerspective);
				applyMask = cvui::checkbox("Mask", &applyMask);
				cvui::printf("canto 1 (%i,%i)", cornerx[0], cornery[0]);
				cvui::printf("canto 2 (%i,%i)", cornerx[1], cornery[1]);
				cvui::printf("canto 3 (%i,%i)", cornerx[2], cornery[2]);
				cvui::printf("canto 4 (%i,%i)", cornerx[3], cornery[3]);
				cvui::printf("n cont (%i)", mouse_count);
				colorCount = cvui::counter(&colorCount, 1);
				cvui::text(cores.at(colorCount), 0.5);


			cvui::endColumn();

			cvui::beginColumn(-1, -1, 5);
				cvui::text("HSV Range", 0.8);
				cvui::trackbar(220, &hsvmax.at(colorCount).at(0), (float)0, (float)179);
				cvui::trackbar(220, &hsvmin.at(colorCount).at(0), (float)0, (float)179);
				cvui::trackbar(220, &hsvmax.at(colorCount).at(1), (float)0, (float)255);
				cvui::trackbar(220, &hsvmin.at(colorCount).at(1), (float)0, (float)255);
				cvui::trackbar(220, &hsvmax.at(colorCount).at(2), (float)0, (float)255);
				cvui::trackbar(220, &hsvmin.at(colorCount).at(2), (float)0, (float)255);
			cvui::endColumn();

			cvui::beginColumn(100, 200, 5);
				cvui::printf("Time    Amarelo  |  Azul");
				if (applyMask) {
					cvui::printf("Vermelho  (%i,%i)  |  (%i,%i)", teamblu.at(0).at(0).x, teamblu.at(0).at(0).y, teamblu.at(1).at(0).x, teamblu.at(1).at(0).y);
					cvui::printf("Verde     (%i,%i)  |  (%i,%i)", teamblu.at(0).at(1).x, teamblu.at(0).at(1).y, teamblu.at(1).at(1).x, teamblu.at(1).at(1).y);
					cvui::printf("Rosa      (%i,%i)  |  (%i,%i)", teamblu.at(0).at(2).x, teamblu.at(0).at(2).y, teamblu.at(1).at(2).x, teamblu.at(1).at(2).y);
				}
				
			cvui::endColumn();

		cvui::endRow();

		tableCorners(cornerx, cornery, mouse_count, status); // get the corner points


		cvui::imshow(WINDOW_NAME, frame);
		if (cv::waitKey(5) == 27) {
			break;
		}
	}

	return 0;
}