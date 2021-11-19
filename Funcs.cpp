#include "Funcs.h"


void filters(cv::Mat& src, cv::Mat& dst)
{

	cv::bilateralFilter(src, dst, 5, 75, 75);
}

void HsvMask(cv::Mat& src, std::vector<float> hsvmin, std::vector<float> hsvmax, cv::Mat& masks)
{
	cv::Mat teste;
	cv::inRange(src, hsvmin, hsvmax, teste);

	
	cv::Mat estruero = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(10, 10));
	cv::Mat estrudil = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(teste, estruero, 3);
	//cv::morphologyEx(teste, teste, cv::MORPH_CLOSE, estru);
	cv::dilate(teste, masks, estrudil);
	//teste.convertTo(dst, CV_8UC3);
}


void tableCorners(int mousex[], int mousey[], int& cont, int mouseStatus) {
	if (mouseStatus == cvui::CLICK) {
		mousex[cont] = cvui::mouse().x;
		mousey[cont] = cvui::mouse().y;
		if (cont == 3) {
			cont = 0;
		}
		else {
			cont += 1;
		}
	}
}

void perspective(cv::Mat& src, cv::Mat& dst, cv::Mat& transform, bool apply) { //passar dst como referencia para mudar resultado globalmente
	if (apply) {
		cv::warpPerspective(src, dst, transform, cv::Size(600, 520));
	}
	else {
		dst = src;
	}
}

void totalMask(std::vector<cv::Mat>& masks, cv::Mat& finalMask)
{
	for (int i = 0; i < masks.size(); ++i) {
		//cv::bitwise_or(masks.at(i), finalMask, finalMask);
		finalMask += masks.at(i);
	}
}

void minArea(std::vector<std::vector<cv::Point>>& contorno)
{
	double area = 0;
	for (int x = 0; x < contorno.size(); ++x ) {
		area = cv::contourArea(contorno[x]);
		if (area < 200) {
			contorno.erase(contorno.begin() + x--);
		}
	}
}

void centers(std::vector<std::vector<cv::Point>>& contorno, std::vector<cv::Point>& poseColor)
{
	poseColor.clear();
	std::vector<cv::Moments> mu(contorno.size());
	for (int i = 0; i < contorno.size(); ++i) {
		mu[i] = cv::moments(contorno[i], false);
		poseColor.push_back(cv::Point(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00));
	}
}

void robotPose(std::vector<std::vector<cv::Point>>& teamPose,  std::vector<std::vector<cv::Point>>& poseColor)
{
	int posex=0;
	int posey=0;
	int colorId = 0;
	int objId = 0;
	double dist = 0;
	int robx = 0;
	int roby = 0;
	int color = 0;
	int difx = 0;
	int dify = 0;
	double distance = 10000.0;
	std::vector<std::vector<cv::Point>> head = { poseColor.at(0), poseColor.at(1) };
	for (int count = 0; count < head.size(); ++count) {//loop pelo amarelo e azul
		for (int i = 0; i < head.at(count).size(); ++i) {//loop cada elemento de amarelo e azul
			distance = 10000.0;
			posex = head.at(count).at(i).x;
			posey = head.at(count).at(i).y;
			for (int x = 2; x < poseColor.size(); ++x) {//loop outras cores
				for (int j = 0; j < poseColor.at(x).size(); ++j) {// loop elemento de outras cores
					difx = posex - poseColor.at(x).at(j).x;
					dify = posey - poseColor.at(x).at(j).y;
					dist = std::sqrt(std::pow(difx, 2) + std::pow(dify, 2));
					if (dist < distance) {
						distance = dist;
						colorId = x;
						objId = j;
					}
				}
			}
			color = colorId - 2;
			robx = (posex + poseColor.at(colorId).at(objId).x) / 2;
			roby = (posey + poseColor.at(colorId).at(objId).y) / 2;
			teamPose.at(count).at(color) = cv::Point(robx, roby);
			
		}
	
		
	}//sequencia da sec color vai ser "Vermelho", "Verde", "Rosa" loga na posi��o 2 sera vermelho e azul ou amarelo
}
