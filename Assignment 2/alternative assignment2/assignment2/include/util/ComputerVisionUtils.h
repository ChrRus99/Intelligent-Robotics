#ifndef COMPUTER_VISION_UTILS_H
#define COMPUTER_VISION_UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <stdexcept>

/* 
	Converts the Mat image type into a string representing the OpenCV data type.
 
	Parameters:
		- type: src_img.type()
 
	Returns: a string representing the OpenCV data type.
 */
std::string type2str(int type);

/*
	Implements the fullfile function analogous to the one defined in MATLAB.
	Concatenates two paths in a cross-platforms way, by checking if first_path 
	ends with the path separator. If it does, it directly appends second_paths.
	Otherwise, it appends the path separator and then second_paths.

	Parameters:
		- first_path: the path to a folder
		- second_path: the file name or subfolder

	Returns: the concatenated path
 */
/* Requires C++ 11 or later
std::string fullfile(const std::string& first_path, const std::string& second_path);
*/

/*
	Implements the subplot function analogous to the one defined in MATLAB.
	Displays a subplot grid containing all the provided images in the selected 
	window.
	If the number of grid cells is lower than the number of images, the 
	overflowing images will not be displayed, conversely, if the number of grid 
	cells is greater than the number of images, the remaining cells will be empty.

	Parameters:
		- window_name: the name of the window in which display the subplot
		- images: a vector containing all the images we want to display
		- num_rows: the number of rows of the subplot grid
		- num_cols: the number of columns of the subplot grid
		- do_wait: execute waitKey()
	
	Returns: (void)
 */
void subplot(const std::string& window_name, 
			 const std::vector<cv::Mat>& src_vect,
			 int num_rows, int num_cols, 
			 bool do_wait = false);

/*
	Implements the subplot function analogous to the one defined in MATLAB.
	Displays a subplot grid containing all the provided images in the selected
	window.
	If the number of grid cells is lower than the number of images, the
	overflowing images will not be displayed, conversely, if the number of grid
	cells is greater than the number of images, the remaining cells will be empty.

	Parameters:
		- window_name: the name of the window in which display the subplot
		- images: a vector containing all the images we want to display
		- labels: a vector containing the labels of the images to display
		- num_rows: the number of rows of the subplot grid
		- num_cols: the number of columns of the subplot grid
		- do_wait: execute waitKey()

	Returns: (void)
 */
void subplot(const std::string& window_name, 
			 const std::vector<cv::Mat>& src_vect,
			 std:: vector<std::string>& labels, 
			 int num_rows, 
			 int num_cols, 
			 bool do_wait = false);

#endif // !COMPUTER_VISION_UTILS_H