#include "util/ComputerVisionUtils.h"
//#include <filesystem>

using namespace std;
using namespace cv;

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

/* Requires C++ 11 or later
string fullfile(const string& first_path, const string& second_path) {
    filesystem::path filePath(first_path);
    filePath /= second_path;

    return filePath.string();
}
*/

void subplot(const string& window_name, const vector<Mat>& images, int num_rows, int num_cols, bool do_wait)
{
    // Check input parameters
    if (images.empty() || num_rows < 0 || num_cols < 0) {
        throw invalid_argument("Invalid parameters: one or more passed parameters are not valid");
    }

    if (images.size() > num_rows * num_cols) {
        throw invalid_argument("Invalid parameters: the number of images is greather than the grid size (num_rows x num_cols)");
    }

    // Determine the maximum height and width among all images
    int max_height = 0;
    int max_width = 0;

    for (const auto& image : images) {
        max_height = max(max_height, image.rows);
        max_width = max(max_width, image.cols);
    }

    // Create a blank canvas to hold the subplot grid
    Mat subplot_grid(max_height * num_rows, max_width * num_cols, CV_8UC3, Scalar(0, 0, 0));

    // Populate the subplot grid with images
    int row_index = 0;
    int col_index = 0;

    for (const auto& image : images) {
        // Check if the image is valid
        if (image.empty()) {
            throw runtime_error("Error: One or more images are not initialized or not valid");
        }

        // Resize the image
        Mat resized_image;
        resize(image, resized_image, Size(max_width, max_height));

        // Convert image type if necessary
        if (resized_image.channels() != subplot_grid.channels()) {
            cvtColor(resized_image, resized_image, COLOR_GRAY2BGR);
        }

        // Copy the resized image to the corresponding position in the subplot grid
        Rect roi(col_index * max_width, row_index * max_height, max_width, max_height);
        resized_image.copyTo(subplot_grid(roi));

        // Update the column index and row index
        col_index++;

        if (col_index >= num_cols) {
            col_index = 0;
            row_index++;
        }
    }

    // Display the subplot grid
    namedWindow(window_name, WINDOW_NORMAL);
    imshow(window_name, subplot_grid);
    if(do_wait)
        waitKey(0);
    
    destroyWindow(window_name);
}

void subplot(const string& window_name, const vector<Mat>& images, vector<string>& labels, int num_rows, int num_cols, bool do_wait)
{
    // Check input parameters
    if (images.empty() || num_rows < 0 || num_cols < 0) {
        throw invalid_argument("Invalid parameters: one or more passed parameters are not valid");
    }

    if (images.size() > num_rows * num_cols) {
        throw invalid_argument("Invalid parameters: the number of images is greather than the grid size (num_rows x num_cols)");
    }

    if (labels.size() != images.size()) {
        throw invalid_argument("Invalid parameters: the number of labels does not correspond to the same number of images");
    }

    // Determine the maximum height and width among all images
    int max_height = 0;
    int max_width = 0;

    for (const auto& image : images) {
        max_height = max(max_height, image.rows);
        max_width = max(max_width, image.cols);
    }

    // Create a blank canvas to hold the subplot grid
    Mat subplot_grid(max_height * num_rows, max_width * num_cols, CV_8UC3, Scalar(0, 0, 0));

    // Populate the subplot grid with images
    int row_index = 0;
    int col_index = 0;

    for (size_t i = 0; i < images.size(); i++) {
        // Check if the image is valid
        if (images[i].empty()) {
            throw runtime_error("Error: One or more images are not initialized or not valid");
        }

        // Resize the image
        Mat resized_image;
        resize(images[i], resized_image, Size(max_width, max_height));

        // Convert image type if necessary
        if (resized_image.channels() != subplot_grid.channels()) {
            cvtColor(resized_image, resized_image, COLOR_GRAY2BGR);
        }

        // Copy the resized image to the corresponding position in the subplot grid
        Rect roi(col_index * max_width, row_index * max_height, max_width, max_height);
        resized_image.copyTo(subplot_grid(roi));

        // Add label text over the image
        putText(subplot_grid, labels[i], Point(roi.x + 20, roi.y + 40), FONT_HERSHEY_SIMPLEX, 1.1, Scalar(244, 0, 161), 3, LINE_AA);

        // Update the column index and row index
        col_index++;

        if (col_index >= num_cols) {
            col_index = 0;
            row_index++;
        }
    }

    // Display the subplot grid
    namedWindow(window_name, WINDOW_NORMAL);
    imshow(window_name, subplot_grid);
    if(do_wait)
        waitKey(0);
    
    destroyWindow(window_name);
}