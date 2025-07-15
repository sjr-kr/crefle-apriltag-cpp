#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <map>

using namespace cv;
using namespace std;

bool runCalibration(int cameraIndex, Size boardSize, float squareSize, int numCaptures) {
    VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open camera " << cameraIndex << endl;
        return false;
    }

    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;
    Mat frame, gray;
    vector<Point3f> objCorners;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            objCorners.emplace_back(j * squareSize, i * squareSize, 0);

    int captures = 0;
    cout << "Calibration Mode: Press 'c' to capture when checkerboard detected (ESC to exit)" << endl;
    while (captures < numCaptures) {
        cap >> frame;
        if (frame.empty()) break;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(frame, boardSize, corners, found);
        }
        imshow("Calibration", frame);
        char key = (char)waitKey(30);
        if (key == 27) break;
        if (found && (key == 'c' || key == 'C')) {
            imagePoints.push_back(corners);
            objectPoints.push_back(objCorners);
            captures++;
            cout << "Captured " << captures << " / " << numCaptures << endl;
        }
    }
    destroyWindow("Calibration");
    if (captures < numCaptures) {
        cerr << "Error: Only captured " << captures << " frames." << endl;
        return false;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, gray.size(),
                                 cameraMatrix, distCoeffs, rvecs, tvecs);
    cout << "Calibration RMS error: " << rms << endl;

    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();
    cout << "Saved intrinsics to intrinsics.yml" << endl;
    return true;
}

void drawCube(Mat &frame, const Vec3d &rvec, const Vec3d &tvec, float size,
              const Mat &cameraMatrix, const Mat &distCoeffs) {
    // Half size
    float s = size / 2.0f;
    vector<Point3f> pts = {
        {-s, -s, 0}, { s, -s, 0}, { s,  s, 0}, {-s,  s, 0},  // bottom
        {-s, -s, -size},{ s, -s, -size},{ s,  s, -size},{-s,  s, -size} // top (upwards)
    };
    vector<Point2f> imgpts;
    projectPoints(pts, rvec, tvec, cameraMatrix, distCoeffs, imgpts);
    // draw bottom
    for (int i = 0; i < 4; ++i)
        line(frame, imgpts[i], imgpts[(i+1)%4], Scalar(255,0,0), 2);
    // draw top
    for (int i = 4; i < 8; ++i)
        line(frame, imgpts[i], imgpts[4 + (i+1)%4], Scalar(255,0,0), 2);
    // draw sides
    for (int i = 0; i < 4; ++i)
        line(frame, imgpts[i], imgpts[i+4], Scalar(255,0,0), 2);
}

bool runPoseEstimation(int cameraIndex, const string &intrinsicsFile,
                       float tagSize, float platformSize) {
    FileStorage fs(intrinsicsFile, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Error: Cannot open " << intrinsicsFile << endl;
        return false;
    }
    Mat cameraMatrix, distCoeffs;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    auto baseDict = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    Ptr<aruco::Dictionary> dictionary = makePtr<aruco::Dictionary>(baseDict);
    Ptr<aruco::DetectorParameters> params = makePtr<aruco::DetectorParameters>();

    VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open camera " << cameraIndex << endl;
        return false;
    }
    cout << "Pose Estimation Mode: detecting AprilTags... (ESC to exit)" << endl;

    Mat frame;
    while (cap.read(frame)) {
        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(frame, dictionary, corners, ids, params);
        if (!ids.empty()) {
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, tagSize,
                                             cameraMatrix, distCoeffs,
                                             rvecs, tvecs);
            map<int,int> idx;
            for (int i = 0; i < (int)ids.size(); ++i) idx[ids[i]] = i;

            // platform
            bool ok = true;
            vector<Vec3d> pcenters;
            for (int pid = 0; pid < 4; ++pid) {
                if (!idx.count(pid)) { ok = false; break; }
                pcenters.push_back(tvecs[idx[pid]]);
            }
            if (!ok) {
                putText(frame, "Platform tags missing", Point(10,30),
                        FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
            }

            // draw markers
            aruco::drawDetectedMarkers(frame, corners, ids);

            // object
            if (ok && idx.count(10)) {
                // compute rel pos
                Vec3d center(0,0,0);
                for (auto &v : pcenters) center += v;
                center *= 0.25;
                Vec3d relVec;
                Mat tvec_c = (Mat_<double>(3,1) <<
                             tvecs[idx[10]][0], tvecs[idx[10]][1], tvecs[idx[10]][2]);
                Mat t_center = (Mat_<double>(3,1) <<
                                center[0], center[1], center[2]);
                // platform orientation
                Vec3d v01 = pcenters[1] - pcenters[0];
                Vec3d v03 = pcenters[3] - pcenters[0];
                Vec3d xAxis = v01 / norm(v01);
                Vec3d yAxis = v03 / norm(v03);
                Vec3d zAxis = xAxis.cross(yAxis);
                zAxis /= norm(zAxis);
                yAxis = zAxis.cross(xAxis);
                Mat R_pc = (Mat_<double>(3,3) << xAxis[0], yAxis[0], zAxis[0],
                                               xAxis[1], yAxis[1], zAxis[1],
                                               xAxis[2], yAxis[2], zAxis[2]);
                Mat rel_m = R_pc.t() * (tvec_c - t_center);
                relVec = Vec3d(rel_m);
                // display in mm with Z upwards
                double x_mm = relVec[0] * 1000;
                double y_mm = relVec[1] * 1000;
                double z_mm = -relVec[2] * 1000;
                stringstream ss;
                ss << fixed << setprecision(1)
                   << "Obj rel pos (mm): X=" << x_mm
                   << " Y=" << y_mm
                   << " Z=" << z_mm;
                putText(frame, ss.str(), Point(10,frame.rows-20),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);

                // draw cube
                drawCube(frame, rvecs[idx[10]], tvecs[idx[10]], tagSize,
                         cameraMatrix, distCoeffs);
            }
        }
        imshow("Pose Estimation", frame);
        if (waitKey(1) == 27) break;
    }
    return true;
}

int main() {
    cout << "Select mode:\n1: Camera Calibration\n2: AprilTag Pose Estimation\nYour choice: ";
    int mode; cin >> mode;
    if (mode == 1) {
        int camIdx, rows, cols, captures;
        float sqSize;
        cout << "Enter camera index: "; cin >> camIdx;
        cout << "Checkerboard inner corners rows: "; cin >> rows;
        cout << "Checkerboard inner corners cols: "; cin >> cols;
        cout << "Square size (m): "; cin >> sqSize;
        cout << "Number of captures: "; cin >> captures;
        if (!runCalibration(camIdx, Size(cols, rows), sqSize, captures))
            return -1;
    } else if (mode == 2) {
        int camIdx;
        string intrin;
        float tagSize, platformSize;
        cout << "Enter camera index: "; cin >> camIdx;
        cout << "Intrinsics file (e.g., intrinsics.yml): "; cin >> intrin;
        cout << "AprilTag size (m): "; cin >> tagSize;
        cout << "Platform side length (m): "; cin >> platformSize;
        if (!runPoseEstimation(camIdx, intrin, tagSize, platformSize))
            return -1;
    } else {
        cerr << "Invalid mode selected." << endl;
        return -1;
    }
    return 0;
}
