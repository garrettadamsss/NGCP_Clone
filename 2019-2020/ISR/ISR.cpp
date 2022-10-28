#include "ISR.h"


using namespace cv;
using namespace std;
using namespace grip;


void ISR::log(String message) {
    cout << message << endl;
    log_file << message << endl;
}


double ISR::grab_altitude() {
    return 39.3700787402 * alt; // convert from meters to feet
}


void ISR::array3d(Mat small, int array[][100][100]) {
    Vec3b color[small.cols][small.rows];

    if (!small.empty()) {

        for (int i = 0; i < small.cols; i++) {
            for (int j = 0; j < small.rows; j++) {
                color[i][j] = small.at<Vec3b>(Point(i, j));
            }
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < small.cols; j++) {
                for (int k = 0; k < small.rows; k++) {
                    array[i][j][k] = (int) color[j][k][i];
                }
            }
        }
    }
}

//takes a 3d array and puts rgb values into mat
void ISR::reCreate(int array[][100][100]) {
    Vec3b color[100][100];
    Mat img = Mat(100, 100, CV_8UC3);
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 100; j++) {
            color[i][j].val[0] = array[0][i][j];
            color[i][j].val[1] = array[1][i][j];
            color[i][j].val[2] = array[2][i][j];
            img.at<Vec3b>(Point(i, j)) = color[i][j];
        }
    }
    imshow("100x100", img);
}

void ISR::process_mats(Mat img, vector<vector<Point> > *ballPoints, vector<vector<Point> > *bullseyePoints) {
    double altitude = grab_altitude();
    visball->Process(img, altitude);
    bullseye->Process(img, altitude);
    *ballPoints = visball->GetConvexHullsOutput();
    *bullseyePoints = bullseye->GetConvexHullsOutput();
}

void ISR::process_contour(vector<vector<Point> > hullsOutput, cv::Point *pt, int *radius) {
    // Vector that holds list of contours, each contour is a list of points
    vector<Point> output;

    // Min/Max to find radius (only x is needed)
    int min_pt = 0;
    int max_pt = 0;

    // If several outputs exist, there are multiple "balls" detected - i.e. bad news
    if (hullsOutput.size() > 1) {
        pt->x = -1;
        pt->y = -1;
        *radius = -1;
    } else if (hullsOutput.size() == 1) {
        // No need for outermost vector, as this is the only contour
        output = hullsOutput[0];

        min_pt = output[0].x;
        max_pt = output[0].x;

        // Average all points to find center
        for (int i = 0; i < output.size(); i++) {
            pt->x += output[i].x;
            pt->y += output[i].y;
            if (min_pt > output[i].x) min_pt = output[i].x;
            if (max_pt < output[i].x) max_pt = output[i].x;
        }
        pt->x /= output.size();
        pt->y /= output.size();
        *radius = max_pt - min_pt;

    } else { // nothing was found
        pt->x = -1;
        pt->y = -1;
        *radius = -1;
    }
}

void
ISR::send_to_db(cv::Point point, double radius, GPSLocation gps, cv::Mat imgOriginal, int db_index) {
    String obj_detected = "";
    if (db_index == 0) {
        obj_detected = "Ball";
    } else if (db_index == 1) {
        obj_detected = "Bullseye";
    } else if (db_index == 2) {
        obj_detected = "Thermal";
    }

    gps.addBL(point.x, point.y, alt, yaw, lat, lon);
    uav_db.InsertTARGET_LOCAL(0, gps.getLatGPS(), gps.getLongGPS(), db_index);
    //cv::circle(imgOriginal, point, radius, CV_RGB(0, 0, 0), 2);

    log(obj_detected + " detected at pixel " + to_string(point.x) + ", " + to_string(point.y) + "\t lat: " +
        to_string(gps.getLatGPS()) + ", long: " + to_string(gps.getLongGPS()));

    Mat small;

    // Ensure object fits on 100 x 100 screen
    if (point.x - 50 > 0 && point.y - 50 > 0 && point.x + 50 < imgOriginal.cols
        && point.y + 50 < imgOriginal.rows) {
        small = imgOriginal(cv::Rect(point.x - 50, point.y - 50, 100, 100));
    }

    if (!small.empty()) {
        if (DEBUG_ENABLED)
            imshow(obj_detected, small);
        int array[3][100][100];
        img.clearBuffer();
        uav_db.ClearImageString_S();
        array3d(small, array);
        img.imageToString(array);
        //cout << img.size_stringBuffer << endl;
        for (int i = 0; i < img.size_stringBuffer; ++i) {
            uav_db.InsertImageString_S(img.stringBuffer[i]);
        }
        // !-DEBUG
        //reCreate(array);
        //(2000000);
    }
}

void ISR::end(){
    log("Mission has been ended via comms");
    camera.release();
    log_file.close();
}


ISR::ISR() {
    uav_db.Connect("plane1", "root", "ngcp"); //Connect to the database
    init(); //move init to own function in case need arises to restart from within this class
}

void ISR::init(){
    ready = true;

    sleep(1);
    uav_db.ClearImageString_S();
    
    int mission_status = 0;
    while(mission_status != 3){
    	mission_status = uav_db.CheckMissionStatus();
    }

    img.mapCharToValue();

    //4096 x 2160

    // Attempt to connect to webcam on port 0
    camera = VideoCapture(0);

    if (!camera.isOpened()) {
        cout << "Unable to connect to camera on port 0." << endl;
        ready = false;
    }

    //Create ISR object outside of while loop
    visball = new Visball();
    bullseye = new TargetIsolator();
    thermal_detector = new ThermalFailsafe();

    srand(time(NULL));

    int random = rand() % 100000;

    // File info for saving logs
    string log_file_name = to_string(random) + ".txt";
    log_file = ofstream(log_file_name);
}


void ISR::start(bool *visible_search, mutex *lock, double *gps_info, double *yaw_in, int *mission_status){
    bool vis;

    while(true){
        lock->lock();
        if(*mission_status != 3){
            //TODO: REMOVE DEBUG
            printf("EXITING ISR\n");
            lock->unlock();
            return; //isr has ended
        }
        alt = gps_info[0];
        lat = gps_info[1];
        lon = gps_info[2];
        yaw = *yaw_in;
        vis = *visible_search;
        lock->unlock();
        run(vis);
    }
}


int ISR::run(bool visible_search) {
    if (!ready) {  //if something prevented it from being ready, try to set everything up again
        init();
    }
    if (!ready) return 1; //if still not ready, can't proceed, stop

    // Attempt to capture mat
    Mat imgOriginal;
    


    //TODO: REMOVE DEBUG
    printf("RUNNING ISR\n");

    camera >> imgOriginal;
    if (imgOriginal.empty()) {
        log("Unable to retrieve image from camera.");
        return 0;
    }

    Mat last = imgOriginal; //cycle through frames to clear buffer and increase speed
    for (int i = 0; i < 5; i++) {
        camera >> imgOriginal;
        if (imgOriginal.empty()) {
            imgOriginal = last;
            break;
        }
        last = imgOriginal;
    }

    if (DEBUG_ENABLED) {
        imshow("original", imgOriginal);
    }

    // Process mat
    if (visible_search) {
        process_mats(imgOriginal, &ballPoints, &bullseyePoints);

        // Center pixel location of objects
        cv::Point center_ball;
        cv::Point center_bullseye;

        int radius_ball = 0;
        int radius_bullseye = 0;


        // Analyze contours return from process_mats

        process_contour(ballPoints, &center_ball, &radius_ball);

        //ball located
        if (center_ball.x != -1) {
            send_to_db(center_ball, radius_ball, ballGPS, imgOriginal, 0);
        }

        process_contour(bullseyePoints, &center_bullseye, &radius_bullseye);

        //target located
        if (center_bullseye.x != -1) {
            send_to_db(center_bullseye, radius_bullseye, targetGPS, imgOriginal, 1);
        }
    } else { //not visible search, assume in IR
        double altitude = grab_altitude();
        thermal_flag_centers = thermal_detector->process_img(imgOriginal, altitude);
        for (Point pt : thermal_flag_centers) {
            send_to_db(pt, 20, thermalGPS, imgOriginal, 2);
        }
    }
    return 0;
}
