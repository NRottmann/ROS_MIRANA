/*
 * Node which records the marker recognized by the robot and stores them
 * into .csv files. Based on this files we can combine locations with
 * positions on the map
 */

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/UserData.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <signal.h>                     // Packages for shutdown commands
#include <ros/xmlrpc_manager.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


// Here we replace the SIGINT handle in order to be able to give some last commands after shut down ROS
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void mySigIntHandler(int sig){
        g_request_shutdown = 1;
}
// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result){
        int num_params = 0;
        if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
                num_params = params.size();
        if (num_params > 1) {
                std::string reason = params[1];
                ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
                g_request_shutdown = 1; // Set flag
        }
        result = ros::xmlrpc::responseInt(1, "", 0);
}


struct Marker {
        double id;
        double dist;
        double stamp;
        float x;
        float y;
};

class SubscribeAndPublish {

public:
SubscribeAndPublish(){
        //Topics to publish
        pubUserData = nh.advertise<rtabmap_ros::UserData>("marker_signal", 1);
        idSignalCloudPub = nh.advertise<sensor_msgs::PointCloud2>("id_signals", 1);

        //Topics to subscribe
        subMarker = nh.subscribe("/ar_pose_marker", 1, &SubscribeAndPublish::markerCallback, this);
        subMap = nh.subscribe("/mapData", 1, &SubscribeAndPublish::mapCallback, this);
}

void saveInCsv() {
        ROS_INFO("Write to file");

        std::string path = ros::package::getPath("automap");
        std::string filename = "/datastore/id2pos.csv";
        path += filename;

        // create and open the .csv file
        std::ofstream file;
        file.open(path.c_str());

        file << "Id" << "," << "x"<< "," << "y" << std::endl;

        // Save all markers to csv
        for(std::map<double, Marker>::iterator iter=nearestMarkers.begin(); iter!=nearestMarkers.end(); ++iter) {
                Marker marker = iter->second;

                file << marker.id << "," << marker.x << "," << marker.y << std::endl;
        }

        file.close();

        ROS_INFO("Write to file end");
}

void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
        for (int i = 0; i < msg->markers.size(); i++) {

                double id = msg->markers[i].id;

                if (!(id == 6 || id == 11 || id == 10)){
                    continue;
                }
                
                double dist = getDist(msg->markers[i]);
                ros::Time stamp = msg->markers[i].header.stamp; //ros::Time::now();

                // Create user data [dist] with the value
                cv::Mat data(1, 3, CV_64FC1);
                data.at<double>(0) = id;
                data.at<double>(1) = dist;
                data.at<double>(2) = stamp.toSec();

                // publish message to user data
                rtabmap_ros::UserData dataMsg;
                dataMsg.header.frame_id = msg->markers[i].header.frame_id;
                dataMsg.header.stamp = stamp;
                rtabmap_ros::userDataToROS(data, dataMsg, false);
                pubUserData.publish<rtabmap_ros::UserData>(dataMsg);
        }

}

void mapCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg) {
        rtabmap::Transform mapToOdom;
        std::map<int, rtabmap::Transform> poses;
        std::multimap<int, rtabmap::Link> links;
        std::map<int, rtabmap::Signature> signatures;
        rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

        // handle the case where we can receive only latest data, or if all data are published
        for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter) {
                int id = iter->first;

                rtabmap::Signature & node = iter->second;

                nodeStamps_.insert(std::make_pair(node.getStamp(), node.id()));

                if(!node.sensorData().userDataCompressed().empty()) {

                        cv::Mat data;
                        node.sensorData().uncompressDataConst(0,0, 0, &data);

                        if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 3) {
                                // format [int id, double dist, double stamp]
                                Marker marker;
                                marker.id = data.at<double>(0);
                                marker.dist = data.at<double>(1);
                                marker.stamp = data.at<double>(2);
                                markers.insert(std::make_pair(marker.stamp, marker));

                        } else if(!data.empty()) {
                                ROS_ERROR("Wrong user data format for marker signal.");
                        }
                }
        }

        // for the logic below, we should keep only stamps for
        // nodes still in the graph (in case nodes are ignored when not moving)
        std::map<double, int> nodeStamps;
        for(std::map<double, int>::iterator iter=nodeStamps_.begin(); iter!=nodeStamps_.end(); ++iter) {
                std::map<int, rtabmap::Transform>::const_iterator jter = poses.find(iter->second);

                if(jter != poses.end()) {
                        nodeStamps.insert(*iter);
                }
        }

        if(markers.size() == 0) {
                ROS_WARN("No marker signal detected yet in user data of map data");
        }

        //============================
        // Add Marker symbols
        //============================

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledWifiSignals(new pcl::PointCloud<pcl::PointXYZRGB>);
        int id = 0;

        for(std::map<double, Marker>::iterator iter=markers.begin(); iter!=markers.end(); ++iter, ++id) {
                //ROS_INFO("Check stamp for %d", iter->second);

                // The Marker value may be taken between two nodes, interpolate its position.
                double stampMarker = iter->first;
                std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stampMarker); // lower bound of the stamp
                if(previousNode!=nodeStamps.end() && previousNode->first > stampMarker && previousNode != nodeStamps.begin()) {
                        --previousNode;
                }
                std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stampMarker); // upper bound of the stamp

                if(previousNode != nodeStamps.end() &&
                   nextNode != nodeStamps.end() &&
                   previousNode->second != nextNode->second &&
                   uContains(poses, previousNode->second) && uContains(poses, nextNode->second)) {

                        rtabmap::Transform poseA = poses.at(previousNode->second);
                        rtabmap::Transform poseB = poses.at(nextNode->second);
                        double stampA = previousNode->first;
                        double stampB = nextNode->first;
                        UASSERT(stampMarker>=stampA && stampMarker <=stampB);

                        rtabmap::Transform v = poseA.inverse() * poseB;
                        double ratio = (stampMarker-stampA)/(stampB-stampA);

                        v.x()*=ratio;
                        v.y()*=ratio;
                        v.z()*=ratio;

                        rtabmap::Transform markerPose = (poseA*v).translation(); // rip off the rotation


                        // Construct marker with pose on map
                        Marker marker = iter->second;

                        Marker newMarker;
                        newMarker.id = marker.id;
                        newMarker.dist = marker.dist;
                        newMarker.x = markerPose.x();
                        newMarker.y = markerPose.y();

                        // Add marker to nearest markers if not present or id is at a nearer distance
                        std::map<double, Marker>::iterator idx = nearestMarkers.find(marker.id);

                        if(idx == nearestMarkers.end()) {
                                nearestMarkers.insert(std::make_pair(newMarker.id, newMarker));

                        }else if((idx->second).dist >= newMarker.dist) {
                                idx->second = newMarker;
                        }
                }
        }
}

private:
ros::NodeHandle nh;
ros::Publisher pubUserData, idSignalCloudPub;
ros::Subscriber subMarker, subMap;
std::map<double, Marker> markers;
std::map<double, Marker> nearestMarkers;
std::map<double, int> nodeStamps_;

inline double getDist(ar_track_alvar_msgs::AlvarMarker marker) {
        double x = marker.pose.pose.position.x;
        double y = marker.pose.pose.position.y;
        double z = marker.pose.pose.position.z;
        return std::sqrt(x*x + y*y + z*z);
}

};

int main(int argc, char** argv){
		ROS_INFO("run1");
        // Initialize the ROS System
        ros::init(argc, argv, "marker2Map", ros::init_options::NoSigintHandler);    // Initialize this node to with noSigintHandle to add an own one
        signal(SIGINT, mySigIntHandler);
        // Override XMLRPC shutdown
        ros::XMLRPCManager::instance()->unbind("shutdown");
        ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

        ros::NodeHandle nh;

        //Create an object of class SubscribeAndPublish that will take care of everything
        SubscribeAndPublish SAPObject;

		ROS_INFO("run2");
        while (!g_request_shutdown) {
                ros::spinOnce();
        }
		ROS_INFO("run3");
        SAPObject.saveInCsv();

        ros::shutdown();

        return 0;
}
