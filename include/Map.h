/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
//没用注释的用来建图
//sky,person,rider可以用来作为Outline track
    enum eSegmentLable{
        ROAD = 0, //点很多,分割的效果比较好，可用
        SIDEWAILK = 1,//可用
        BUILDING = 2,//远处不是很准，可用于场景重建，可用。
//    WALL = 3,  //有时会把围栏当做墙（05），这个比较不准，
//    FENCE = 4, //垃圾桶会被当做Fence，大部分能割出来，不全
                POLE = 5,  //比较有特点，可以尝试踢掉视差图中的outliner
//    TRAFFIC_LIGHT=6, //太弱，建议去掉
//    TRAFFIC_SIGN =7,//交通牌，不是很稳定，数量比较稀少，暂时不考虑加入。
//    VEGETATION = 8,//主要是树，由于树比较容易变化，不建议用于地图场景
//    TERRAIN = 9,  //这个很少，也不稳定，建议去掉
                SKY = 10,
        PERSION = 11,
        RIDER=12,
        CAR=13,//可用
        TRUCK=14, //分割非常不稳定，建议与car统一，可用
        BUS=15,
        TRAIN=16,
        MOTORCYCLE=17,
        BICYCLE=18
    };
class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
