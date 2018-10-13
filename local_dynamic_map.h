#ifndef LOCAL_DYNAMIC_MAP_H
#define LOCAL_DYNAMIC_MAP_H

#include "t109_app.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace T109 {

using std::shared_ptr;
using std::vector;
using std::map;
using std::make_pair;
using std::pair;

using ScenSim::SimulationEngineInterface;
using ScenSim::TimeType;
using ScenSim::NodeIdType;
using ScenSim::ObjectMobilityModel;

//LDM(Local Dynamic Map) class
class LocalDynamicMap: public std::enable_shared_from_this<LocalDynamicMap>
{
public:
    LocalDynamicMap(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const NodeIdType& initNodeId);

	  ~LocalDynamicMap();

    bool AddEntry(
		    const NodeIdType& nodeId,
		    TimeType sendTime,
		    int latitude,
		    int longitude,
		    short int speed,
		    unsigned short int heading,
        short int altitude,
        double n,
        double receiveRssiDbm,
        double P,
        TimeType receiveTime);

    // データ表示用
    void Clear();
    void PrintForTest();
    void PrintDistance(const NodeIdType& nodeId, int iter, double n);

    // 提案手法
    void KalmanfilterForVehicleToVehicle(
        const NodeIdType& nodeId,
        int latitude,
        int longitude,
        double& receiveRssiDbm,
        double& P,
        TimeType receiveTime);
    void KalmanfilterForPedestrian(
        const NodeIdType& nodeId,
        int latitude,
        int longitude,
        double& receiveRssiDbm,
        double& P,
        TimeType receiveTime);
    double CalculatePassLossIndex(const NodeIdType& nodeId, TimeType delay, int latitude, int longitude, double receiveRssiDbm);
    double DeterminePassLossIndex(double latitude, double longitude); // 重みづけなし
    void Trilateration();
    void WeightedCentroidLocalization();
    void LeastSquaresMethod();

    int GetBeaconCoordinateX(const NodeIdType& nodeId);
    int GetBeaconCoordinateY(const NodeIdType& nodeId);

    // 位置推定結果を取得
    double GetMyLatitude();
    double GetMyLongtitude();
    TimeType GetTrilaterationTime();

    NodeIdType GetMyNodeId();
private:
    double GetMyPositionX();
    double GetMyPositionY();

    double GetPositionX(const NodeIdType& nodeId, int iter);
    double GetPositionY(const NodeIdType& nodeId, int iter);

    double CalculateDistance(double x1, double y1, double x2, double y2); // 2点間の距離を算出
    double EstimateDistance(const NodeIdType& nodeId, double receiveRssiDbm, double n); // 推定距離の算出

    double convertCMtoM(unsigned short int distance);
    double convertMMtoM(int pos);
    TimeType convertMilliToNanoSec(unsigned int milliSecond);

    void DeleteEntry(const NodeIdType& nodeId);
    bool isEntry(const NodeIdType& nodeId);

    struct BeaconCoordinate{
        int latitude;
        int longitude;

        BeaconCoordinate(const double initLatitude, const double initLongitude)
        :
        latitude(initLatitude),
        longitude(initLongitude)
        {}
    };

    //VehicleInformation//
    struct VehicleInformation {
          TimeType sendTime;
          int latitude;
          int longitude;
          short int speed;
          unsigned short int heading;
          short int altitude;
          double n;
          double receiveRssiDbm;
          double P;
          TimeType receiveTime;

          VehicleInformation(
              const TimeType initSendTime,
              const int initLatitude,
              const int initLongitude,
              const short int initSpeed,
              const unsigned short int initHeading,
              const short int initAltitude,
              const double initn,
              const double initReceiveRssiDbm,
              const double initP,
              const TimeType initReceiveTime)
              :
              sendTime(initSendTime),
              latitude(initLatitude),
              longitude(initLongitude),
              speed(initSpeed),
              heading(initHeading),
              altitude(initAltitude),
              n(initn),
              receiveRssiDbm(initReceiveRssiDbm),
              P(initP),
              receiveTime(initReceiveTime)
              {}

    };//VehicleInformation//

    // ソート用の構造体
    struct DistanceInfo{
        NodeIdType nodeId;
        double latitude;
        double longitude;
        double distance;
        double error;

        bool operator<(const DistanceInfo& right) const
        {
            return this->distance < right.distance;
        }
        bool operator>(const DistanceInfo& right) const
        {
            return this->distance > right.distance;
        }
    };
    void SortTable(vector<DistanceInfo>& distance_table); // 昇順ソート

    // 座標用の構造体
    struct Point{
        double x;
        double y;

        Point(const double initx, const double inity):x(initx), y(inity){}
    };

    // 推定結果
    double myLatitude;
    double myLongitude;
    TimeType trilaterationTime;

    // 基準RSSI
    double AForCar;
    double AForBeacon;
    double THREWSHOLD;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<ObjectMobilityModel> mobilityModelPtr;
    NodeIdType myNodeId;
    map<NodeIdType, vector<VehicleInformation>> local_table;
    map<NodeIdType, BeaconCoordinate> beacon_table;

    /*// interval_time後の座標を算出
    double CalculateMyPositionX(
        double x,
        double speed,
        double heading,
        TimeType interval_time);
    double CalculateMyPositionY(
        double y,
        double speed,
        double heading,
        TimeType interval_time);*/

};//LocalDynamicMap//

}//namespace

#endif
