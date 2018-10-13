#include "local_dynamic_map.h"

namespace T109 {

	//LocalDynamicMap//
	LocalDynamicMap::LocalDynamicMap(
		const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
		const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
		const NodeIdType& initNodeId)
		:
		simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
		mobilityModelPtr(initNodeMobilityModelPtr),
		myNodeId(initNodeId),
		trilaterationTime(0),
		myLatitude(0),
		myLongitude(0),
		AForCar(-10.816),
		AForBeacon(-61),
		THREWSHOLD(-50)
		{
		// 歩行者はビーコンデータベースの初期化（ビーコンの位置情報を全て知っている）
		if (myNodeId == 200) {
			// 20m
			/*BeaconCoordinate b1(10200, 30000), b3(10200, 10200), b6(30000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(151, b1));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(153, b3));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(156, b6));

			BeaconCoordinate b7(-10200, 30000), b9(-10200, 10200), b12(-30000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(157, b7));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(159, b9));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(162, b12));

			BeaconCoordinate b13(-10200, -30000), b15(-10200, -10200), b18(-30000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(163, b13));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(165, b15));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(168, b18));

			BeaconCoordinate b19(10200, -30000), b21(10200, -10200), b24(30000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(169, b19));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(171, b21));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(174, b24));*/

			// 15m
			/*eaconCoordinate b1(10200, 32500), b3(10200, 17500), b4(17500, 10200), b6(32500, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(151, b1));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(153, b3));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(154, b4));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(156, b6));

			BeaconCoordinate b7(-10200, 32500), b9(-10200, 17500), b10(-17500, 10200), b12(-32500, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(157, b7));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(159, b9));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(160, b10));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(162, b12));

			BeaconCoordinate b13(-10200, -32500), b15(-10200, -17500), b16(-17500, -10200), b18(-32500, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(163, b13));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(165, b15));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(166, b16));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(168, b18));

			BeaconCoordinate b19(10200, -32500), b21(10200, -17500), b22(17500, -10200), b24(32500, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(169, b19));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(171, b21));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(172, b22));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(174, b24));*/

			// 10m
			BeaconCoordinate b1(10200, 35000), b2(10200, 25000), b3(10200, 15000), b4(15000, 10200), b5(25000, 10200), b6(35000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(151, b1));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(152, b2));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(153, b3));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(154, b4));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(155, b5));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(156, b6));

			BeaconCoordinate b7(-10200, 35000), b8(-10200, 25000), b9(-10200, 15000), b10(-15000, 10200), b11(-25000, 10200), b12(-35000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(157, b7));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(158, b8));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(159, b9));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(160, b10));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(161, b11));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(162, b12));

			BeaconCoordinate b13(-10200, -35000), b14(-10200, -25000), b15(-10200, -15000), b16(-15000, -10200), b17(-25000, -10200), b18(-35000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(163, b13));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(164, b14));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(165, b15));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(166, b16));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(167, b17));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(168, b18));

			BeaconCoordinate b19(10200, -35000), b20(10200, -25000), b21(10200, -15000), b22(15000, -10200), b23(25000, -10200), b24(35000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(169, b19));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(170, b20));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(171, b21));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(172, b22));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(173, b23));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(174, b24));

			// 5m
			/*BeaconCoordinate b25(10200, 30000), b26(10200, 20000), b27(10200, 10000), b28(20000, 10200), b29(30000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(175, b25));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(176, b26));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(177, b27));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(178, b28));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(179, b29));

			BeaconCoordinate b30(-10200, 30000), b31(-10200, 20000), b32(-10200, 10000), b33(-20000, 10200), b34(-30000, 10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(180, b30));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(181, b31));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(182, b32));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(183, b33));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(184, b34));

			BeaconCoordinate b35(-10200, -30000), b36(-10200, -20000), b37(-10200, -10000), b38(-20000, -10200), b39(-30000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(185, b35));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(186, b36));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(187, b37));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(188, b38));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(189, b39));

			BeaconCoordinate b40(10200, -30000), b41(10200, -20000), b42(10200, -10000), b43(20000, -10200), b44(30000, -10200);
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(190, b40));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(191, b41));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(192, b42));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(193, b43));
			beacon_table.insert(std::pair<NodeIdType, BeaconCoordinate>(194, b44));*/
		}
		Clear();

	}//LocalDynamicMap//

	//~LocalDynamicMap//
	LocalDynamicMap::~LocalDynamicMap()
	{
	}//~LocalDynamicMap//

	//AddEntry//
	bool LocalDynamicMap::AddEntry(
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
		TimeType receiveTime)
		{
		if(myNodeId == nodeId) {
			std::cout << "same" << endl;
			return false;
		}
		VehicleInformation viList(sendTime, latitude, longitude, speed, heading, altitude, n, receiveRssiDbm, P, receiveTime);

		if (isEntry(nodeId)) {
			map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);

			// 送信時刻が最終送信時刻より早い
			if (it->second.back().sendTime >= sendTime) {
				std::cout << nodeId << ":失敗しました" << it->second.back().sendTime << ", " << sendTime << endl;
				return false;
			}

			// 車両は過去5つのデータを保持
			if (it->first <= 150) {
				if (it->second.size() == 5) {
					it->second.erase(it->second.begin());
				}
				it->second.push_back(viList);

			// ビーコンは過去２つのデータを保持
			} else {
				if (it->second.size() == 2) {
					it->second.erase(it->second.begin());
				}
				it->second.push_back(viList);
			}
		} else {
			vector<VehicleInformation> vehicleInformationList;
			vehicleInformationList.push_back(viList);
			local_table.insert(std::pair<NodeIdType, vector<VehicleInformation>>(nodeId, vehicleInformationList)); // 車載情報の追加
		}
		return true;
	}//AddEntry//

	//Clear//
	void LocalDynamicMap::Clear()
	{
		local_table.clear();
	}//Clear//

	//CalculatePassLossIndex//
	double LocalDynamicMap::CalculatePassLossIndex(
		const NodeIdType& nodeId,
		TimeType delay,
		int latitude,
		int longitude,
		double receiveRssiDbm)
		{
		// 範囲内の電波強度を利用
		if (receiveRssiDbm < AForCar && receiveRssiDbm > THREWSHOLD) {
			double distance = CalculateDistance(convertMMtoM(latitude), convertMMtoM(longitude), GetMyPositionX(), GetMyPositionY());
			double n = (AForCar - receiveRssiDbm) / (10 * log10(distance)); // 車両間の伝搬損失指数の算出

			if (distance == 1) {
				return 2; // 距離が1mの時に車両間の伝搬損失指数は求められない
			} else if (n <= 0 || n > 4) {
				return 0;
			} else {
				return n;
			}
		} else {
			return 0;
		}
	}//CalculatePassLossIndex//

	//DeterminePassLossIndex//
	double LocalDynamicMap::DeterminePassLossIndex(
		double latitude,
		double longitude)
		{
		/*ostringstream oss1;
		oss1 << "伝搬損失指数の決定.txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);*/

		double n = 0, cnt = 0;
		map<NodeIdType, vector<VehicleInformation>>::iterator it;
		for(it = local_table.begin(); it != local_table.end(); it++) {
			if(it->first != myNodeId) {
				// 新しい順にデータを見ていく
				for (int i = it->second.size() - 1; i >= 0; i--) {
					VehicleInformation viList = it->second[i];
					if ( (simulationEngineInterfacePtr->CurrentTime() - viList.receiveTime) <= 500 * MILLI_SECOND) { // 500msが一番いいかの実験結果を示す
						if (viList.n != 0) {
							n += viList.n;
							cnt++;
							//if (myNodeId == 1) ofs1 << viList.receiveRssiDbm << ", " << viList.n << endl;
						}
					} else {
						break;
					}
				}
			}
		}
		if(cnt != 0) {
			//if (myNodeId == 1) ofs1 << endl;
			return n / cnt;
		} else {
			return 2.0;
		}
	}//DeterminePassLossIndex//

	//KalmanfilterForPedestrian//
	void LocalDynamicMap::KalmanfilterForPedestrian(
		const NodeIdType& nodeId,
		int latitude,
		int longitude,
		double& receiveRssiDbm,
		double& P,
		TimeType receiveTime)
		{
		/*ostringstream oss1;
		oss1 << "フィルタリング前" << nodeId << ".txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);
		ofs1 << receiveRssiDbm << endl;*/

		double xhatm, xhat, Pm, G;
		double Q = 0, R = 0;

		// 現在時刻の速度の取得
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp;
		mobilityModelPtr->GetPositionForTime(currentTime,omp);
		double currentSpeed = omp.VelocityMetersPerSecond();

		// 静止中
		if (currentSpeed == 0) {
			if (nodeId <= 150) {
				Q = 4.44;
				R = 25.44;
			} else {
				Q = 0.00046;
				R = 19.0454;
			}
			// 移動中
		} else {
			if (nodeId <= 150) {
				Q = 5.37;
				R = 27;
			} else {
				Q = 5.41;
				R = 13.3;
			}
		}

		if (isEntry(nodeId)){
			map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
			VehicleInformation viList = it->second.back();

			if (nodeId <= 150 && (receiveTime - viList.receiveTime) > 200 * MILLI_SECOND) {
				xhat = 0;
				P = 1000;
			} else if (nodeId > 150 && (receiveTime - viList.receiveTime) > (2 * SECOND)) {
				xhat = 0;
				P = 1000;
			} else {
				xhat = viList.receiveRssiDbm;
				P = viList.P;
			}
		} else {
			xhat = 0;
			P = 1000;
		}

		xhatm = xhat;
		Pm = P + Q;
		G = Pm / (Pm + R);
		receiveRssiDbm = xhatm + G * (receiveRssiDbm - xhatm);
		P = (1 - G) * Pm;

		/*ostringstream oss2;
		oss2 << "フィルタリング後" << nodeId << ".txt";
		std::ofstream ofs2(oss2.str(), std::ios::out | std::ios::app);
		ofs2 << receiveRssiDbm << endl;*/
	}//KalmanfilterForPedestrian//

	//WeightedCentroidLocalization//
	void LocalDynamicMap::WeightedCentroidLocalization()
	{
		ostringstream oss1;
		oss1 << "nの評価（車両）.txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);

		ostringstream oss2;
		oss2 << myNodeId << "WCL結果.txt";
		std::ofstream ofs2(oss2.str(), std::ios::out | std::ios::app);

		ostringstream oss3;
		oss3 << "nの評価（ビーコン）.txt";
		std::ofstream ofs3(oss3.str(), std::ios::out | std::ios::app);

		// 現在時刻の速度の取得
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp1;
		mobilityModelPtr->GetPositionForTime(currentTime,omp1);
		double currentSpeed = omp1.VelocityMetersPerSecond();

		// 1時刻前の速度の取得
		const TimeType previousTime = GetTrilaterationTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp2;
		mobilityModelPtr->GetPositionForTime(previousTime,omp2);
		double previousSpeed = omp2.VelocityMetersPerSecond();

		// リストの作成
		vector<DistanceInfo> distance_table;
		DistanceInfo distanceInfo;
		double beaconCnt = 0, carCnt = 0;

		map<NodeIdType, vector<VehicleInformation>>::iterator it;
		for(it = local_table.begin(); it != local_table.end(); it++) {
			// 歩行者が静止中
			if ( (previousSpeed == 0) && (currentSpeed == 0) ) {
				int tmpi = 0, max = -1000;

				// ビーコンまたは車が静止中の時は過去１sで電波強度が最大のデータを取得
				if ( (it->second.front().speed == 0 && it->second.back().speed == 0) ) {
					// 新しい順にデータを見ていく
					for (int i = it->second.size() - 1; i >= 0; i--) {
						VehicleInformation viList = it->second[i];
						if ( (currentTime - viList.receiveTime) <= SECOND ) {
							if (viList.receiveRssiDbm > max && viList.receiveRssiDbm > -81) {
								tmpi = i;
								max = viList.receiveRssiDbm;
							}
						} else {
							break;
						}
					}
					// ビーコンの電波強度が最大のデータが見つかった
					if ( (it->first > 150) && (max != -1000) ) {
						distanceInfo.nodeId = it->first;
						distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n);
						distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
						distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
						PrintDistance(it->first, tmpi, it->second[tmpi].n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

					// 車の電波強度が最大のデータが見つかった
					} else if ( (it->first <= 150) && (max != -1000) ) {
						if (it->second[tmpi].receiveRssiDbm > THREWSHOLD) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
							distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
							distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n); // 推定距離;
							PrintDistance(it->first, tmpi, it->second[tmpi].n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}

					// 移動中の車の電波強度
					} else {
						VehicleInformation viList = it->second.back(); // 最新のデータを取得

						if (viList.receiveTime > GetTrilaterationTime()) {
							if (viList.receiveRssiDbm > THREWSHOLD) {
								distanceInfo.nodeId = it->first;
								distanceInfo.latitude = convertMMtoM(viList.latitude);
								distanceInfo.longitude = convertMMtoM(viList.longitude);
								distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n); // 推定距離;
								PrintDistance(it->first, it->second.size()-1, viList.n);
								distance_table.push_back(distanceInfo);
								carCnt++;
							}
						}
					}

			// 歩行者が移動中は最新のデータを取得
			} else {
				VehicleInformation viList = it->second.back(); // 最新のデータを取得

				if (viList.receiveTime > GetTrilaterationTime()) {
					// ビーコンの電波強度
					if (it->first > 150 && viList.receiveRssiDbm > -81) {
						distanceInfo.nodeId = it->first;
						distanceInfo.latitude = convertMMtoM(viList.latitude);
						distanceInfo.longitude = convertMMtoM(viList.longitude);
						distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
						PrintDistance(it->first, it->second.size()-1, viList.n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

						// 車の電波強度
					} else {
						if (viList.receiveRssiDbm > THREWSHOLD) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(viList.latitude);
							distanceInfo.longitude = convertMMtoM(viList.longitude);
							distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
							PrintDistance(it->first, it->second.size()-1, viList.n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}
				}
			}
		}
		ofs1 << endl;
		SortTable(distance_table); // 距離で昇順ソート

		// WCL法メイン
		double weight = 0, x = 0, y = 0, sumWeight = 0;
		double size = 0;
		/*if (distance_table.size() > 3) {
			size = 3;
		} else {
			size = distance_table.size();
		}*/
		for (int t = 0; t < distance_table.size(); t++) {
			weight = pow(1 / distance_table[t].distance, 1.5); // この重みにした根拠は？→実験から
			x += weight * distance_table[t].latitude;
			y += weight * distance_table[t].longitude;
			sumWeight += weight;
		}
		if (sumWeight != 0) {
			myLatitude = x / sumWeight;
			myLongitude = y / sumWeight;
		}

		trilaterationTime = currentTime;
		ofs2 << CalculateDistance(myLatitude, myLongitude, GetMyPositionX(), GetMyPositionY()) << " " << carCnt << " " << beaconCnt << endl;
		//ofs2 << myLatitude << " " << myLongitude << " " << GetMyPositionX() << " " << GetMyPositionY() << endl;
	}//WeightedCentroidLocalization//

	//LeastSquaresMethod//
	void LocalDynamicMap::LeastSquaresMethod()
	{
		ostringstream oss1;
		oss1 << "nの評価（車両）.txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);

		ostringstream oss2;
		oss2 << myNodeId << "LS結果.txt";
		std::ofstream ofs2(oss2.str(), std::ios::out | std::ios::app);

		ostringstream oss3;
		oss3 << "nの評価（ビーコン）.txt";
		std::ofstream ofs3(oss3.str(), std::ios::out | std::ios::app);

		// 現在時刻の速度の取得
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp1;
		mobilityModelPtr->GetPositionForTime(currentTime,omp1);
		double currentSpeed = omp1.VelocityMetersPerSecond();

		// 1時刻前の速度の取得
		const TimeType previousTime = GetTrilaterationTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp2;
		mobilityModelPtr->GetPositionForTime(previousTime,omp2);
		double previousSpeed = omp2.VelocityMetersPerSecond();

		// リストの作成
		vector<DistanceInfo> distance_table;
		DistanceInfo distanceInfo;
		double beaconCnt = 0, carCnt = 0;

		map<NodeIdType, vector<VehicleInformation>>::iterator it;
		for(it = local_table.begin(); it != local_table.end(); it++) {
			// 歩行者が静止中
			if ( (previousSpeed == 0) && (currentSpeed == 0) ) {
				int tmpi = 0, max = -1000;

				// ビーコンまたは車が静止中の時は過去１sで電波強度が最大のデータを取得
				if ( (it->second.front().speed == 0 && it->second.back().speed == 0) ) {
					// 新しい順にデータを見ていく
					for (int i = it->second.size() - 1; i >= 0; i--) {
						VehicleInformation viList = it->second[i];
						if ( (currentTime - viList.receiveTime) <= SECOND ) {
							if (viList.receiveRssiDbm > max && viList.receiveRssiDbm > -81) {
								tmpi = i;
								max = viList.receiveRssiDbm;
							}
						} else {
							break;
						}
					}
					// ビーコンの電波強度が最大のデータが見つかった
					if ( (it->first > 150) && (max != -1000) ) {
						distanceInfo.nodeId = it->first;
						distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
						distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
						distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n);
						PrintDistance(it->first, tmpi, it->second[tmpi].n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

					// 車の電波強度が最大のデータが見つかった
					} else if ( (it->first <= 150) && (max != -1000) ) {
						if (it->second[tmpi].receiveRssiDbm > -40) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
							distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
							distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n); // 推定距離;
							PrintDistance(it->first, tmpi, it->second[tmpi].n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}

					// 移動中の車の電波強度
					} else {
						VehicleInformation viList = it->second.back(); // 最新のデータを取得

						if (viList.receiveTime > GetTrilaterationTime()) {
							if (viList.receiveRssiDbm > -40) {
								distanceInfo.nodeId = it->first;
								distanceInfo.latitude = convertMMtoM(viList.latitude);
								distanceInfo.longitude = convertMMtoM(viList.longitude);
								distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n); // 推定距離;
								PrintDistance(it->first, it->second.size()-1, viList.n);
								distance_table.push_back(distanceInfo);
								carCnt++;
							}
						}
					}

			// 歩行者が移動中は最新のデータを取得
			} else {
				VehicleInformation viList = it->second.back(); // 最新のデータを取得

				if (viList.receiveTime > GetTrilaterationTime()) {
					// ビーコンの電波強度
					if (it->first > 150 && viList.receiveRssiDbm > -81) {
						distanceInfo.nodeId = it->first;
						distanceInfo.latitude = convertMMtoM(viList.latitude);
						distanceInfo.longitude = convertMMtoM(viList.longitude);
						distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
						PrintDistance(it->first, it->second.size()-1, viList.n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

						// 車の電波強度
					} else {
						if (viList.receiveRssiDbm > -40) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(viList.latitude);
							distanceInfo.longitude = convertMMtoM(viList.longitude);
							distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
							PrintDistance(it->first, it->second.size()-1, viList.n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}
				}
			}
		}
		ofs1 << endl;
		ofs3 << endl;

		SortTable(distance_table); // 距離で昇順ソート
		const int tableSize = distance_table.size(); // テーブルの要素数
		double x0, y0 = 0; // ユーザの位置
		vector<double> matrix_r;
		vector< vector<double> > matrix_a, matrix_trans_aw, matrix_aa;
		double matrix_x[2] = {};
		int indic = 0; // ニュートン法の反復回数

		matrix_r.resize(tableSize); // n*1
		matrix_a.resize(tableSize); // n*2
		matrix_trans_aw.resize(2); // 2*n
		matrix_aa.resize(2); // 2*2
		for (int i = 0; i < tableSize; i++) {
			matrix_a[i].resize(2);
		}

		for (int i = 0; i < 2; i++) {
			matrix_trans_aw[i].resize(tableSize);
			matrix_aa[i].resize(2);
		}

		// 初期位置の決定
		for (int i = 0; i < tableSize; i++) {
			x0 += distance_table[i].latitude;
			y0 += distance_table[i].longitude;
		}
		x0 = x0 / tableSize;
		y0 = y0 / tableSize;

		if (tableSize >= 3) {
			// ニュートン法
			do {
				// 行列 A と R を作成 (n*2, n*1)
				for (int i = 0; i < tableSize; i++) {
					double initdistance = sqrt(pow(distance_table[i].latitude - x0, 2) + pow(distance_table[i].longitude - y0, 2));
					matrix_a[i][0] = -(distance_table[i].latitude - x0) / initdistance;
					matrix_a[i][1] = -(distance_table[i].longitude - y0) / initdistance;
					matrix_r[i] = distance_table[i].distance - initdistance;
				}

				// A の転置行列（At）* W を作成（2*n）
				for (int i = 0; i < tableSize; i++) {
					for (int j = 0; j < 2; j++) {
						matrix_trans_aw[j][i] = matrix_a[i][j] / (distance_table[i].distance);
					}
				}

				// AtW * A (2*n * n*2 = 2*2)
				for (int i = 0; i < 2; i++) {
					for (int j = 0; j < 2; j++) {
						double sum = 0;
						for (int k = 0; k < tableSize; k++) {
							sum += matrix_trans_aw[i][k] * matrix_a[k][j];
						}
						matrix_aa[i][j] = sum;
					}
				}

				// AtW * A の逆行列 (2*2)
				double matrix_inv_aa[2][2] = {};
				double det = matrix_aa[0][0] * matrix_aa[1][1] - matrix_aa[0][1] * matrix_aa[1][0];
				matrix_inv_aa[0][0] = matrix_aa[1][1];
				matrix_inv_aa[0][1] = -matrix_aa[0][1];
				matrix_inv_aa[1][0] = -matrix_aa[1][0];
				matrix_inv_aa[1][1] = matrix_aa[0][0];

				// AtW * R (2*n * n*1 = 2*1)
				double matrix_ar[2] = {};
				for (int i = 0; i < 2; i++) {
					double sum = 0;
					for (int j = 0; j < tableSize; j++) {
						sum += matrix_trans_aw[i][j] * matrix_r[j];
					}
					matrix_ar[i] = sum;
				}

				// X の算出 (2*2 * 2*1 = 2*1)
				for (int i = 0; i < 2; i++) {
					double sum = 0;
					for (int j = 0; j < 2; j++) {
						sum += matrix_inv_aa[i][j] * matrix_ar[j];
					}
					matrix_x[i] = sum / det;
				}

				if (fabs(matrix_x[0]) <= 0.01 && fabs(matrix_x[1]) <= 0.01) break;
				x0 += matrix_x[0];
				y0 += matrix_x[1];

				indic++;
				if (indic == 20) break;
			} while(true);
		} else {
			ofs2 << "サイズが小さすぎます, size = " << distance_table.size() << " ";
		}
		myLatitude = x0;
		myLongitude = y0;
		trilaterationTime = currentTime;
		ofs2 << CalculateDistance(myLatitude, myLongitude, GetMyPositionX(), GetMyPositionY()) << " " << indic << " " << tableSize << endl;
	}//LeastSquaresMethod//

	//Trilateration//
	void LocalDynamicMap::Trilateration()
	{
		ostringstream oss1;
		oss1 << "nの評価（車両）.txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);

		ostringstream oss2;
		oss2 << myNodeId << "Trilateration結果.txt";
		std::ofstream ofs2(oss2.str(), std::ios::out | std::ios::app);

		ostringstream oss3;
		oss3 << "nの評価（ビーコン）.txt";
		std::ofstream ofs3(oss3.str(), std::ios::out | std::ios::app);

		// 現在時刻の速度の取得
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp1;
		mobilityModelPtr->GetPositionForTime(currentTime,omp1);
		double currentSpeed = omp1.VelocityMetersPerSecond();

		// 1時刻前の速度の取得
		const TimeType previousTime = GetTrilaterationTime(); // NanoSec
		ScenSim::ObjectMobilityPosition omp2;
		mobilityModelPtr->GetPositionForTime(previousTime,omp2);
		double previousSpeed = omp2.VelocityMetersPerSecond();

		// リストの作成
		vector<DistanceInfo> distance_table;
		DistanceInfo distanceInfo;
		double beaconCnt = 0, carCnt = 0;

		map<NodeIdType, vector<VehicleInformation>>::iterator it;
		for(it = local_table.begin(); it != local_table.end(); it++) {
			// 歩行者が静止中
			if ( (previousSpeed == 0) && (currentSpeed == 0) ) {
				int tmpi = 0, max = -1000;

				// ビーコンまたは車が静止中の時は過去１sで電波強度が最大のデータを取得
				if ( (it->second.front().speed == 0 && it->second.back().speed == 0) ) {
					// 新しい順にデータを見ていく
					for (int i = it->second.size() - 1; i >= 0; i--) {
						VehicleInformation viList = it->second[i];
						if ( (currentTime - viList.receiveTime) <= SECOND ) {
							if (viList.receiveRssiDbm > max && viList.receiveRssiDbm > -81) {
								tmpi = i;
								max = viList.receiveRssiDbm;
							}
						} else {
							break;
						}
					}
					// ビーコンの電波強度が最大のデータが見つかった
					if ( (it->first > 150) && (max != -1000) ) {
						distanceInfo.nodeId = it->first;
						distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
						distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
						distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n);
						distanceInfo.error = -(it->second[tmpi].receiveRssiDbm) / 100;
						PrintDistance(it->first, tmpi, it->second[tmpi].n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

						// 車の電波強度が最大のデータが見つかった
					} else if ( (it->first <= 150) && (max != -1000) ) {
						if (it->second[tmpi].receiveRssiDbm > -40) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(it->second[tmpi].latitude);
							distanceInfo.longitude = convertMMtoM(it->second[tmpi].longitude);
							distanceInfo.distance = EstimateDistance(it->first, it->second[tmpi].receiveRssiDbm, it->second[tmpi].n); // 推定距離;
							distanceInfo.error = -(it->second[tmpi].receiveRssiDbm) / 100;
							PrintDistance(it->first, tmpi, it->second[tmpi].n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}

				// 移動中の車の電波強度
				} else {
					VehicleInformation viList = it->second.back(); // 最新のデータを取得

					if (viList.receiveTime > GetTrilaterationTime()) {
						if (viList.receiveRssiDbm > -40) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(viList.latitude);
							distanceInfo.longitude = convertMMtoM(viList.longitude);
							distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n); // 推定距離;
							distanceInfo.error = -(viList.receiveRssiDbm) / 100;
							PrintDistance(it->first, it->second.size()-1, viList.n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}
				}

			// 歩行者が移動中は最新のデータを取得
			} else {
				VehicleInformation viList = it->second.back(); // 最新のデータを取得

				if (viList.receiveTime > GetTrilaterationTime()) {
					// ビーコンの電波強度
					if (it->first > 150 && viList.receiveRssiDbm > -81) {
						distanceInfo.nodeId = it->first;
						distanceInfo.latitude = convertMMtoM(viList.latitude);
						distanceInfo.longitude = convertMMtoM(viList.longitude);
						distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
						distanceInfo.error = -(viList.receiveRssiDbm) / 100;
						PrintDistance(it->first, it->second.size()-1, viList.n);
						distance_table.push_back(distanceInfo);
						beaconCnt++;

						// 車の電波強度
					} else {
						if (viList.receiveRssiDbm > -40) {
							distanceInfo.nodeId = it->first;
							distanceInfo.latitude = convertMMtoM(viList.latitude);
							distanceInfo.longitude = convertMMtoM(viList.longitude);
							distanceInfo.distance = EstimateDistance(it->first, viList.receiveRssiDbm, viList.n);
							distanceInfo.error = -(viList.receiveRssiDbm) / 100;
							PrintDistance(it->first, it->second.size()-1, viList.n);
							distance_table.push_back(distanceInfo);
							carCnt++;
						}
					}
				}
			}
		}
		//ofs1 << endl;

		SortTable(distance_table); // 距離で昇順ソート
		if (distance_table.size() >= 1) {
			// 解空間の設定
			vector<Point> pointList, tempList; // 解候補リスト
			Point point(0, 0); // 座標オブジェクト
			for (double i = -50; i <= 50; i += 0.2) {
				for (double j = -50; j <= 50; j += 0.2) {
					point.x = i;
					point.y = j;
					pointList.push_back(point);
				}
			}

			double dif, min; // 評価関数
			Point temp(0,0), result(0,0);

			// ノードごとの解の組合せを探す
			for (int t = 0; t < distance_table.size(); t++) {
				min = 100;
				// 解候補の座標を一つずつ, 評価関数により検証
				ofs2 << pointList.size() << endl;
				for (int i = 0; i < pointList.size(); i++) {
					dif = pow((pointList[i].x - distance_table[t].latitude), 2) + pow((pointList[i].y - distance_table[t].longitude), 2)
					- pow(distance_table[t].distance, 2);

					if (dif <= distance_table[t].error) {
						temp.x = pointList[i].x; // 解の組み合わせを算出
						temp.y = pointList[i].y; // 解の組み合わせを算出
						ofs2 << temp.x << " " << temp.y << endl;
						tempList.push_back(temp);
					}
				}
				if (tempList.empty()) {
					break; // 解が見つからなかった
				}
				pointList = tempList; // リストの更新
				tempList.erase(tempList.begin(), tempList.end());
			}
		} else {
			ofs2 << "サイズが小さすぎます, size = " << distance_table.size() << endl << endl;
			myLatitude = GetMyLatitude();
			myLongitude = GetMyLongtitude();
		}
		trilaterationTime = simulationEngineInterfacePtr->CurrentTime();
		ofs3 << sqrt(pow(fabs(myLatitude-GetMyPositionX()), 2) + pow(fabs(myLongitude-GetMyPositionY()), 2)) << endl;
	}//Trilateration//

	//GetMyLatitude//
	double LocalDynamicMap::GetMyLatitude()
	{
		return myLatitude;
	}//GetMyLatitude//

	//GetMyLongtitude//
	double LocalDynamicMap::GetMyLongtitude()
	{
		return myLongitude;
	}//GetMyLongtitude//

	//GetTrilaterationTime//
	TimeType LocalDynamicMap::GetTrilaterationTime()
	{
		return trilaterationTime;
	}//GetTrilaterationTime//

	//SortTable//
	void LocalDynamicMap::SortTable(vector<DistanceInfo>& distance_table)
	{
		sort(distance_table.begin(), distance_table.end());
	}//SortTable//

	//GetMyPositionX//
	double LocalDynamicMap::GetMyPositionX()
	{
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
		ScenSim::ObjectMobilityPosition omp;
		mobilityModelPtr->GetPositionForTime(currentTime, omp); // M

		return omp.X_PositionMeters();
	}//GetMyPositionX//

	//GetMyPositionY//
	double LocalDynamicMap::GetMyPositionY()
	{
		const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
		ScenSim::ObjectMobilityPosition omp;
		mobilityModelPtr->GetPositionForTime(currentTime,omp); // M

		return omp.Y_PositionMeters();
	}//GetMyPositionY//

	//GetPositionX//
	double LocalDynamicMap::GetPositionX(const NodeIdType& nodeId, int iter)
	{
		if (isEntry(nodeId)){
			map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
			VehicleInformation viList = it->second[iter];
			return convertMMtoM(viList.latitude);
		}

		return 0;
	}//GetPositionX//

	//GetPositionY//
	double LocalDynamicMap::GetPositionY(const NodeIdType& nodeId, int iter)
	{
		if (isEntry(nodeId)){
			map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
			VehicleInformation viList = it->second[iter];
			return convertMMtoM(viList.longitude);
		}
		return 0;
	}//GetPositionY//

	//CalculateDistance//
	double LocalDynamicMap::CalculateDistance(double x1, double y1, double x2, double y2)
	{
		return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
	}//CalculateDistance//

	//EstimateDistance//
	double LocalDynamicMap::EstimateDistance(const NodeIdType& nodeId, double receiveRssiDbm, double n)
	{
		double A = 0;
		if (nodeId > 150) {
			A = AForBeacon;
		} else {
			A = AForCar;
		}
		return pow(10, ((A - receiveRssiDbm) / (10 * n)));
	}//EstimateDistance//

	//GetBeaconCoordinateX/
	int LocalDynamicMap::GetBeaconCoordinateX(const NodeIdType& nodeId)
	{
		map<NodeIdType, BeaconCoordinate>::iterator it = beacon_table.find(nodeId);
		return it->second.latitude;
	}//GetBeaconCoordinateX/

	//GetBeaconCoordinateY/
	int LocalDynamicMap::GetBeaconCoordinateY(const NodeIdType& nodeId)
	{
		map<NodeIdType, BeaconCoordinate>::iterator it = beacon_table.find(nodeId);
		return it->second.longitude;
	}//GetBeaconCoordinateY/

	//convertCMtoM//
	double LocalDynamicMap::convertCMtoM(unsigned short int distance)
	{
		return (double)(distance) / 100;
	}//convertCMtoM//

	//convertMMtoM//
	double LocalDynamicMap::convertMMtoM(int pos)
	{
		return (double)(pos) / 1000;
	}//convertMMtoM//

	//convertMilliToNanoSec//
	TimeType LocalDynamicMap::convertMilliToNanoSec(unsigned int milliSecond)
	{
		return static_cast<TimeType>(milliSecond) * MILLI_SECOND;
	}//convertMilliToNanoSec//

	//GetMyNodeId//
	NodeIdType LocalDynamicMap::GetMyNodeId()
	{
		return myNodeId;
	}//GetMyNodeId//

	//DeleteEntry//
	void LocalDynamicMap::DeleteEntry(const NodeIdType& nodeId)
	{
		local_table.erase(nodeId);
		return;
	}//DeleteEntry//

	//isEntry//
	bool LocalDynamicMap::isEntry(const NodeIdType& nodeId)
	{
		map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
		if (it == local_table.end()){
			return false;
		}
		return true;
	}//isEntry//

	//PrintDistance//
	void LocalDynamicMap::PrintDistance(const NodeIdType& nodeId, int iter, double n)
	{
		ostringstream oss1;
		if (nodeId <= 150) {
			oss1 << "nの評価（車両）.txt";
		} else {
			oss1 << "nの評価（ビーコン）.txt";
		}
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);

		map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
		VehicleInformation viList = it->second[iter];
		ScenSim::ObjectMobilityPosition omp;
		mobilityModelPtr->GetPositionForTime(viList.receiveTime, omp); // M

		double d1 = CalculateDistance(convertMMtoM(viList.latitude), convertMMtoM(viList.longitude), omp.X_PositionMeters(), omp.Y_PositionMeters()); // 真の距離
		double d2 = EstimateDistance(nodeId, viList.receiveRssiDbm, n); // 動的
		double d3 = EstimateDistance(nodeId, viList.receiveRssiDbm, 2); // 静的

		//ofs1 << nodeId << " ";
		ofs1 << viList.receiveRssiDbm << " " << n; // rssiと伝搬損失指数
		ofs1 << " " << std::fabs(d1 - d2) << " " << std::fabs(d1 - d3) << endl;
		//ofs1 << " " << d1 << " " << d2 <<  " " << d3 << endl;
		//ofs1 << " " << viList.receiveTime / MICRO_SECOND << " " << viList.sendTime / MICRO_SECOND; //　時間

	}//PrintDistance//

	//PrintForTest//
	void LocalDynamicMap::PrintForTest()
	{
		ostringstream oss1;
		oss1 << "データベース.txt";
		std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);
		ofs1 << "出力開始" << endl;

		map<NodeIdType, vector<VehicleInformation>>::iterator it;
		for(it = local_table.begin(); it != local_table.end(); it++) {
			for (int i = 0; i < it->second.size(); i++) {
				VehicleInformation viList = it->second[i];
				ofs1 << it->first << ", " << viList.receiveTime / MILLI_SECOND << ", " << viList.receiveRssiDbm << ", " << viList.n << endl;
			}
		}
	}//PrintForTest//

	/*
	//KalmanfilterForCar//
	void LocalDynamicMap::KalmanfilterForVehicleToVehicle(
		const NodeIdType& nodeId,
		int latitude,
		int longitude,
		double& receiveRssiDbm,
		double& P,
		TimeType receiveTime)
		{
		if (myNodeId == 8) {
			ostringstream oss1;
			oss1 << "車フィルタリング前" << nodeId << ".txt";
			std::ofstream ofs1(oss1.str(), std::ios::out | std::ios::app);
			ofs1 << receiveRssiDbm << endl;
		}
		double Q = 5.94, R = 23.68; //　実験から算出したがこの値はそれほど重要ではない
		double xhatm, xhat, Pm, G;

		if (isEntry(nodeId)){
			map<NodeIdType, vector<VehicleInformation>>::iterator it = local_table.find(nodeId);
			VehicleInformation viList = it->second.back();
			if (receiveTime - viList.receiveTime > 200 * MILLI_SECOND) {
				xhat = 0;
				P = 1000;
			} else {
				xhat = viList.receiveRssiDbm;
				P = viList.P;
			}
		} else {
			xhat = 0;
			P = 1000;
		}

		xhatm = xhat;
		Pm = P + Q;
		G = Pm / (Pm + R);
		receiveRssiDbm = xhatm + G * (receiveRssiDbm - xhatm);
		P = (1 - G) * Pm;

		if (myNodeId == 8) {
			ostringstream oss2;
			oss2 << "車フィルタリング後" << nodeId << ".txt";
			std::ofstream ofs2(oss2.str(), std::ios::out | std::ios::app);
			ofs2 << receiveTime/MILLI_SECOND << ", " << receiveRssiDbm << ", " << xhatm << ", " << P << ", " << G << endl;
		}
	}//kalmanfilterForCar//

	//CalculateMyPositionX//
	double LocalDynamicMap::CalculateMyPositionX(double x, double speed, double heading, TimeType interval_time)
	{
		return x + speed * cos(heading) * (double)(interval_time/SECOND);
	}//CalculateMyPositionX//

	//CalculateMyPositionY//
	double LocalDynamicMap::CalculateMyPositionY(double y, double speed, double heading, TimeType interval_time)
	{
		return y + speed * sin(heading) * (double)(interval_time/SECOND);
	}//CalculateMyPositionY//
	*/
}
